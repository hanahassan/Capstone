#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node runs the control policy
Author : Florent Pralong
Date of creation : 28/06/2026
Version : 2.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import os
import numpy as np
import rospy
import rospkg
import yaml
import threading

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

try:
    import onnxruntime as ort
except Exception:
    ort = None

# ---------------------------
# Utility functions
# ---------------------------

def quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw),     2 * (qx*qz + qy*qw)],
        [2 * (qx*qy + qz*qw),     1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)],
        [2 * (qx*qz - qy*qw),     2 * (qy*qz + qx*qw),     1 - 2 * (qx*qx + qy*qy)]
    ], dtype=np.float32)


def resolve_model_path(path_param):
    if path_param:
        if os.path.isabs(path_param):
            return path_param
        pkg = rospkg.RosPack().get_path("quadruped_control")
        return os.path.join(pkg, path_param)
    pkg = rospkg.RosPack().get_path("quadruped_control")
    return os.path.join(pkg, "policies", "flat_pushing_pt2.onnx")


class PolicyNodeReal:

    # ---------------------------
    # INITIALIZATION
    # ---------------------------

    def __init__(self):
        rospy.init_node("policy_node_real")

        # ---------------------------
        # PARAMETERS
        # ---------------------------

        # Bypass transformations for testing purposes
        self.bypass_base_lin_vel_calc = rospy.get_param("~bypass_base_lin_vel_calc", True)
        self.bypass_base_gravity_trans = rospy.get_param("~bypass_base_gravity_trans", True)

        # Joint order and initial position ------------------
        self.joint_order = rospy.get_param("~joint_order", [
            "FL_HAA", "FL_HFE", "FL_KFE",
            "FR_HAA", "FR_HFE", "FR_KFE",
            "HL_HAA", "HL_HFE", "HL_KFE", "HL_AFE",
            "HR_HAA", "HR_HFE", "HR_KFE", "HR_AFE",
        ])

        self.joint_order_obs = rospy.get_param("~joint_order_obs", [
            "FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA",
            "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE",
            "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE",
            "HL_AFE", "HR_AFE",
        ])

        self.model_q0 = rospy.get_param("~model_q0", {
            "FL_HAA": 0.0, "FR_HAA": 0.0, "HL_HAA": 0.0, "HR_HAA": 0.0,
            "FL_HFE": 0.4102, "FR_HFE": 0.4102, "HL_HFE": -0.6981, "HR_HFE": -0.6981,
            "FL_KFE": -1.2716, "FR_KFE": -1.2716, "HL_KFE": 1.676, "HR_KFE": 1.676,
            "HL_AFE": -1.7219, "HR_AFE": -1.7219,
        })
        # ---------------------------------------------------------

        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.output_topic = rospy.get_param("~output_topic", "/joint_targets_rl")
        self.episode_len_s = float(rospy.get_param("~episode_len_s", 6.0))
        self.action_order_model = rospy.get_param("~action_order_model", list(self.joint_order_obs))

        # latest value for each topic
        self.latest_imu = None
        self.latest_cmd = np.zeros(4, dtype=np.float32)
        self.latest_joint_state = None
        self.latest_height_scan_raw = np.zeros(187, dtype=np.float32)
        self.latest_ceiling_height_scan_raw = np.zeros(187, dtype=np.float32)
        self.last_action_model = np.zeros(14, dtype=np.float32)

        self.episode_start = None
        self.policy_started = False

        self.policy_lock = threading.RLock()

        self.policies_config_path = rospy.get_param(
            "~policies_config_path",
            os.path.join(rospkg.RosPack().get_path("quadruped_control"), "config", "policies.yaml")
        )

        self.policies = {}
        self.active_policy_id = None
        self.active_policy = None

        # Load the policies from the .yaml config file and switch to the default policy
        self.load_policies_config()
        self.switch_policy(rospy.get_param("~initial_policy_name", "flat"))

        # IMU parameters

        self.imu_acc_buffer = []
        self.imu_time_buffer = []
        self.imu_est_lin_vel = np.zeros(3, dtype=np.float32)

        # ---------------------------
        # ROS Interface
        # ---------------------------
        # Publisher
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)

        # Subscribers
        rospy.Subscriber("/imu", Imu, self.cb_imu, queue_size=50)
        rospy.Subscriber("/cmd", Float32MultiArray, self.cb_cmd, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states, queue_size=50)
        rospy.Subscriber("/lidar", Float32MultiArray, self.cb_lidar, queue_size=10)
        rospy.Subscriber("/switch_cp", String, self.cb_switch_cp, queue_size=10)
        
        rospy.loginfo("policy_node_real ready: active_policy=%s, hz=%.1f",
              self.active_policy_id,
              self.rate_hz)
        
        # Main loop execution
        self.loop()

    # ---------------------------
    # ROS CALLBACK FUNCTIONS
    # ---------------------------  
    
    # Define what happen when a change is detected on the /imu topic
    def cb_imu(self, msg):
        self.latest_imu = msg

    # Define what happen when a change is detected on the /cmd topic
    def cb_cmd(self, msg):
        cmd = np.zeros(4, dtype=np.float32)
        n = min(4, len(msg.data))
        if n > 0:
            cmd[:n] = np.asarray(msg.data[:n], dtype=np.float32)
        self.latest_cmd = cmd

    # Define what happen when a change is detected on the /joint_states topic
    def cb_joint_states(self, msg):
        self.latest_joint_state = msg

    # Define what happen when a change is detected on the /lidar topic
    def cb_lidar(self, msg):
        ranges = np.asarray(msg.data, dtype=np.float32)

        if ranges.size == 0:
            return

        ranges[~np.isfinite(ranges)] = 0.0

        with self.policy_lock:
            active_policy_id = self.active_policy_id

        # Exception for crouch policy which need two high scanners
        if active_policy_id == "crouch":
            if ranges.size < 188:
                rospy.logwarn_throttle(
                    2.0,
                    "Crouch policy expects 188 lidar values: 187 ceiling height + 1 ground height"
                )
                return

            self.latest_ceiling_height_scan_raw = ranges[:187].astype(np.float32)
            self.latest_height_scan_raw = ranges[187:188].astype(np.float32)

        else:
            self.latest_height_scan_raw = ranges.astype(np.float32)

    # Define what happen when a change is detected on the /lidar topic
    def cb_switch_cp(self, msg):
        policy_name = msg.data.strip()
        self.switch_policy(policy_name)

    # ---------------------------
    # POLICY FUNCTIONS
    # ---------------------------  

    # Use to load the policies configuration (policies.yaml)
    def load_policies_config(self):
        with open(self.policies_config_path, "r") as f:
            data = yaml.safe_load(f)

        for policy_name, cfg in data["policies"].items():
            policy_name = str(policy_name)

            cfg["model_path"] = resolve_model_path(cfg["model_path"])
            cfg["model_format"] = cfg.get("model_format", "onnx").strip().lower()
            cfg["ort_input_name"] = cfg.get("ort_input_name", None)
            cfg["action_mode"] = cfg.get("action_mode", "delta").strip().lower()
            cfg["action_scale"] = float(cfg.get("action_scale", 1.0))
            cfg["obs_dim"] = int(cfg["obs_dim"])
            cfg["joint_order_obs"] = cfg.get("joint_order_obs", self.joint_order_obs)
            cfg["action_order_model"] = cfg.get("action_order_model", self.action_order_model)
            cfg["model_q0"] = cfg.get("model_q0", self.model_q0)
            cfg["model"] = None
            cfg["ort_session"] = None

            # Load the model based on the specified format
            if cfg["model_format"] == "onnx":
                cfg["ort_session"] = ort.InferenceSession(
                    cfg["model_path"],
                    providers=["CPUExecutionProvider"]
                )
                if cfg["ort_input_name"] is None:
                    cfg["ort_input_name"] = cfg["ort_session"].get_inputs()[0].name


            else:
                raise RuntimeError("Unknown model_format: %s" % cfg["model_format"])

            self.policies[policy_name] = cfg

        rospy.loginfo("Loaded %d policies from %s", len(self.policies), self.policies_config_path)

    # Use to switch between policies based on the policy name
    def switch_policy(self, policy_name):
        with self.policy_lock:
            policy_name = str(policy_name).strip()

            if policy_name not in self.policies:
                rospy.logerr("Unknown policy name: %s", policy_name)
                return
            
            if policy_name == self.active_policy_id:
                return

            self.active_policy_id = policy_name
            self.active_policy = self.policies[policy_name]

            self.joint_order_obs = self.active_policy["joint_order_obs"]
            self.action_order_model = self.active_policy["action_order_model"]
            self.model_q0 = self.active_policy["model_q0"]

            self.last_action_model = np.zeros(len(self.action_order_model), dtype=np.float32)
            self.episode_start = None
            self.policy_started = False

            rospy.logwarn("Switched to policy %s: %s",
                        policy_name,
                        self.active_policy.get("name", "unnamed"))
            
    # Use to run the active policy
    def run_policy(self, obs):
        out = self.active_policy["ort_session"].run(
            None,
            {self.active_policy["ort_input_name"]: obs.reshape(1, -1)}
        )[0]

        return np.asarray(out, dtype=np.float32).reshape(-1)


    # --------------------------------
    # POLICY OBSERVATIONS FUNCTIONS
    # --------------------------------

    # This function builds the observation vector (inputs) for the active policy
    def build_obs(self):
        q, dq = self.joint_pos_vel()

        height_scan_dim = int(self.active_policy.get("height_scan_dim", 187))
        ceiling_height_scan_dim = int(self.active_policy.get("ceiling_height_scan_dim", 187))

        height_scan = self.resize_vector(
            self.latest_height_scan_raw,
            height_scan_dim
        )

        ceiling_height_scan = self.resize_vector(
            self.latest_ceiling_height_scan_raw,
            ceiling_height_scan_dim
        )

        term_values = {
            "base_lin_vel": self.imu_base_lin_vel(),
            "base_ang_vel": self.imu_base_ang_vel(),
            "projected_gravity": self.imu_projected_gravity(),
            "pose_commands": self.latest_cmd,
            "joint_pos": q,
            "joint_vel": dq,
            "actions": self.last_action_model,
            "height_scan": height_scan,
            "ceiling_height_scan": ceiling_height_scan,
            "time_remaining_s": np.array([0.0], dtype=np.float32),
        }

        obs_parts = []

        for term in self.active_policy["obs_terms"]:
            if term not in term_values:
                raise RuntimeError("Unknown observation term: %s" % term)
            obs_parts.append(term_values[term])

        obs = np.concatenate(obs_parts).astype(np.float32)

        expected_dim = self.active_policy["obs_dim"]
        if obs.shape[0] != expected_dim:
            raise RuntimeError(
                "observation size is %d, expected %d for policy %s"
                % (obs.shape[0], expected_dim, self.active_policy_id)
            )

        return obs

    # Check if the observation is consistent (only imu data are used because joint states are only published when the control policy is alredy running)
    def obs_ready(self, obs):
        if self.latest_imu is None:
            return False

        return True
    
    # Add the time remaining to the observation vector
    # Done to ensure the remaining time did not get impacted by the software cycle time
    def inject_time_remaining(self, obs):
        obs = obs.copy()

        time_index = 0

        q_dim = {
            "base_lin_vel": 3,
            "base_ang_vel": 3,
            "projected_gravity": 3,
            "pose_commands": 4,
            "joint_pos": 14,
            "joint_vel": 14,
            "actions": len(self.action_order_model),
            "height_scan": int(self.active_policy.get("height_scan_dim", 187)),
            "ceiling_height_scan": int(self.active_policy.get("ceiling_height_scan_dim", 187)),
            "time_remaining_s": 1,
        }

        for term in self.active_policy["obs_terms"]:
            if term == "time_remaining_s":
                obs[time_index] = self.time_remaining()[0]
                return obs
            time_index += q_dim[term]

        return obs


    # -------------------------------------------
    # IMU FUNCTIONS
    # -------------------------------------------
    
    # Extract the base linear velocity from the IMU data using the base linear acceleration and a simple integration method
    def imu_base_lin_vel(self):

        if self.latest_imu is None:
            return self.imu_est_lin_vel.copy()

        now = self.latest_imu.header.stamp
        if now.to_sec() == 0.0:
            now = rospy.Time.now()

        a_msg = self.latest_imu.linear_acceleration

        if self.bypass_base_lin_vel_calc:
            return np.array([a_msg.x, a_msg.y, a_msg.z], dtype=np.float32)
        
        acc = np.array([a_msg.x, a_msg.y, a_msg.z], dtype=np.float32)

        self.imu_time_buffer.append(now)
        self.imu_acc_buffer.append(acc)

        if len(self.imu_time_buffer) > 3:
            self.imu_time_buffer.pop(0)
            self.imu_acc_buffer.pop(0)

        if len(self.imu_time_buffer) < 3:
            return self.imu_est_lin_vel.copy()

        t0 = self.imu_time_buffer[0]
        t1 = self.imu_time_buffer[1]
        t2 = self.imu_time_buffer[2]

        a0 = self.imu_acc_buffer[0]
        a1 = self.imu_acc_buffer[1]
        a2 = self.imu_acc_buffer[2]

        dt01 = (t1 - t0).to_sec()
        dt12 = (t2 - t1).to_sec()
        dt_total = (t2 - t0).to_sec()

        if dt01 <= 0.0 or dt12 <= 0.0 or dt_total <= 0.0:
            self.imu_time_buffer = [now]
            self.imu_acc_buffer = [acc]
            return self.imu_est_lin_vel.copy()

        if dt_total > 0.2:
            self.imu_time_buffer = [now]
            self.imu_acc_buffer = [acc]
            return self.imu_est_lin_vel.copy()

        ratio = dt01 / dt12
        if ratio < 0.5 or ratio > 2.0:
            self.imu_time_buffer = [self.imu_time_buffer[-1]]
            self.imu_acc_buffer = [self.imu_acc_buffer[-1]]
            return self.imu_est_lin_vel.copy()

        delta_v = (dt_total / 6.0) * (a0 + 4.0 * a1 + a2)

        self.imu_est_lin_vel += delta_v

        leak_tau = 2.0
        self.imu_est_lin_vel *= np.exp(-dt_total / leak_tau)

        self.imu_time_buffer = [t2]
        self.imu_acc_buffer = [a2]

        return self.imu_est_lin_vel.copy()

    # Extract the base angular velocity from the IMU data
    def imu_base_ang_vel(self):
        if self.latest_imu is None:
            return np.zeros(3, dtype=np.float32)
        w = self.latest_imu.angular_velocity
        return np.array([w.x, w.y, w.z], dtype=np.float32)

    # Extract the projected gravity vector from the IMU data
    def imu_projected_gravity(self):
        
        if self.latest_imu is None:
            return np.array([0.0, 0.0, -1.0], dtype=np.float32)
        
        q = self.latest_imu.orientation
        
        if self.bypass_base_gravity_trans:
            return np.array([q.x, q.y, q.z], dtype=np.float32)
        
        R = quat_to_rot(q.w, q.x, q.y, q.z)
        return R.dot(np.array([0.0, 0.0, -1.0], dtype=np.float32)).astype(np.float32)

    # -------------------------------------------
    # OTHER FUNCTIONS
    # -------------------------------------------

    # Build the joint position (q) and velocity (dq) vectors from the latest joint state message
    def joint_pos_vel(self):
        q = np.zeros(14, dtype=np.float32)
        dq = np.zeros(14, dtype=np.float32)

        if self.latest_joint_state is None:
            return q, dq

        idx = {name: i for i, name in enumerate(self.latest_joint_state.name)}
        for k, name in enumerate(self.joint_order_obs):
            i = idx.get(name)
            if i is None:
                continue
            if i < len(self.latest_joint_state.position):
                q[k] = float(self.latest_joint_state.position[i]) 
            if i < len(self.latest_joint_state.velocity):
                dq[k] = float(self.latest_joint_state.velocity[i])
                
        return q, dq


    # Calculate the remaining time in the current episode
    def time_remaining(self):
        T = max(self.episode_len_s, 1e-3)

        if self.episode_start is None:
            return np.array([T], dtype=np.float32)

        elapsed = (rospy.Time.now() - self.episode_start).to_sec()
        return np.array([T - (elapsed % T)], dtype=np.float32)

    # Resize height scan or ceiling height scan vectors to the correct dimension expected by the policy.
    def resize_vector(self, data, dim):
        data = np.asarray(data, dtype=np.float32).reshape(-1)

        if dim <= 0:
            return np.zeros(0, dtype=np.float32)

        if data.size == 0:
            return np.zeros(dim, dtype=np.float32)

        if data.size == dim:
            return data.astype(np.float32)

        if dim == 1:
            return np.array([float(np.mean(data))], dtype=np.float32)

        x_old = np.linspace(0.0, 1.0, data.size)
        x_new = np.linspace(0.0, 1.0, dim)

        return np.interp(x_new, x_old, data).astype(np.float32)

    # Build the base joint position vector
    def base_vector(self):
        return np.array([float(self.model_q0.get(name, 0.0)) for name in self.joint_order], dtype=np.float32)

    # Assign each command from the policy to the correct joint
    def map_model_action_to_control(self, action_model):
        action_model = np.asarray(action_model, dtype=np.float32).reshape(-1)
        if action_model.size != len(self.action_order_model):
            raise RuntimeError("policy output size is %d, expected %d" %
                               (action_model.size, len(self.action_order_model)))

        by_name = {name: action_model[i] for i, name in enumerate(self.action_order_model)}
        return np.array([float(by_name[name]) for name in self.joint_order], dtype=np.float32)

    # Convert the joint positions to a ROS JointState message
    def command_to_joint_state(self, positions):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(self.joint_order)
        msg.position = np.asarray(positions, dtype=np.float32).tolist()
        return msg
    
    # ---------------------------
    # MAIN LOOP
    # --------------------------- 
    
    # Main programm
    def loop(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            try:
                with self.policy_lock:
                    obs = self.build_obs()

                    # Stop the policy execution if the policy is not ready (e.g., no valid observation)
                    if not self.obs_ready(obs):
                        rospy.logwarn_throttle(
                            2.0,
                            "Waiting for valid non-zero observation before running policy..."
                        )
                        rate.sleep()
                        continue

                    # Initialize the policy if it hasn't been started yet
                    if not self.policy_started:
                        self.episode_start = rospy.Time.now()
                        self.policy_started = True
                        rospy.loginfo("Valid observation received. Starting policy execution.")

                    obs = self.inject_time_remaining(obs)
                    action_model = self.run_policy(obs)
                    self.last_action_model = action_model.copy()

                    action_ctrl = self.map_model_action_to_control(action_model)

                    action_mode = self.active_policy["action_mode"]
                    action_scale = self.active_policy["action_scale"]
                    q0 = self.base_vector()

                # Actions are applied to the robot based on the specified action mode
                if action_mode == "delta":
                    q_cmd = (action_scale * action_ctrl) + q0
                elif action_mode == "absolute":
                    q_cmd = action_ctrl
                else:
                    raise RuntimeError("unknown action_mode: %s" % action_mode)

                self.pub.publish(self.command_to_joint_state(q_cmd))

            except Exception as exc:
                rospy.logerr_throttle(2.0, "policy loop error: %s", str(exc))

            rate.sleep()


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    try:
        PolicyNodeReal()
    except rospy.ROSInterruptException:
        pass