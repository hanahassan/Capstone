#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node provide a simple graphical visualization of ContinuO to check the behavor of the policy before sending commands on the real robot.
Author : Florent Pralong
Date of creation : 23/06/2026
Version : 1.0
"""
# ---------------------------
# IMPORTS
# ---------------------------

import rospy
import tkinter as tk
from sensor_msgs.msg import JointState
import math


class CommandsVisuNode:

    # ---------------------------
    # INITIALIZATION
    # ---------------------------

    def __init__(self):
        rospy.init_node("commands_visu_node")

        # ---------------------------
        # PARAMETERS
        # ---------------------------

        self.topic_name = "/joint_targets_rl"
        self.joint_values = {}

        self.root = tk.Tk()
        self.root.title("Joint Target Viewer")

        self.labels = {}

        # Visualization
        self.canvas_width = 700
        self.canvas_height = 500
        self.body_center_x = 350
        self.body_center_y = 250
        self.link1H = 80
        self.link2H = 100
        self.link3H = 50
        self.link1F = 100
        self.link2F = 70

        # Subscriber
        rospy.Subscriber(
            self.topic_name,
            JointState,
            self.joint_callback,
            queue_size=1
        )

        # Initialization of the interface
        self.build_interface()
        self.update_gui()

    # Interface build
    def build_interface(self):
        title = tk.Label(
            self.root,
            text="Received Joints Commands",
            font=("Arial", 16, "bold")
        )
        title.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        headers = ["Joint", "Target position [rad]"]

        for col, text in enumerate(headers):
            label = tk.Label(
                self.root,
                text=text,
                font=("Arial", 12, "bold"),
                borderwidth=1,
                relief="solid",
                width=20
            )
            label.grid(row=1, column=col, padx=2, pady=2)

        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="white"
        )
        self.canvas.grid(row=0, column=3, rowspan=30, padx=20, pady=10)

    # ROS Callback
    def joint_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_values[name] = position

    # Interface update 
    def update_gui(self):
        row = 2

        for joint_name in sorted(self.joint_values.keys()):
            if joint_name not in self.labels:
                name_label = tk.Label(
                    self.root,
                    text=joint_name,
                    font=("Arial", 11),
                    borderwidth=1,
                    relief="solid",
                    width=20
                )
                value_label = tk.Label(
                    self.root,
                    text="0.000",
                    font=("Arial", 11),
                    borderwidth=1,
                    relief="solid",
                    width=20
                )

                name_label.grid(row=row, column=0, padx=2, pady=2)
                value_label.grid(row=row, column=1, padx=2, pady=2)

                self.labels[joint_name] = value_label

            self.labels[joint_name].config(
                text=f"{self.joint_values[joint_name]: .4f}"
            )

            row += 1

        self.draw_robot()

        self.root.after(50, self.update_gui)

    # MAIN Loop
    def run(self):
        self.root.mainloop()

    # Robot visualization
    def draw_robot(self):
        self.canvas.delete("all")

        cx = self.body_center_x
        cy = self.body_center_y

        # Robot base
        self.canvas.create_rectangle(
            cx - 100,
            cy - 30,
            cx + 100,
            cy + 30,
            width=3
        )
        
        self.canvas.create_line(cx + 100, cy + 30, cx + 200, cy - 10, width=3)
        self.canvas.create_line(cx + 200, cy - 10, cx + 200, cy - 70, width=3)
        self.canvas.create_line(cx + 100, cy - 30, cx + 200, cy - 70, width=3)

        self.canvas.create_line(cx - 0, cy - 70, cx + 200, cy - 70, width=3)
        self.canvas.create_line(cx - 100, cy - 30, cx - 0, cy - 70, width=3)

        self.canvas.create_line(cx - 100, cy + 30, cx - 0, cy - 10, width=3, dash=1)

        self.canvas.create_line(cx - 0, cy - 10, cx - 0, cy - 70, width=3, dash=1)
        self.canvas.create_line(cx - 0, cy - 10, cx + 200, cy - 10, width=3, dash=1)

        # Front legs declaration
        Flegs = {
            "FL": (cx + 200, cy - 10, 1),
            "FR": (cx + 100, cy + 30, 1)
        }

        # Hind legs declaration
        Hlegs = {
            "HL": (cx - 0, cy - 10, 1),
            "HR": (cx - 100, cy + 30, 1)
        }

        # Hind legs drawing
        for leg, (x0, y0, direction) in Hlegs.items():
            haa = self.joint_values.get(f"{leg}_HAA", 0.0)
            hfe = self.joint_values.get(f"{leg}_HFE", 0.0)
            kfe = self.joint_values.get(f"{leg}_KFE", 0.0)
            afe = self.joint_values.get(f"{leg}_AFE", 0.0)

           
            angle1 = direction * math.pi / 2 + hfe
            angle2 = angle1 + kfe
            angle3 = angle2 + afe

            x1 = x0 + self.link1H * math.cos(angle1)
            y1 = y0 + self.link1H * math.sin(angle1)

            x2 = x1 + self.link2H * math.cos(angle2)
            y2 = y1 + self.link2H * math.sin(angle2)

            x3 = x2 + self.link3H * math.cos(angle3)
            y3 = y2 + self.link3H * math.sin(angle3)

            # Hips are represented by an offset
            lateral_offset_x = 50 * math.sin(haa)
            lateral_offset_y = 20 * math.sin(haa)
            x0 += lateral_offset_x
            x1 += lateral_offset_x
            x2 += lateral_offset_x
            x3 += lateral_offset_x

            y0 -= lateral_offset_y
            y1 -= lateral_offset_y
            y2 -= lateral_offset_y
            y3 -= lateral_offset_y


            # Legs
            self.canvas.create_line(x0, y0, x1, y1, width=4)
            self.canvas.create_line(x1, y1, x2, y2, width=4)
            self.canvas.create_line(x1, y1, x2, y2, width=4)
            self.canvas.create_line(x2, y2, x3, y3, width=4)

            # Joints 
            self.canvas.create_oval(x0 - 6, y0 - 6, x0 + 6, y0 + 6, fill="black")
            self.canvas.create_oval(x1 - 5, y1 - 5, x1 + 5, y1 + 5, fill="gray")
            self.canvas.create_oval(x2 - 5, y2 - 5, x2 + 5, y2 + 5, fill="grey")
            self.canvas.create_oval(x3 - 5, y3 - 5, x3 + 5, y3 + 5, fill="red")

            # Legs labels
            self.canvas.create_text(x0 - 25, y0 - 15, text=leg)


        # Front legs drawing
        for leg, (x0, y0, direction) in Flegs.items():
            haa = self.joint_values.get(f"{leg}_HAA", 0.0)
            hfe = self.joint_values.get(f"{leg}_HFE", 0.0)
            kfe = self.joint_values.get(f"{leg}_KFE", 0.0)

            angle1 = direction * math.pi / 2 + hfe
            angle2 = angle1 + kfe

            x1 = x0 + self.link1F * math.cos(angle1)
            y1 = y0 + self.link1F * math.sin(angle1)

            x2 = x1 + self.link2F * math.cos(angle2)
            y2 = y1 + self.link2F * math.sin(angle2)

            # Hips are represented by an offset
            lateral_offset_x = 50 * math.sin(haa)
            lateral_offset_y = 20 * math.sin(haa)
            x0 += lateral_offset_x
            x1 += lateral_offset_x
            x2 += lateral_offset_x

            y0 -= lateral_offset_y
            y1 -= lateral_offset_y
            y2 -= lateral_offset_y


            # Legs
            self.canvas.create_line(x0, y0, x1, y1, width=4)
            self.canvas.create_line(x1, y1, x2, y2, width=4)
            self.canvas.create_line(x1, y1, x2, y2, width=4)

            # Joints
            self.canvas.create_oval(x0 - 6, y0 - 6, x0 + 6, y0 + 6, fill="black")
            self.canvas.create_oval(x1 - 5, y1 - 5, x1 + 5, y1 + 5, fill="gray")
            self.canvas.create_oval(x2 - 5, y2 - 5, x2 + 5, y2 + 5, fill="red")

            # Leg label
            self.canvas.create_text(x0 - 25, y0 - 10, text=leg)


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    try:
        viewer = CommandsVisuNode()
        viewer.run()
    except rospy.ROSInterruptException:
        pass