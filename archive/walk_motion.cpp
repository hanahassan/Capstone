#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>
#include <thread>
#include <sstream>
#include <memory>
#include <iomanip> // For std::fixed and std::setprecision

#include "myactuator_rmd/actuator_interface.hpp"  
#include "myactuator_rmd/driver/can_driver.hpp" 

// Define variables
std::vector<int> motor_IDs = {1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15, 16, 17};
double t = 0;
double timestep = 0.012;

int kp = 50;
int kd = 819;
int kd2 = 1400;

std::string file_pos = "Data_joint_positions_walk_0.002.txt";
std::string file_vel = "Data_joint_velocities_walk_0.002.txt";

int bus_error_no = 0;
int step_error_no = 0;

double v2_reduction = 9;
double v3_reduction = 6.2;

// Function to read a line from a file and convert it to a vector of doubles
std::vector<double> read_line_as_array(const std::string& filename, int line_number) {
    std::ifstream file(filename);
    std::vector<double> result;
    if (!file.is_open()) {
        std::cerr << "File '" << filename << "' not found.\n";
        return result;
    }

    std::string line;
    int current_line = 0;
    while (getline(file, line)) {
        current_line++;
        if (current_line == line_number) {
            std::stringstream ss(line);
            double value;
            while (ss >> value) {
                result.push_back(value);
            }
            break;
        }
    }
    return result;
}

// Function to convert from model to motor position
double model_to_motor_pos(int motor_id, double model_pos) {
    switch (motor_id) {
        case 1: case 2:
            return -model_pos;
        case 3:
            return model_pos - M_PI / 180.0 * 96.790;
        case 4:
            return -model_pos + M_PI / 180.0 * 96.720;
        case 5:
            return model_pos - M_PI / 180.0 * -193.090;
        case 6:
            return -model_pos + M_PI / 180.0 * -193.290;
        case 10: case 11:
            return model_pos;
        case 12:
            return -model_pos + M_PI / 180.0 * -80.650;
        case 13:
            return model_pos - M_PI / 180.0 * -81.890;
        case 14:
            return -model_pos + M_PI / 180.0 * 182.540;
        case 15:
            return model_pos - M_PI / 180.0 * 182.940;
        case 16:
            return -model_pos + M_PI / 180.0 * -199.990;
        case 17:
            return model_pos - M_PI / 180.0 * -200.610;
        default:
            return model_pos;
    }
}

int main() {
    // Setup CAN channels and motors.
    std::map<int, std::unique_ptr<myactuator_rmd::motion_mode::ActuatorInterface>> motor;

    // Front right leg.
    myactuator_rmd::CanDriver can_fr("can0");
    motor[1] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fr, 1); // Shoulder
    motor[3] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fr, 3); // Hip
    motor[5] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fr, 5); // Knee

    // Front left leg.
    myactuator_rmd::CanDriver can_fl("can1");
    motor[2] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fl, 2); // Shoulder
    motor[4] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fl, 4); // Hip
    motor[6] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fl, 6); // Knee

    // Front right leg.
    myactuator_rmd::CanDriver can_hr("can2");
    motor[11] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 11); // Shoulder
    motor[13] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 13); // Shoulder
    motor[15] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 15); // Hip
    motor[17] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 17); // Knee

    // Front left leg.
    myactuator_rmd::CanDriver can_hl("can3");
    motor[10] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 10); // Shoulder
    motor[12] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 12); // Shoulder
    motor[14] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 14); // Hip
    motor[16] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 16); // Knee

    // Read initial position of model files
    std::map<int, double> ready_pos;
    std::vector<double> model_ready_pos = read_line_as_array(file_pos, 1);

    // Set up ready positions for each motor
    ready_pos[1] = model_to_motor_pos(1, model_ready_pos[0]);
    ready_pos[3] = model_to_motor_pos(3, model_ready_pos[1]);
    ready_pos[5] = model_to_motor_pos(5, model_ready_pos[2]);

    // Add other legs...
    ready_pos[2] = model_to_motor_pos(2, model_ready_pos[3]);
    ready_pos[4] = model_to_motor_pos(4, model_ready_pos[4]);
    ready_pos[6] = model_to_motor_pos(6, model_ready_pos[5]);
    ready_pos[10] = model_to_motor_pos(10, model_ready_pos[6]);
    ready_pos[12] = model_to_motor_pos(12, model_ready_pos[7]);
    ready_pos[14] = model_to_motor_pos(14, model_ready_pos[8]);
    ready_pos[16] = model_to_motor_pos(16, model_ready_pos[9]);
    ready_pos[11] = model_to_motor_pos(11, model_ready_pos[10]);
    ready_pos[13] = model_to_motor_pos(13, model_ready_pos[11]);
    ready_pos[15] = model_to_motor_pos(15, model_ready_pos[12]);
    ready_pos[17] = model_to_motor_pos(17, model_ready_pos[13]);

    // Example CAN control loop
    for (int id : motor_IDs) {
        try {
            // Assuming `motionModeControl` method is implemented in `ActuatorInterface`
            motor[id]->motionModeControl(ready_pos[id], 0, kp, kd2, 0);
        } catch (const std::exception& e) {
            bus_error_no++;
            std::cerr << "Bus Error: " << bus_error_no << "\n";
        }
    }

    return 0;
}
