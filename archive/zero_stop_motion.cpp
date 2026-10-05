#include <iostream>
#include <map>
#include <memory>  // For std::unique_ptr and std::make_unique
#include "myactuator_rmd/actuator_interface.hpp"
#include "myactuator_rmd/driver/can_driver.hpp"

int kp = 82;  
int kd = 819; 

void init_canports() {
    // Assuming this function initializes CAN ports.
}

int main() {
    init_canports();

    // Setup CAN channels and motors.
    std::map<int, std::unique_ptr<myactuator_rmd::motion_mode::ActuatorInterface>> motor;

    // Front right leg.
    myactuator_rmd::CanDriver can_fr("can0");

    // Initialize motors using std::make_unique and store them in the map.
    motor[1] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fr, 1); // Shoulder
    motor[3] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fr, 3); // Hip
    motor[5] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fr, 5); // Knee

    // Front left leg.
    myactuator_rmd::CanDriver can_fl("can1");

    // Initialize motors using std::make_unique and store them in the map.
    motor[2] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fl, 2); // Shoulder
    motor[4] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fl, 4); // Hip
    motor[6] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_fl, 6); // Knee

    // Front left leg.
    myactuator_rmd::CanDriver can_hr("can2");

    // Initialize motors using std::make_unique and store them in the map.
    motor[11] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 11); // Shoulder
    motor[13] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 13); // Shoulder
    motor[15] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 15); // Hip
    motor[17] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hr, 17); // Knee

     // Front left leg.
    myactuator_rmd::CanDriver can_hl("can3");

    // Initialize motors using std::make_unique and store them in the map.
    motor[10] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 10); // Shoulder
    motor[12] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 12); // Shoulder
    motor[14] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 14); // Hip
    motor[16] = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(can_hl, 16); // Knee

    // Create motor IDs list.
    std::vector<int> motor_IDs;
    for (int i = 1; i <= 6; ++i) {
        motor_IDs.push_back(i);
    }
    for (int i = 10; i <= 17; ++i) {
        motor_IDs.push_back(i);
    }


    // Loop through motor IDs to reset encoder zero and reset the motor.
      for (int id : motor_IDs) {
        // Reset encoder zero.
        while (true) {
            try {
                std::int32_t encoder_zero = motor[id]->setCurrentPositionAsEncoderZero();
                std::cout << "Encoder zero set to: " << encoder_zero << " for motor ID: " << id << std::endl;
                break;  // Break the loop once successful.
            } catch (const std::runtime_error& exc) {
                std::cerr << exc.what() << " zeroing: " << id << std::endl;
            }
        }

        // Reset the motor.
        while (true) {
            try {
                motor[id]->reset();
                break;  // Break the loop once successful.
            } catch (const std::runtime_error& exc) {
                std::cerr << exc.what() << " resetting: " << id << std::endl;
            }
        }
    }

    // Wait after reset.
    //std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;

    // Call motion mode control (example)
    //motor[1]->motionModeControl(0.1f, 0.0f, kp, kd, 0.0f); 

    //return 0;
}