#include "kalman_filter.hpp"
#include "velocity_controller.hpp"
#include "pid_controller.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <random>

/**

 * @brief Main simulation file for integrating the Kalman Filter, Velocity Controller, and PID Controller.
 * * This file simulates a motor system controlled in a closed loop to track a target velocity.
 */


struct MotorModel {
    // Physical Parameters (Simple DC Motor Approximation)
    const float INERTIA = 0.01f;     // Motor rotor inertia (kg*m^2, or equivalent for our units)
    const float KT = 0.5f;           // Torque constant (Torque per unit Current)
    const float DAMPING = 0.001f;    // Viscous friction coefficient

    // State
    float position_deg = 0.0f;       // Current angular position (degrees)
    float velocity_dps = 0.0f;       // Current angular velocity (degrees/s)

    float update(float current_command, float dt, float noise_stddev) {
        // 1. Calculate Torque
        float applied_torque = KT * current_command;
        
        // 2. Apply Viscous Friction (Proportional to velocity)
        float friction_torque = DAMPING * velocity_dps;
        float net_torque = applied_torque - friction_torque;

        // 3. Calculate Angular Acceleration (alpha = Torque / Inertia)
        float angular_acceleration = net_torque / INERTIA;

        // 4. Integrate Velocity (Euler integration)
        velocity_dps += angular_acceleration * dt;

        // 5. Integrate Position
        position_deg += velocity_dps * dt;

        // --- Simulate Sensor Measurement ---
        
        // Setup random number generator for Gaussian noise
        static std::random_device rd;
        static std::mt19937 gen(rd());
        // Use a small fixed seed for reproducibility in tests:
        // static std::mt19937 gen(12345);
        std::normal_distribution<float> d(0.0f, noise_stddev);
        
        // Measured position = True position + Gaussian noise
        float measured_position = position_deg + d(gen);
        
        return measured_position;
    }
};

void run_simulation() {
    // --- 1. Simulation Parameters ---
    const float DT = 0.001f;           // Control loop time step (1000 Hz)
    const float SIM_DURATION = 5.0f;   // Total simulation duration (seconds)
    const int NUM_STEPS = static_cast<int>(SIM_DURATION / DT);
    const float STEP_CHANGE_TIME = 1.0f; // Time when target velocity changes

    // --- 2. Instantiate Components ---

    // a) Kalman Filter Parameters
    const float KF_Q_POS = 0.01f;      // Process noise covariance (Position)
    const float KF_Q_VEL = 0.1f;       // Process noise covariance (Velocity)
    const float KF_R_POS = 1.0f;       // Measurement noise covariance (Position)
    KalmanFilter kf(DT, KF_Q_POS, KF_Q_VEL, KF_R_POS);
    

    // b) Motion Planner
    MotionPlanner planner;
    planner.set_target(TARGET_DISPLACEMENT, MOTION_DURATION); 

    // c) PID Controller Parameters
    const float KP = 0.15f;            // Proportional Gain
    const float KI = 5.0f;             // Integral Gain
    const float KD = 0.001f;           // Derivative Gain
    const float MAX_NORMALIZED_OUTPUT = 1.0f; // Limit the control signal to +/- 1.0
    const float INTEGRAL_LIMIT = 5.0f; 
    
    // PID outputs the normalized control signal
    PIDController pid(DT, KP, KI, KD, MAX_NORMALIZED_OUTPUT, INTEGRAL_LIMIT);

    // d) Motor Model (The physical system)
    const float MOTOR_SENSOR_NOISE_STD = 0.1f; // 0.1 degrees of sensor noise
    const float MAX_PHYSICAL_SPEED_DPS = 360.0f; 
    MotorModel motor;

    // Initial target velocity
    float motion_elapsed_time = 0.0f;

    // --- 4. Main Simulation Loop ---
    std::cout << "Starting Motor Control Simulation " << std::endl;

    for (int i = 0; i < NUM_STEPS; ++i) {

        float current_time = i * DT;

        // Step 4a: Motion Planning
        float ramped_velocity = 0.0f;
        if (current_time >= MOTION_START_TIME) {
            // Update elapsed time for the current motion
            motion_elapsed_time += DT;
            
            // Get the velocity command from the pre-calculated S-curve profile
            ramped_velocity = planner.update(motion_elapsed_time);
        }

        // Step 4b: State Estimation (Kalman Filter)
        // We first update the motor with 0 current to get the noisy measurement of the previous state
        float measured_position = motor.update(0.0f, 0.0f, MOTOR_SENSOR_NOISE_STD); 
        kf.update(measured_position, motor.position_deg); 
        float estimated_velocity = kf.get_estimated_velocity();
        

        // Step 4c: PID Control - Output the normalized control signal
        float control_signal = pid.update(ramped_velocity, estimated_velocity);

        control_signal = control_signal / MAX_PHYSICAL_SPEED_DPS; // normalize the signal to range (-1,1)
        
        // Step 4d: Motor Update - The motor model scales the normalized signal to real physics
        // We use the control signal to drive the motor over the current time step DT
        motor.update(control_signal, DT, MOTOR_SENSOR_NOISE_STD);


        // Data Logging
        if (i % 50 == 0) { 
            float error = ramped_velocity - estimated_velocity;

            std::cout << current_time << " | " 
                      << motor.position_deg << " | " 
                      << ramped_velocity << " | " 
                      << current_command << " | " 
                      << motor.velocity_dps << " | "
                      << estimated_velocity << " | "
                      << error 
                      << std::endl;
        }

       
    }


    std::cout << "Simulation finished. Final Position: " << motor.position_deg << " degrees." << std::endl;
    std::cout << "Final True Velocity: " << motor.velocity_dps << " dps." << std::endl;
    std::cout << "Final Estimated Velocity: " << kf.get_estimated_velocity() << " dps." << std::endl;
}

int main() {
    run_simulation();
    return 0;
}
