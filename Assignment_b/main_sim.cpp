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
    
    // b) Velocity Controller Parameters
    const float MAX_ACCELERATION = 720.0f; // Limit acceleration to 720 deg/s^2
    VelocityController vel_ramp(DT, MAX_ACCELERATION);

    // c) PID Controller Parameters
    const float KP = 0.15f;            // Proportional Gain
    const float KI = 5.0f;             // Integral Gain
    const float KD = 0.001f;           // Derivative Gain (on measurement)
    const float MAX_CURRENT = 10.0f;   // Max output current command (A)
    const float INTEGRAL_LIMIT = 5.0f; // Anti-windup limit
    PIDController pid(DT, KP, KI, KD, MAX_CURRENT, INTEGRAL_LIMIT);

    // d) Motor Model (The physical system)
    const float MOTOR_SENSOR_NOISE_STD = 0.1f; // 0.1 degrees of sensor noise
    MotorModel motor;

    // Initial target velocity
    float target_velocity = 0.0f; 

    // --- 4. Main Simulation Loop ---
    std::cout << "Starting Motor Control Simulation " << std::endl;

    for (int i = 0; i < NUM_STEPS; ++i) {
        float current_time = i * DT;

        // Step 4a: Define Target Profile (Step change at 1.0s)
        if (current_time >= STEP_CHANGE_TIME && target_velocity == 0.0f) {
            target_velocity = 360.0f; // Target 1 full rotation per second
        }

        // Step 4b: Velocity Ramping
        float ramped_velocity = vel_ramp.update(target_velocity);

        // Step 4c: Motor Update and Measurement
        float measured_position = motor.update(pid.update(ramped_velocity, motor.velocity_dps), DT, MOTOR_SENSOR_NOISE_STD);
        // NOTE: In this simplified loop, we use the motor's true velocity in the PID. 
        // A more rigorous loop would use the Kalman filter output as the measured_value.
        // Let's correct this and use the Kalman Filter for *both* velocity and position.

        // Step 4d: State Estimation (Kalman Filter)
        kf.update(measured_position, motor.position_deg); // Measured position for correction

        // Get the estimated velocity from the Kalman Filter
        float estimated_velocity = kf.get_estimated_velocity();
        
        // Step 4e: PID Control (Using ramped velocity and estimated velocity)
        float current_command = pid.update(ramped_velocity, estimated_velocity);
        
        // Step 4f: Motor Update (The motor is driven by the PID output current)
        motor.update(current_command, DT, MOTOR_SENSOR_NOISE_STD);


        // Data Logging
        if (i % 50 == 0) { 
            float error = ramped_velocity - estimated_velocity;

            std::cout << current_time << " | " 
                      << target_velocity << " | " 
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
