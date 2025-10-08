#include "pid_controller.hpp"
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs

/**
 * @file pid_controller.cpp
 * @brief Implementation of the PIDController class.
 */

/**
 * @brief Constructor for the PID Controller.
 */
PIDController::PIDController(float dt, float kp, float ki, float kd, float out_limit, float integral_limit) 
    : dt_(dt), 
      Kp_(kp), Ki_(ki), Kd_(kd),
      output_limit_(out_limit), integral_limit_(integral_limit) {
    
    // Initialize state variables
    reset();
}

/**
 * @brief Resets the state variables of the controller.
 */
void PIDController::reset() {
    integral_sum_ = 0.0f;
    prev_error_ = 0.0f;
    prev_measured_value_ = 0.0f;
}

/**
 * @brief Executes one iteration of the PID control loop.
 */
float PIDController::update(float setpoint, float measured_value) {
    // 1. Calculate Error
    float error = setpoint - measured_value;

    // --- P Term (Proportional) ---
    float p_term = Kp_ * error;

    // --- I Term (Integral) ---
    // Accumulate the integral sum
    integral_sum_ += error * dt_;
    
    // Apply Anti-Windup: Limit the integral sum to prevent excessive buildup
    if (integral_sum_ > integral_limit_) {
        integral_sum_ = integral_limit_;
    } else if (integral_sum_ < -integral_limit_) {
        integral_sum_ = -integral_limit_;
    }

    float i_term = Ki_ * integral_sum_;
    
    // --- D Term (Derivative) ---
    // Use Derivative on Error (DoE):
    // float derivative = (error - prev_error_) / dt_;
    
    // Use Derivative on Measurement (DoM) to avoid overshoot from setpoint changes:
    float derivative = -(measured_value - prev_measured_value_) / dt_;
    
    float d_term = Kd_ * derivative;
    
    // 2. Calculate Total Output
    float output = p_term + i_term + d_term;

    // 3. Apply Saturation (Output Limit)
    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }
    
    // 4. Update State for next iteration
    // prev_error_ = error; // Only needed for Derivative on Error
    prev_measured_value_ = measured_value;

    return output;
}

// --- Setter Implementations ---

void PIDController::set_gains(float kp, float ki, float kd) {
    Kp_ = kp;
    Ki_ = ki;
    Kd_ = kd;
}

void PIDController::set_output_limit(float out_limit) {
    if (out_limit >= 0.0f) {
        output_limit_ = out_limit;
    }
}

void PIDController::set_integral_limit(float integral_limit) {
    if (integral_limit >= 0.0f) {
        integral_limit_ = integral_limit;
    }
}
