#include "pid_controller.hpp"
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs

/**
 * @file pid_controller.cpp
 * @brief Implementation of the PIDController class.
 */

PIDController::PIDController(float kp, float ki, float kd, float max_output, float integral_limit)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), integral_limit_(integral_limit) {}

/**
 * @brief Calculates the normalized control signal.
 */
float PIDController::update(float setpoint, float measurement) {
    // 1. Calculate Error
    float error = setpoint - measurement;

    // 2. Proportional Term
    float p_term = kp_ * error;

    // 3. Integral Term (with Anti-Windup)
    integral_sum_ += error;
    integral_sum_ = std::min(integral_limit_, std::max(-integral_limit_, integral_sum_));
    float i_term = ki_ * integral_sum_;

    // 4. Derivative Term (Applied on the measurement, not the error)
    // d(measurement)/dt approx (measurement - last_measurement) / dt
    float d_measurement = last_error_-error;
    float d_term = -kd_ * d_measurement;
    
    // 5. Total Control Output (Current is normalized to 1.0 here)
    float raw_output = p_term + i_term + d_term;

    // 6. Output Clamping (Normalized to +/- 1.0)
    float output = std::min(max_output_, std::max(-max_output_, raw_output));

    // 7. Update History
    last_measurement_ = measurement;
    last_error_ = error;

    return output;
}
