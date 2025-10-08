#include "velocity_controller.hpp"
#include <algorithm> 


/**
 * @brief Constructor for the Velocity Controller.
 * @param dt Time step (seconds) of the control loop.
 * @param max_acceleration The maximum allowed rate of change in velocity (degrees/s^2).
 */
VelocityController::VelocityController(float dt, float max_acceleration) 
    : dt_(dt), max_acceleration_(max_acceleration), current_velocity_(0.0f) {}

/**
 * @brief Sets a new maximum acceleration/deceleration limit.
 * @param new_max_accel The new acceleration limit (degrees/s^2). Must be non-negative.
 */
void VelocityController::set_max_acceleration(float new_max_accel) {
    // Ensure acceleration limit is positive or zero
    if (new_max_accel >= 0.0f) {
        max_acceleration_ = new_max_accel;
    }
}

/**
 * @brief Calculates the next safe, ramped velocity based on the desired target velocity.
 * * This is the core ramping logic. It limits the change in velocity per time step (dt)
 * based on the maximum allowed acceleration, ensuring a smooth profile.
 * * @param target_velocity The user-commanded target velocity (degrees/s).
 * @return The next safe, ramped velocity (degrees/s) to command to the motor.
 */
float VelocityController::update(float target_velocity) {
    // 1. Calculate the maximum allowed change in velocity in one time step (dt)
    // This is the maximum acceleration/deceleration limit applied over the time step.
    const float max_delta_v = max_acceleration_ * dt_;
    
    // 2. Calculate the required change (error) to reach the target velocity
    const float delta_v_needed = target_velocity - current_velocity_;

    // 3. Apply the acceleration limits to the required change

    float delta_v_limited;

    if (delta_v_needed > 0) {
        // Acceleration required: limit the increase to max_delta_v
        delta_v_limited = std::min(delta_v_needed, max_delta_v);
    } else {
        // Deceleration required: limit the decrease to -max_delta_v
        delta_v_limited = std::max(delta_v_needed, -max_delta_v);
    }

    // 4. Update the current ramped velocity and store it
    current_velocity_ += delta_v_limited;

    return current_velocity_;
}
