#include "motion_planner.hpp"
#include <algorithm>

/**
 * @file motion_planner.cpp
 * @brief Implementation of the MotionPlanner using a pre-calculated polynomial profile.
 */

MotionPlanner::MotionPlanner() = default;

/**
 * @brief Uses the derivative of the Smootherstep polynomial for a jerk-free S-curve velocity profile.
 * The integral of this function from tau=0 to tau=1 must be 1.0.
 * The derivative of P(tau) = 6*tau^5 - 15*tau^4 + 10*tau^3 is:
 * V'(tau) = 30*tau^4 - 60*tau^3 + 30*tau^2.
 * The integral of V'(tau) is 1.0, so this is already normalized for time.
 */
float MotionPlanner::get_normalized_velocity(float tau) const {
    // Clamp tau between 0 and 1
    tau = std::min(1.0f, std::max(0.0f, tau));

    // V_norm = 30*tau^4 - 60*tau^3 + 30*tau^2
    // This polynomial ensures V_norm(0)=0, V_norm(1)=0, and the derivative V'_norm(0)=0, V'_norm(1)=0 (zero jerk).
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    
    return 30.0f * tau4 - 60.0f * tau3 + 30.0f * tau2;
}

/**
 * @brief Sets the motion parameters.
 */
void MotionPlanner::set_target(float target_displacement, float total_time) {
    if (total_time <= 0.0f) {
        total_time_ = 1.0f;
    } else {
        total_time_ = total_time;
    }
    
    // Determine direction and store absolute displacement
    if (target_displacement >= 0.0f) {
        sign_ = 1.0f;
        target_displacement_ = target_displacement;
    } else {
        sign_ = -1.0f;
        target_displacement_ = -target_displacement;
    }
}

/**
 * @brief Calculates the velocity command at a given time.
 * Velocity V(t) = (Displacement / Time) * V_normalized(t/T)
 */
float MotionPlanner::update(float elapsed_time) const {
    if (elapsed_time >= total_time_) {
        return 0.0f; // Motion is complete
    }
    if (total_time_ == 0.0f || target_displacement_ == 0.0f) {
        return 0.0f;
    }

    // 1. Calculate normalized time (tau)
    float tau = elapsed_time / total_time_;
    
    // 2. Calculate the normalized velocity factor (V_norm)
    float v_norm = get_normalized_velocity(tau);
    
    // 3. Calculate the average velocity, which is the scaling factor
    // Average velocity = Total Displacement / Total Time
    float average_velocity = target_displacement_ / total_time_;
    
    // 4. Calculate the instantaneous velocity
    // V(t) = Average Velocity * Normalized Velocity * Direction
    float instantaneous_velocity = average_velocity * v_norm * sign_;
    
    return instantaneous_velocity;
}
