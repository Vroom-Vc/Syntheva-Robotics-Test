#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP

#include <cmath>

/**
 * @file motion_planner.hpp
 * @brief Defines the MotionPlanner class for generating a smooth, pre-calculated S-Curve 
 * velocity profile based on total displacement and time.
 */
class MotionPlanner {
public:
    /**
     * @brief Constructor.
     */
    MotionPlanner();

    /**
     * @brief Set the target motion parameters and prepare the profile.
     * @param target_displacement The total angle/distance to move (degrees).
     * @param total_time The duration over which the motion should occur (seconds).
     */
    void set_target(float target_displacement, float total_time);

    /**
     * @brief Calculates the desired instantaneous velocity based on elapsed time.
     * @param elapsed_time The time passed since the motion started (seconds).
     * @return The commanded velocity at this time step (degrees/s).
     */
    float update(float elapsed_time) const;

private:
    float target_displacement_ = 0.0f;
    float total_time_ = 1.0f;
    float sign_ = 1.0f; // Stores the direction of movement (+1 or -1)

    /**
     * @brief Calculates the normalized velocity profile (S-curve).
     * Uses the Smootherstep polynomial derivative: V'(tau) = 30*tau^2 - 60*tau^3 + 30*tau^4
     * This function ranges from 0 to a peak value, and integrates to 1 over tau = 0 to 1.
     * @param tau Normalized time (elapsed_time / total_time), between 0.0 and 1.0.
     * @return Normalized velocity scaling factor.
     */
    float get_normalized_velocity(float tau) const;
};

#endif // MOTION_PLANNER_HPP
