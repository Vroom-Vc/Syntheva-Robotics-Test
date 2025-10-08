#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

/**
 * @brief Defines the VelocityController class for implementing smooth velocity ramping.
 * * This module ensures that the motor's command velocity changes smoothly over time, 
 * limiting acceleration/deceleration to prevent mechanical shock and electrical overshoot.
 */
class VelocityController {
public:
    /**
     * @brief Constructor for the Velocity Controller.
     * @param dt Time step (seconds) of the control loop.
     * @param max_acceleration The maximum allowed rate of change in velocity (degrees/s^2).
     */
    VelocityController(float dt, float max_acceleration);

    /**
     * @brief Calculates the next safe, ramped velocity based on the desired target velocity.
     * @param target_velocity The user-commanded target velocity (degrees/s).
     * @return The next safe, ramped velocity (degrees/s) to command to the motor.
     */
    float update(float target_velocity);

    /**
     * @brief Get the current ramped velocity output.
     * @return Current ramped velocity in degrees/s.
     */
    float get_current_velocity() const { return current_velocity_; }

    /**
     * @brief Set a new maximum acceleration/deceleration limit.
     * @param new_max_accel The new acceleration limit (degrees/s^2).
     */
    void set_max_acceleration(float new_max_accel);

private:
    float dt_;                  // Control loop time step (seconds)
    float max_acceleration_;    // Maximum allowed acceleration (degrees/s^2)
    float current_velocity_;    // The output velocity after ramping is applied
};

#endif // VELOCITY_CONTROLLER_HPP
