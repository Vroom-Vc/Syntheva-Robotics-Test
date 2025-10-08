#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

/**
 * @file pid_controller.hpp
 * @brief Defines the PIDController class for velocity control.
 */
class PIDController {
public:
    /**
     * @brief Constructor for the PID Controller.
     * @param dt Time step (seconds).
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param max_output Maximum output limit (now normalized to 1.0).
     * @param integral_limit Anti-windup limit for the integral term.
     */
    PIDController(float dt, float kp, float ki, float kd, float max_output, float integral_limit);

    /**
     * @brief Calculates the normalized control signal based on velocity error.
     * @param setpoint The desired velocity (ramped_velocity).
     * @param measurement The estimated actual velocity (estimated_velocity).
     * @return Normalized control signal between -max_output and +max_output (-1.0 to 1.0).
     */
    float update(float setpoint, float measurement);

private:
    float dt_;
    float kp_, ki_, kd_;
    float max_output_;
    float integral_limit_;

    float integral_sum_ = 0.0f;
    float last_error_ = 0.0f;
    
    // Note: Derivative is applied on the measurement (velocity), not the error, to reduce derivative kick.
    float last_measurement_ = 0.0f; 
};

#endif // PID_CONTROLLER_HPP
