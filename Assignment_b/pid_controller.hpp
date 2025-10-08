#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

/**

 * @brief Defines the PIDController class for proportional-integral-derivative control.
 * This controller is used to regulate the motor's velocity by calculating the 
 * required motor torque/current command based on the velocity error.
 */
class PIDController {
public:
    /**
     * @brief Constructor for the PID Controller.
     * @param dt Time step (seconds) of the control loop.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param out_limit Maximum absolute value for the final output command (e.g., max motor current).
     * @param integral_limit Maximum absolute value for the integral term (for anti-windup).
     */
    PIDController(float dt, float kp, float ki, float kd, float out_limit, float integral_limit);

    /**
     * @brief Executes one iteration of the PID control loop.
     * @param setpoint The desired value (e.g., target velocity from the Velocity Controller).
     * @param measured_value The actual observed value (e.g., estimated velocity from the Kalman Filter).
     * @return The calculated control output command (e.g., motor current/voltage).
     */
    float update(float setpoint, float measured_value);

    /**
     * @brief Reset the integral term and previous error, useful when switching modes.
     */
    void reset();

    // Setter functions for runtime tuning
    void set_gains(float kp, float ki, float kd);
    void set_output_limit(float out_limit);
    void set_integral_limit(float integral_limit);

private:
    float dt_;                  // Time step
    
    // Gains
    float Kp_;
    float Ki_;
    float Kd_;

    // Limits
    float output_limit_;        // Max absolute output value
    float integral_limit_;      // Max absolute integral sum (anti-windup)

    // State Variables
    float integral_sum_;        // Accumulator for the integral term
    float prev_error_;          // Error from the previous time step (for derivative)
    float prev_measured_value_; // Used for derivative on measurement (if Kd is on measurement)
};

#endif // PID_CONTROLLER_HPP
