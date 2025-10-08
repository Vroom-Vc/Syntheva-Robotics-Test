#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

/**

 * @brief Defines the KalmanFilter class for estimating motor state (position and velocity).
 * * The Kalman Filter is essential for filtering out sensor noise to provide a smooth, 
 * low-latency, and accurate state estimate for the PID controller.
 * * State Vector X (2x1): [Position (degrees), Velocity (degrees/s)]
 * Measurement Z (1x1): [Position (degrees)]
 */
class KalmanFilter {
public:
    /**
     * @brief Constructor for the Kalman Filter.
     * @param dt Time step (seconds) of the control loop (e.g., 0.01 for 100Hz).
     * @param process_noise_q Process noise covariance (Q) scalar (e.g., 0.1 for high noise).
     * @param measurement_noise_r Measurement noise covariance (R) scalar (e.g., 1.0 for low noise).
     */
    KalmanFilter(float dt, float process_noise_q, float measurement_noise_r);

    /**
     * @brief Executes the Kalman Filter prediction and update steps.
     * @param raw_measurement The latest raw position reading from the sensor.
     * @return The filtered position estimate.
     */
    float update(float raw_measurement);

    /**
     * @brief Get the current filtered position estimate.
     * @return Filtered position in degrees.
     */
    float get_position() const { return state_x_[0]; }

    /**
     * @brief Get the current filtered velocity estimate.
     * @return Filtered velocity in degrees/s.
     */
    float get_velocity() const { return state_x_[1]; }

private:
    // Time step between updates (Delta T)
    float dt_; 

    // State Vector X (Position, Velocity)
    float state_x_[2]; 

    // State Covariance Matrix P (2x2)
    float P_[2][2]; 

    // Process Noise Covariance Q (2x2) 
    // Q is the covariance of the process, related to unmodeled acceleration
    float Q_[2][2]; 

    // Measurement Noise Covariance R (1x1) - Sensor noise
    float R_; 

    // Process Model Matrix A (2x2) - Constant velocity model
    // A = [[1, dt], [0, 1]]
    float A_[2][2]; 

    // Measurement Matrix H (1x2) - We only measure position
    // H = [[1, 0]]
    const float H_[2] = {1.0f, 0.0f};

    // Helper function for matrix multiplication (2x2 * 2x2)
    void multiply_2x2(const float a[2][2], const float b[2][2], float result[2][2]);

    // Helper function for matrix transposition (2x2)
    void transpose_2x2(const float a[2][2], float result[2][2]);

    // Helper function for matrix addition (2x2)
    void add_2x2(const float a[2][2], const float b[2][2], float result[2][2]);
    
    // Helper function for matrix subtraction (2x2)
    void subtract_2x2(const float a[2][2], const float b[2][2], float result[2][2]);

    // Helper function for matrix inversion (2x2)
    bool inverse_2x2(const float a[2][2], float result[2][2]);
};

#endif // KALMAN_FILTER_HPP
