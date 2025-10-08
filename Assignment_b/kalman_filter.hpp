#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <vector>

/**
 * @file kalman_filter.hpp
 * @brief Defines the KalmanFilter class for state estimation (Position and Velocity).
 */
class KalmanFilter {
public:
    /**
     * @brief Constructor.
     * @param dt Time step (seconds).
     * @param q_pos Process noise for position.
     * @param q_vel Process noise for velocity.
     * @param r_pos Measurement noise for position sensor.
     */
    KalmanFilter(float dt, float q_pos, float q_vel, float r_pos);

    /**
     * @brief Updates the filter with a new position measurement.
     * @param measured_position The noisy sensor reading (degrees).
     * @param true_position The true position (only for logging/error checking in simulation).
     */
    void update(float measured_position, float true_position);

    /**
     * @brief Get the current estimated velocity.
     * @return Estimated angular velocity (degrees/s).
     */
    float get_estimated_velocity() const { return X_[1]; }

private:
    float dt_;

    // State Vector X_ = [position, velocity]'
    std::vector<float> X_ = {0.0f, 0.0f};

    // Covariance Matrix P_
    std::vector<std::vector<float>> P_ = {{0.0f, 0.0f}, {0.0f, 0.0f}};

    // Process Noise Covariance Matrix Q_
    std::vector<std::vector<float>> Q_;

    // Measurement Noise Covariance R_
    float R_pos_;
};

#endif // KALMAN_FILTER_HPP
