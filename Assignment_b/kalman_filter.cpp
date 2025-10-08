#include "kalman_filter.hpp"
#include <iostream>

/**
 * @file kalman_filter.cpp
 * @brief Implementation of a simple 2D (Position/Velocity) Kalman Filter.
 */

KalmanFilter::KalmanFilter(float dt, float q_pos, float q_vel, float r_pos) 
    : dt_(dt), R_pos_(r_pos) {
    // Initialize Covariance P (Start with high uncertainty)
    P_[0][0] = 1000.0f; P_[0][1] = 0.0f;
    P_[1][0] = 0.0f; P_[1][1] = 1000.0f;

    // Initialize Process Noise Q
    Q_ = {{q_pos, 0.0f}, {0.0f, q_vel}};
}

/**
 * @brief Updates the filter with a new position measurement.
 */
void KalmanFilter::update(float measured_position, float true_position) {
    // --- 1. Prediction Step ---
    // State transition matrix F (constant velocity model)
    // F = [[1, dt], [0, 1]]
    
    // Predict new state X_ = F * X_
    // X_[0] (Position) = X_[0] + X_[1] * dt
    // X_[1] (Velocity) = X_[1] (constant)
    X_[0] += X_[1] * dt_;

    // Predict covariance P_ = F * P * F' + Q
    float p00_temp = P_[0][0];
    float p01_temp = P_[0][1];
    float p10_temp = P_[1][0];
    float p11_temp = P_[1][1];

    P_[0][0] = p00_temp + dt_ * (p10_temp + p01_temp) + dt_ * dt_ * p11_temp + Q_[0][0];
    P_[0][1] = p01_temp + dt_ * p11_temp;
    P_[1][0] = p10_temp + dt_ * p11_temp;
    P_[1][1] = p11_temp + Q_[1][1];
    
    // --- 2. Update Step ---
    // Measurement matrix H = [1, 0] (We only measure position)
    
    // Calculate Kalman Gain K = P * H' * (H * P * H' + R)^-1
    // The measurement residual covariance S = H * P * H' + R = P[0][0] + R_pos
    float S = P_[0][0] + R_pos_;

    if (S < 1e-6) {
        // Avoid division by zero, treat as no update
        return; 
    }

    // Kalman Gain K = [ K0; K1 ]
    float K0 = P_[0][0] / S;
    float K1 = P_[1][0] / S;

    // Update state X = X + K * y
    // y (Innovation/Residual) = measured_position - H * X = measured_position - X[0]
    float innovation = measured_position - X_[0];
    X_[0] += K0 * innovation;
    X_[1] += K1 * innovation;

    // Update covariance P = (I - K * H) * P
    float p00_new = (1.0f - K0) * P_[0][0];
    float p01_new = (1.0f - K0) * P_[0][1];
    float p10_new = -K1 * P_[0][0] + P_[1][0];
    float p11_new = -K1 * P_[0][1] + P_[1][1];

    P_[0][0] = p00_new;
    P_[0][1] = p01_new;
    P_[1][0] = p10_new;
    P_[1][1] = p11_new;
}
