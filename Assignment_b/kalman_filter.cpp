#include "kalman_filter.hpp"
#include <cmath> 

/**
 * @file kalman_filter.cpp
 * @brief Implementation of the KalmanFilter class using specialized 2x2 matrix math 
 * for performance in an embedded environment.
 */

// --- Helper Matrix Operations (Specialized for 2x2 Matrices) ---

// Helper function for matrix multiplication (2x2 * 2x2)
void KalmanFilter::multiply_2x2(const float a[2][2], const float b[2][2], float result[2][2]) {
}

// Helper function for matrix transposition (2x2)
void KalmanFilter::transpose_2x2(const float a[2][2], float result[2][2]) {
    
}

// Helper function for matrix addition (2x2)
void KalmanFilter::add_2x2(const float a[2][2], const float b[2][2], float result[2][2]) {
    
}

// Helper function for matrix subtraction (2x2)
void KalmanFilter::subtract_2x2(const float a[2][2], const float b[2][2], float result[2][2]) {
    
}

// Helper function for matrix inversion (2x2)

bool KalmanFilter::inverse_2x2(const float a[2][2], float result[2][2]) {
    
}

/**
 * @brief Initializes the Kalman Filter with time step and noise characteristics.
 * @param dt The time step (seconds) of the control loop.
 * @param process_noise_q Scalar representing uncertainty in the model (Q).
 * @param measurement_noise_r Scalar representing sensor noise (R).
 */
KalmanFilter::KalmanFilter(float dt, float process_noise_q, float measurement_noise_r) 
    : dt_(dt), R_(measurement_noise_r) {
    
    // 1. Initialize State Vector X = [Position, Velocity]
    state_x_[0] = 0.0f; // Initial position (degrees)
    state_x_[1] = 0.0f; // Initial velocity (degrees/s)

    // 2. Initialize State Transition Matrix A (Constant Velocity Model)
    // A = [[1, dt], [0, 1]]
    A_[0][0] = 1.0f; A_[0][1] = dt_;
    A_[1][0] = 0.0f; A_[1][1] = 1.0f;

    // 3. Initialize Error Covariance Matrix P (Start with high uncertainty)
    // P = [[100, 0], [0, 100]]
    P_[0][0] = 100.0f; P_[0][1] = 0.0f;
    P_[1][0] = 0.0f; P_[1][1] = 100.0f;

    // 4. Initialize Process Noise Covariance Matrix Q (Modeling uncertainty in acceleration)
    // Q is derived from process_noise_q, assuming noise affects acceleration (a)
    // Q = [[(dt^4)/4, (dt^3)/2], [(dt^3)/2, dt^2]] * process_noise_q
    float dt2 = dt_ * dt_;
    float dt3 = dt2 * dt_;
    float dt4 = dt3 * dt_;

    Q_[0][0] = (dt4 / 4.0f) * process_noise_q;
    Q_[0][1] = (dt3 / 2.0f) * process_noise_q;
    Q_[1][0] = (dt3 / 2.0f) * process_noise_q;
    Q_[1][1] = dt2 * process_noise_q;
}


/**
 * @brief Executes the Kalman Filter prediction and update steps.
 * @param raw_measurement The latest raw position reading from the sensor.
 * @return The filtered position estimate.
 */
float KalmanFilter::update(float raw_measurement) {
    // --- PREDICTION STEP (Project state and covariance ahead) ---
    
    // 1. Predict State: X_k = A * X_{k-1} 
    // Optimization: Matrix-Vector multiplication with A = [[1, dt], [0, 1]]
    float predicted_pos = state_x_[0] + dt_ * state_x_[1];
    float predicted_vel = state_x_[1]; // Velocity remains constant in prediction
    
    state_x_[0] = predicted_pos;
    state_x_[1] = predicted_vel;

    // 2. Predict Covariance: P_k = A * P_{k-1} * A_T + Q
    
    // Temporary matrices for calculation
    float P_temp1[2][2]; // A * P_{k-1}
    float P_temp2[2][2]; // P_{k-1} * A_T
    float A_T[2][2];     // A transpose

    // Calculate A_T
    transpose_2x2(A_, A_T);

    // Calculate A * P_{k-1}
    multiply_2x2(A_, P_, P_temp1); 

    // Calculate (A * P_{k-1}) * A_T 
    multiply_2x2(P_temp1, A_T, P_temp2); 

    // P_k = P_temp2 + Q
    add_2x2(P_temp2, Q_, P_);


    // --- UPDATE (CORRECTION) STEP (Incorporate measurement) ---

    // 1. Calculate Innovation Covariance (S) 
    // S = H * P_k * H_T + R. Since H = [1, 0], H * P * H_T simplifies to P[0][0].
    // S is a scalar.
    float S = P_[0][0] + R_;

    // Handle singularity (shouldn't happen with R > 0, but good practice)
    if (std::abs(S) < 1e-6f) {
        return state_x_[0]; // Return predicted state if gain calculation is unstable
    }
    
    // S inverse
    float S_inv = 1.0f / S;

    // 2. Calculate Kalman Gain (K) 
    // K = P_k * H_T * S_inv. P * H_T is the first column of P. K is 2x1.
    float K[2];
    K[0] = P_[0][0] * S_inv;
    K[1] = P_[1][0] * S_inv;
    
    // 3. Calculate Innovation (y) 
    // y = Z - H * X_k. Since H = [1, 0], H * X_k = X_k[0]. y is a scalar.
    float innovation_y = raw_measurement - state_x_[0];

    // 4. Update State Estimate: X_k = X_k + K * y
    state_x_[0] = state_x_[0] + K[0] * innovation_y;
    state_x_[1] = state_x_[1] + K[1] * innovation_y;

    // 5. Update Error Covariance: P_k = (I - K * H) * P_k
    // Calculate I - K * H (KH is 2x2 matrix)
    float I_minus_KH[2][2];
    I_minus_KH[0][0] = 1.0f - K[0] * H_[0]; // 1 - K[0]
    I_minus_KH[0][1] = 0.0f - K[0] * H_[1]; // 0
    I_minus_KH[1][0] = 0.0f - K[1] * H_[0]; // -K[1]
    I_minus_KH[1][1] = 1.0f - K[1] * H_[1]; // 1

    // P_k = (I - K * H) * P_k
    float P_new[2][2];
    multiply_2x2(I_minus_KH, P_, P_new);
    P_[0][0] = P_new[0][0]; P_[0][1] = P_new[0][1];
    P_[1][0] = P_new[1][0]; P_[1][1] = P_new[1][1];

    // Return the new filtered position
    return state_x_[0];
}
