#ifndef RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H
#define RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

class ExtendedKalmanFilter {
public:
    /**
     * @brief EKF Constructor for CTRV Model
     * State Vector (5D): [px, py, v, yaw, yaw_rate]
     * Measurement Vector (4D): [px, py, vx, vy] (Compensated velocity from NuScenes)
     */
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();

    /**
     * @brief Initialize the filter with the first measurement
     * @param meas Initial measurement [px, py, vx, vy]
     */
    void init(const Eigen::Vector4d& meas);

    /**
     * @brief Prediction Step (Non-linear CTRV model)
     * @param dt Time delta in seconds
     */
    void predict(double dt);

    /**
     * @brief Update Step (Non-linear Measurement model)
     * @param meas Current measurement [px, py, vx, vy]
     */
    void update(const Eigen::Vector4d& meas);

    // Get the full 5D state
    Eigen::VectorXd getState() const { return x_; }

    // Helper: Convert current internal state to Cartesian format for easy visualization/matching
    // Returns: [px, py, vx, vy]
    Eigen::Vector4d getCartesianState() const;

    // Get current covariance (useful for debugging or gating)
    Eigen::MatrixXd getCovariance() const { return P_; }

    // Check if initialized
    bool isInitialized() const { return is_initialized_; }

private:
    // --- Flags ---
    bool is_initialized_;

    // --- Matrices & Vectors ---
    Eigen::VectorXd x_; // State: [px, py, v, yaw, yaw_rate]
    Eigen::MatrixXd P_; // State Covariance (5x5)
    Eigen::MatrixXd Q_; // Process Noise Covariance (5x5)
    Eigen::MatrixXd R_; // Measurement Noise Covariance (4x4)
    
    // Jacobians
    Eigen::MatrixXd F_j_; // Jacobian of Motion Model (5x5)
    Eigen::MatrixXd H_j_; // Jacobian of Measurement Model (4x5)

    // --- Parameters ---
    // Process noise standard deviations (Tunable)
    double std_a_;      // Linear acceleration noise (m/s^2)
    double std_yawdd_;  // Angular acceleration noise (rad/s^2)

    // --- Helpers ---
    double normalizeAngle(double angle);
};

#endif // RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H