#include "kalman_filter/ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() : is_initialized_(false) {
    // 1. Initialize Dimensions
    x_ = Eigen::VectorXd(5);
    P_ = Eigen::MatrixXd(5, 5);
    Q_ = Eigen::MatrixXd(5, 5);
    R_ = Eigen::MatrixXd(4, 4);
    F_j_ = Eigen::MatrixXd(5, 5);
    H_j_ = Eigen::MatrixXd(4, 5);

    // 2. Initialize Noise Parameters (Tunable based on NuScenes radar characteristics)
    std_a_ = 2.0;       // Assumed max linear acceleration fluctuation
    std_yawdd_ = 1.0;   // Assumed max angular acceleration fluctuation

    // 3. Initialize Measurement Noise R
    // Radar is accurate in radial distance but noisy in lateral position.
    // Velocity measurement is generally okay but has noise.
    R_.setIdentity();
    R_(0, 0) = 1.0 * 1.0; // var_px
    R_(1, 1) = 0.5 * 0.5; // var_py
    R_(2, 2) = 2.0 * 2.0; // var_vx
    R_(3, 3) = 2.0 * 2.0; // var_vy
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

double ExtendedKalmanFilter::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void ExtendedKalmanFilter::init(const Eigen::Vector4d& meas) {
    // Measurement: [px, py, vx, vy]
    double px = meas(0);
    double py = meas(1);
    double vx = meas(2);
    double vy = meas(3);

    double v = sqrt(vx*vx + vy*vy);
    double yaw = 0.0;
    
    // Safety check: atan2 is unstable at very low speeds
    if (v > 0.1) {
        yaw = atan2(vy, vx);
    }

    // Initialize State: [px, py, v, yaw, yaw_rate]
    // yaw_rate is unknown initially, assume 0
    x_ << px, py, v, yaw, 0.0;

    // Initialize Covariance P
    // High uncertainty for unobserved states (yaw_rate)
    P_.setIdentity();
    P_(0,0) = 1.0 * 1.0; 
    P_(1,1) = 0.5 * 0.5;
    P_(2,2) = 2.0 * 2.0;
    P_(3,3) = 2.0 * 2.0;  
    P_(4,4) = 100; 

    is_initialized_ = true;
}

void ExtendedKalmanFilter::predict(double dt) {
    if (!is_initialized_) return;

    double v = x_(2);
    double yaw = x_(3);
    double yawd = x_(4);

    // --- 1. Predict State x_pred = f(x) ---
    // Check for division by zero (driving straight)
    if (fabs(yawd) > 0.001) {
        // Turning
        x_(0) += (v / yawd) * (sin(yaw + yawd * dt) - sin(yaw));
        x_(1) += (v / yawd) * (cos(yaw) - cos(yaw + yawd * dt));
        // v is constant in CTRV
        x_(3) += yawd * dt;
        // yawd is constant in CTRV
    } else {
        // Straight (Limit of sin(x)/x as x->0)
        x_(0) += v * dt * cos(yaw);
        x_(1) += v * dt * sin(yaw);
        x_(3) += 0; 
    }
    
    x_(3) = normalizeAngle(x_(3));

    // --- 2. Calculate Process Noise Q ---
    // Approximation using discrete time noise integration
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    // 预计算方差 (Variance)
    double var_a = std_a_ * std_a_;       // 线性加速度方差
    double var_yawdd = std_yawdd_ * std_yawdd_; // 角加速度方差

    double c = cos(yaw);
    double s = sin(yaw);

    // The Q matrix models the uncertainty in our "constant velocity/turn rate" assumption
    // We assume noise enters via linear acceleration (nu_a) and angular acceleration (nu_yawdd)
    Q_.setZero();

    // 对角线元素 (Variance)
    Q_(0, 0) = 0.25 * dt4 * var_a * c * c;   // px 的方差
    Q_(1, 1) = 0.25 * dt4 * var_a * s * s;   // py 的方差
    Q_(2, 2) = dt2 * var_a;                  // v 的方差

    // 互相关项 (Covariance - Linear)
    // px 与 py 的相关性 (因为它们共享同一个纵向加速度)
    Q_(0, 1) = 0.25 * dt4 * var_a * c * s;
    Q_(1, 0) = Q_(0, 1);

    // px 与 v 的相关性
    Q_(0, 2) = 0.5 * dt3 * var_a * c;
    Q_(2, 0) = Q_(0, 2);

    // py 与 v 的相关性
    Q_(1, 2) = 0.5 * dt3 * var_a * s;
    Q_(2, 1) = Q_(1, 2);


    // --- Part 2: Angular Acceleration Noise (影响 yaw, yaw_rate) ---
    // 这部分噪声与位置和速度是解耦的 (假设)

    // 对角线元素
    Q_(3, 3) = 0.25 * dt4 * var_yawdd; // yaw 的方差
    Q_(4, 4) = dt2 * var_yawdd;        // yaw_rate 的方差

    // 互相关项 (Covariance - Angular)
    // yaw 与 yaw_rate 的相关性
    Q_(3, 4) = 0.5 * dt3 * var_yawdd;
    Q_(4, 3) = Q_(3, 4);
    
    // --- 3. Calculate Jacobian F_j ---
    F_j_.setIdentity(); // Diagonal elements are 1.0

    if (fabs(yawd) > 0.001) {
        // Derivatives of the geometric position update formulas
        F_j_(0, 2) = (1/yawd) * (sin(yaw + yawd*dt) - sin(yaw)); // dpx/dv
        F_j_(0, 3) = (v/yawd) * (cos(yaw + yawd*dt) - cos(yaw)); // dpx/dyaw
        F_j_(0, 4) = (v*dt/yawd)*cos(yaw+yawd*dt) - (v/pow(yawd,2))*(sin(yaw+yawd*dt)-sin(yaw)); // dpx/dyawd

        F_j_(1, 2) = (1/yawd) * (cos(yaw) - cos(yaw + yawd*dt)); // dpy/dv
        F_j_(1, 3) = (v/yawd) * (sin(yaw + yawd*dt) - sin(yaw)); // dpy/dyaw
        F_j_(1, 4) = (v*dt/yawd)*sin(yaw+yawd*dt) - (v/pow(yawd,2))*(cos(yaw)-cos(yaw+yawd*dt)); // dpy/dyawd
        
        F_j_(3, 4) = dt; // dyaw/dyawd
    } else {
        // Simple linear approximation for straight line
        F_j_(0, 2) = dt * cos(yaw);
        F_j_(0, 3) = -v * dt * sin(yaw);
        F_j_(1, 2) = dt * sin(yaw);
        F_j_(1, 3) = v * dt * cos(yaw);
        F_j_(3, 4) = dt;
    }

    // --- 4. Predict Covariance P ---
    P_ = F_j_ * P_ * F_j_.transpose() + Q_;
}

void ExtendedKalmanFilter::update(const Eigen::Vector4d& meas) {
    if (!is_initialized_) return;

    // 1. Measurement Prediction z_pred = h(x)
    // Map State [px, py, v, yaw, yawd] -> Measurement [px, py, vx, vy]
    double v = x_(2);
    double yaw = x_(3);

    Eigen::Vector4d z_pred;
    z_pred(0) = x_(0);
    z_pred(1) = x_(1);
    z_pred(2) = v * cos(yaw);
    z_pred(3) = v * sin(yaw);

    // 2. Innovation y = z - z_pred
    Eigen::Vector4d y = meas - z_pred;

    // 3. Jacobian H_j (Linearize measurement function)
    H_j_.setZero();
    
    // Row 0, 1: Position is linear
    H_j_(0, 0) = 1.0;
    H_j_(1, 1) = 1.0;

    // Row 2: vx = v * cos(yaw)
    H_j_(2, 2) = cos(yaw);      // dvx/dv
    H_j_(2, 3) = -v * sin(yaw); // dvx/dyaw
    
    // Row 3: vy = v * sin(yaw)
    H_j_(3, 2) = sin(yaw);      // dvy/dv
    H_j_(3, 3) = v * cos(yaw);  // dvy/dyaw

    // 4. Standard Kalman Update
    Eigen::MatrixXd S = H_j_ * P_ * H_j_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_j_.transpose() * S.inverse();

    // Update State
    x_ = x_ + (K * y);
    x_(3) = normalizeAngle(x_(3)); // Normalize yaw after update

    // Update Covariance
    long x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_j_) * P_;
}

Eigen::Vector4d ExtendedKalmanFilter::getCartesianState() const {
    // Helper to visualize the output easily
    double v = x_(2);
    double yaw = x_(3);
    
    Eigen::Vector4d cart_state;
    cart_state(0) = x_(0); // px
    cart_state(1) = x_(1); // py
    cart_state(2) = v * cos(yaw); // vx
    cart_state(3) = v * sin(yaw); // vy
    return cart_state;
}