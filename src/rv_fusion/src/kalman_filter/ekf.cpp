#include "kalman_filter/ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(const EKFConfig& config) 
    : config_(config), is_initialized_(false), nis_(-1.0), 
      last_mahalanobis_dist_(0.0), last_update_valid_(false) {
    
    // 参数验证
    validateConfiguration();
    
    int n_x = config_.state_dim;
    int n_z = config_.meas_dim;
    // 动态分配矩阵维度
    x_ = Eigen::VectorXd::Zero(n_x);
    P_ = Eigen::MatrixXd::Zero(n_x, n_x);
    Q_ = Eigen::MatrixXd::Zero(n_x, n_x);
    R_ = Eigen::MatrixXd::Zero(n_z, n_z);
    // 预分配雅可比矩阵内存
    Fj_ = Eigen::MatrixXd::Identity(n_x, n_x);
    Hj_ = Eigen::MatrixXd::Zero(n_z, n_x);
    // 初始化测量噪声矩阵
    initializeMeasurementNoiseMatrix();
    std::cout << "EKF initialized with state_dim=" << n_x 
              << ", meas_dim=" << n_z << std::endl;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::validateConfiguration() const {
    // 检查维度兼容性
    if (config_.state_dim != 5) {
        throw std::invalid_argument("State dimension must be 5 for CTRV model");
    }
    if (config_.meas_dim != 4) {
        throw std::invalid_argument("Measurement dimension must be 4 for [px, py, vx, vy]");
    }
    // 检查噪声参数合理性
    if (config_.process_noise.std_a <= 0) {
        throw std::invalid_argument("Linear acceleration noise must be positive");
    }
    if (config_.process_noise.std_yawdd <= 0) {
        throw std::invalid_argument("Angular acceleration noise must be positive");
    }
    if (config_.meas_noise.std_px <= 0 || config_.meas_noise.std_py <= 0 ||
        config_.meas_noise.std_vx <= 0 || config_.meas_noise.std_vy <= 0) {
        throw std::invalid_argument("Measurement noise standard deviations must be positive");
    }
    // 输出配置警告
    if (config_.process_noise.std_a < 0.5 || config_.process_noise.std_a > 5.0) {
        std::cerr << "Warning: Unusual linear acceleration noise: " 
                  << config_.process_noise.std_a << " m/s²" << std::endl;
    }
}

void ExtendedKalmanFilter::initializeMeasurementNoiseMatrix() {
    R_.setZero();
    R_(0, 0) = config_.meas_noise.std_px * config_.meas_noise.std_px;
    R_(1, 1) = config_.meas_noise.std_py * config_.meas_noise.std_py;
    R_(2, 2) = config_.meas_noise.std_vx * config_.meas_noise.std_vx;
    R_(3, 3) = config_.meas_noise.std_vy * config_.meas_noise.std_vy;
}

// 健壮的角度归一化实现
double ExtendedKalmanFilter::normalizeAngle(double angle) const {
    constexpr double two_pi = 2.0 * M_PI;
    constexpr double pi = M_PI;
    // 处理极端大值
    if (std::fabs(angle) > 10 * two_pi) {
        angle = std::fmod(angle, two_pi);
    }
    // 标准归一化
    angle = std::fmod(angle + pi, two_pi);
    if (angle < 0) {
        angle += two_pi;
    }
    angle -= pi;
    // 边界检查
    if (angle <= -pi || angle > pi) {
        return 0.0; // 归一化失败，使用安全值
    }
    return angle;
}

void ExtendedKalmanFilter::init(const Eigen::Vector4d& meas) {
    // 输入验证
    if (meas.hasNaN()) {
        throw std::invalid_argument("Initial measurement contains NaN values");
    }
    
    double px = meas(0);
    double py = meas(1);
    double vx = meas(2);
    double vy = meas(3);

    // 计算初始速度大小
    double v = std::sqrt(vx*vx + vy*vy);

    // 方向角初始化：低速时使用保守估计
    double yaw = 0.0;
    bool low_speed_init = false;
    if (v > 1.5) {
        // 速度足够大，可以可靠计算方向角
        yaw = std::atan2(vy, vx);
    } else {
        // 低速情况：使用0度方向，但增大不确定性
        yaw = 0.0;
        low_speed_init = true;
    }

    // 初始化状态向量
    x_ << px, py, v, yaw, 0.0; // yaw_rate初始为0

    // 系统性初始化协方差矩阵
    initializeCovarianceMatrix(v, yaw);
    is_initialized_ = true;
    nis_ = -1.0; // 重置NIS

    // 记录初始化信息
    std::cout << "EKF Initialized: v=" << v << " m/s, yaw=" << yaw * 180/M_PI 
              << " deg, low_speed=" << low_speed_init << std::endl;
}

void ExtendedKalmanFilter::initializeCovarianceMatrix(double initial_speed, double initial_yaw) {
    P_.setZero();
    
    // 位置不确定性：基于测量噪声，但考虑初始测量的可靠性
    double pos_quality_factor = calculatePositionQualityFactor(initial_speed);
    P_(0, 0) = pos_quality_factor * config_.meas_noise.std_px * config_.meas_noise.std_px;
    P_(1, 1) = pos_quality_factor * config_.meas_noise.std_py * config_.meas_noise.std_py;
    
    // 速度不确定性：基于速度大小的自适应
    double speed_uncertainty = calculateAdaptiveSpeedUncertainty(initial_speed);
    P_(2, 2) = speed_uncertainty * speed_uncertainty;
    
    // 方向角不确定性：平滑的自适应
    double yaw_uncertainty = calculateAdaptiveYawUncertainty(initial_speed);
    P_(3, 3) = yaw_uncertainty * yaw_uncertainty;
    
    // 转向率不确定性
    P_(4, 4) = config_.init_params.initial_std_yawrate * config_.init_params.initial_std_yawrate;
    
    // 2. 非对角线元素（协方差/相关性）
    initializeCrossCorrelations(initial_speed, initial_yaw, speed_uncertainty, yaw_uncertainty);
    
    // 3. 数值稳定性处理
    ensureCovarianceProperties();
}

// 辅助函数：计算位置质量因子
double ExtendedKalmanFilter::calculatePositionQualityFactor(double speed) const {
    // 低中速位置测量都比较准
    if (speed < 12) {
        return 1.0; 
    }
    else {
        // 高速时，点云拖尾导致中心点漂移，线性增加不确定性
        double ratio = (speed - 20) / 10.0; 
        return std::max(1.0, 1.0 + ratio);
    }
}

// 辅助函数：计算自适应速度不确定性（速度模长）
double ExtendedKalmanFilter::calculateAdaptiveSpeedUncertainty(double speed) const {
    double base_uncertainty = config_.init_params.initial_std_v;
    if (speed < 15) {
        // 低速情况：速度模长测量准确，减小不确定性
        double low_speed_factor = 0.3;  // 低速时不确定性减小到30%
        return base_uncertainty * low_speed_factor;
    }
    else if (speed > 25) {
        // 高速情况：存在拖尾效应，增大不确定性
        double high_speed_factor = 1.0 + (speed - 25.0) / 10.0;  // 线性增长
        high_speed_factor = std::min(high_speed_factor, 3.0);  // 最大3倍
        return base_uncertainty * high_speed_factor;
    }
    else {
        // 正常速度范围：使用基础不确定性
        return base_uncertainty;
    }
}

// 辅助函数：计算自适应方向角不确定性
double ExtendedKalmanFilter::calculateAdaptiveYawUncertainty(double speed) const {
    double base_uncertainty = config_.init_params.initial_std_yaw;
    if (speed < 1.0) {
        // 低速情况：方向完全不可靠
        // 当速度很小时，从速度矢量计算方向角毫无意义
        return M_PI;  // 180度，表示方向完全不确定
    }
    if (speed < 5.0) {
        double ratio = (speed - 1.0) / 4.0; 
        // 线性从 PI 降到 std_yaw (比如 0.1)
        return M_PI - ratio * (M_PI - config_.init_params.initial_std_yaw);
    }
    // 正常行驶
    else {
        // 正常速度范围：使用基础不确定性
        return base_uncertainty;
    }
}

// 辅助函数：初始化状态间相关性
void ExtendedKalmanFilter::initializeCrossCorrelations(double speed, double yaw, 
                                                       double speed_std, double yaw_std) {
    if (speed > config_.thresholds.epsilon_yaw_rate) { // 使用配置的epsilon阈值
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        double corr = config_.init_params.position_correlation;
        
        // 位置-速度相关性（运动方向）
        P_(0, 2) = corr * std::sqrt(P_(0, 0)) * speed_std * cos_yaw;
        P_(2, 0) = P_(0, 2);
        P_(1, 2) = corr * std::sqrt(P_(1, 1)) * speed_std * sin_yaw;
        P_(2, 1) = P_(1, 2);
        
        // 速度-方向角相关性（高速时方向更确定）
        if (speed > 2.0) { // 速度阈值可配置
            double speed_yaw_corr = 0.2; // 可配置
            P_(2, 3) = speed_yaw_corr * speed_std * yaw_std;
            P_(3, 2) = P_(2, 3);
        }
        
        // 方向角-转向率相关性
        double yaw_yawrate_corr = 0.1; // 可配置
        P_(3, 4) = yaw_yawrate_corr * yaw_std * config_.init_params.initial_std_yawrate;
        P_(4, 3) = P_(3, 4);
    }
}

// 辅助函数：确保协方差矩阵性质
void ExtendedKalmanFilter::ensureCovarianceProperties() {
    // 1. 确保对称
    P_ = 0.5 * (P_ + P_.transpose());
    
    // 2. 特征值检查，确保正定性
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(P_);
    if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition failed during covariance initialization");
    }
    
    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    double min_eigenvalue = eigenvalues.minCoeff();
    
    // 3. 如果需要，添加正则化
    if (min_eigenvalue < 1e-8) {
        double regularization = std::max(1e-8, -min_eigenvalue + 1e-8);
        P_ += Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) * regularization;
        
        std::cout << "Covariance matrix regularized with value: " << regularization << std::endl;
    }
    
    // 4. 最终对称性检查
    P_ = 0.5 * (P_ + P_.transpose());
}

Eigen::VectorXd ExtendedKalmanFilter::calculateCTRVModel(const Eigen::VectorXd& x, double dt) {
    double px = x(0), py = x(1), v = x(2), yaw = x(3), yawd = x(4);
    Eigen::VectorXd x_next = x; // 初始化为当前状态

    // 预计算三角函数
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // 阈值判断：避免除零错误
    if (std::fabs(yawd) > 0.001) {
        // --- 转弯模型 (Turning) ---
        double v_yawd = v / yawd;
        double yaw_new = yaw + yawd * dt;
        double cy_new = std::cos(yaw_new);
        double sy_new = std::sin(yaw_new);

        x_next(0) = px + v_yawd * (sy_new - sy);
        x_next(1) = py + v_yawd * (cy - cy_new);
        x_next(3) = yaw_new;
    } else {
        // --- 直线模型 (Straight) ---
        // 使用二阶泰勒展开提高精度
        double yawd_dt = yawd * dt;
        double term1 = v * dt;
        double term2 = 0.5 * yawd_dt * term1; // 二阶修正项
        
        x_next(0) = px + term1 * cy - term2 * sy;
        x_next(1) = py + term1 * sy + term2 * cy;
        x_next(3) = yaw + yawd_dt;
    }
    
    // 速度和转向率在预测阶段保持不变
    // x_next(2) = v; 
    // x_next(4) = yawd;
    
    // 角度归一化 (-PI ~ PI)
    while (x_next(3) > M_PI) x_next(3) -= 2.0 * M_PI;
    while (x_next(3) < -M_PI) x_next(3) += 2.0 * M_PI;

    return x_next;
}

void ExtendedKalmanFilter::predict(double dt) {
    if (dt <= 0) return;

    // --- 0. 准备数据快照 ---
    // 保存当前状态用于计算雅可比，避免使用预测后的状态计算梯度
    Eigen::VectorXd x_curr = x_;
    double v = x_curr(2);
    double yaw = x_curr(3);
    double yawd = x_curr(4);

    // --- 1. 状态预测 (State Prediction) ---
    // 直接调用物理模型得到先验状态 x_{k|k-1}
    x_ = calculateCTRVModel(x_curr, dt);


    // --- 2. 计算雅可比矩阵 F (Jacobian Calculation) ---
    // F = df/dx
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
    
    // 2.1 解析部分 (Analytical)
    // 根据CTRV公式推导的偏导数
    if (std::fabs(yawd) > 0.001) {
        double yaw_new = yaw + yawd * dt;
        double inv_yawd = 1.0 / yawd;
        double sy = std::sin(yaw);
        double cy = std::cos(yaw);
        double sy_new = std::sin(yaw_new);
        double cy_new = std::cos(yaw_new);

        // d(px)/dv, d(px)/dyaw
        F(0, 2) = inv_yawd * (sy_new - sy);
        F(0, 3) = (v * inv_yawd) * (cy_new - cy);
        
        // d(py)/dv, d(py)/dyaw
        F(1, 2) = inv_yawd * (cy - cy_new);
        F(1, 3) = (v * inv_yawd) * (sy_new - sy);
        
        F(3, 4) = dt; // d(yaw)/d(yawd)
    } else {
        // 直线运动的简单雅可比
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);
        F(0, 2) = dt * cy;
        F(0, 3) = -v * dt * sy;
        F(1, 2) = dt * sy;
        F(1, 3) = v * dt * cy;
        F(3, 4) = dt;
    }

    // 2.2 数值部分 (Numerical Differentiation)
    // 计算 d(pos)/d(yawd) 这一项非常复杂，使用数值微分最稳健
    // 只需微扰 yawd 重新计算一次物理模型
    {
        const double h = 1e-5; // 扰动步长
        Eigen::VectorXd x_perturbed = x_curr;
        x_perturbed(4) += h;
        Eigen::VectorXd x_pred_h = calculateCTRVModel(x_perturbed, dt);
        
        // 差分计算第5列 (对应 yawd 的偏导)
        F(0, 4) = (x_pred_h(0) - x_(0)) / h;
        F(1, 4) = (x_pred_h(1) - x_(1)) / h;
    }


    // --- 3. 自适应过程噪声 Q (Adaptive Process Noise) ---
    // 将原本分散在 params, helper 里的逻辑整合在此
    double std_a = config_.process_noise.std_a;         // 基础加速度噪声
    double std_yawdd = config_.process_noise.std_yawdd; // 基础角加速度噪声
    
    // 3.1 自适应因子计算
    double speed_factor = 1.0;
    if (v > 15.0) speed_factor = 0.8; // 高速时模型更稳
    
    double turn_factor = 1.0;
    if (std::fabs(yawd) > 0.5) turn_factor = 1.5; // 急转弯时增加不确定性
    
    double dt_factor = std::pow(dt, 2); // 时间越长，累积误差越大

    // 应用因子
    double var_a = std::pow(std_a * speed_factor * turn_factor, 2);
    double var_yawdd = std::pow(std_yawdd * turn_factor, 2);

    // 3.2 构建 Q 矩阵 (使用驱动矩阵法 G*Qv*G^T)
    // 这种方法比直接填矩阵更符合物理意义
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(5, 2);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double half_dt2 = 0.5 * dt * dt;

    // 噪声驱动矩阵 G
    G(0, 0) = half_dt2 * cy; // ax -> px
    G(1, 0) = half_dt2 * sy; // ax -> py
    G(2, 0) = dt;            // ax -> v
    G(3, 1) = half_dt2;      // yawdd -> yaw
    G(4, 1) = dt;            // yawdd -> yawd

    Eigen::Matrix2d Qv;
    Qv << var_a, 0, 
          0, var_yawdd;
    
    Eigen::MatrixXd Q = G * Qv * G.transpose();

    // 3.3 确保 Q 矩阵数值稳定性 (简化的正则化)
    for(int i=0; i<5; ++i) Q(i,i) = std::max(Q(i,i), 1e-9); // 保证对角线非负


    // --- 4. 协方差预测 (Covariance Prediction) ---
    // P_k = F * P_{k-1} * F^T + Q
    P_ = F * P_ * F.transpose() + Q;

    // 4.1 强制对称性 (Symmetry)
    P_ = 0.5 * (P_ + P_.transpose());

    // 4.2 简单的正定性保护 (Positive Definiteness)
    // 不再每次都做特征值分解，那个太慢。只做对角线检查，只有极端情况才报错或重置
    bool robust = true;
    for(int i=0; i<5; ++i) {
        if (P_(i,i) < 0) {
            robust = false;
            break;
        }
    }
    
    if (!robust) {
        // 只有在协方差崩溃时才进行紧急重置
        std::cerr << "Covariance collapse! Resetting..." << std::endl;
        P_ = Eigen::MatrixXd::Identity(5,5) * 1.0; 
    }
}

void ExtendedKalmanFilter::update(const Eigen::Vector4d& meas) {
    if (!is_initialized_) {
        std::cerr << "EKF not initialized, skipping update" << std::endl;
        return;
    }
    
    // 1. 位置更新
    updatePosition(meas.head(2));

    // 2. 速度更新
    updateVelocity(meas.tail(2));
}

void ExtendedKalmanFilter::updatePosition(const Eigen::Vector2d& pos_meas) {
    // 线性测量矩阵 H = [1 0 0 0 0; 0 1 0 0 0]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d z_pred = x_.head(2);
    Eigen::VectorXd innovation = pos_meas - z_pred;
    
    // 取位置部分的噪声
    Eigen::MatrixXd R_pos = R_.block(0, 0, 2, 2);

    // 创新协方差 S = HPH' + R
    // 针对 H 的特殊结构，可以直接优化计算，这里为了通用性写全
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_pos;

    if (validateMeasurement(innovation, S, config_.thresholds.pos_gating_threshold)) {
        performUpdate(pos_meas, H, R_pos, innovation);
    }
}

void ExtendedKalmanFilter::updateVelocity(const Eigen::Vector2d& vel_meas) {
    double v = x_(2);
    double yaw = x_(3);

    // 非线性测量预测
    Eigen::Vector2d z_pred;
    z_pred << v * std::cos(yaw), v * std::sin(yaw);

    // 雅可比计算
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 2) = std::cos(yaw);        // dvx/dv
    H(0, 3) = -v * std::sin(yaw);   // dvx/dyaw
    H(1, 2) = std::sin(yaw);        // dvy/dv
    H(1, 3) = v * std::cos(yaw);    // dvy/dyaw

    Eigen::VectorXd innovation = vel_meas - z_pred;
    Eigen::MatrixXd R_vel = R_.block(2, 2, 2, 2);
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_vel;

    if (validateMeasurement(innovation, S, config_.thresholds.vel_gating_threshold)) {
        performUpdate(vel_meas, H, R_vel, innovation);
    }
}

bool ExtendedKalmanFilter::validateMeasurement(const Eigen::VectorXd& innovation, 
                                             const Eigen::MatrixXd& S, 
                                             double threshold) {
    // 马氏距离检查 (Mahalanobis Distance)
    // d^2 = inn' * S^-1 * inn
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) return false;
    
    // 技巧：避免直接求逆，解线性方程 L * y = inn，则 d^2 = ||y||^2
    Eigen::VectorXd y = llt.matrixL().solve(innovation);
    double d2 = y.squaredNorm();
    
    return d2 < (threshold * threshold);
}

// ---------------------------------------------------------
// 4. 核心更新逻辑 (Joseph Form & 数值稳定)
// ---------------------------------------------------------
void ExtendedKalmanFilter::performUpdate(const Eigen::VectorXd& z, 
                                       const Eigen::MatrixXd& H, 
                                       const Eigen::MatrixXd& R, 
                                       const Eigen::VectorXd& innovation) {
    // 1. 计算卡尔曼增益 K = PH'S^-1
    Eigen::MatrixXd PHt = P_ * H.transpose();
    Eigen::MatrixXd S = H * PHt + R;
    Eigen::MatrixXd K;

    // 使用 Cholesky 分解求逆，如果失败回退到 LDLT
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() == Eigen::Success) {
        K = PHt * llt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    } else {
        // 保底方案
        K = PHt * S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    }

    // 2. 更新状态
    x_ += K * innovation;
    x_(3) = normalizeAngle(x_(3)); // 立即归一化角度

    // 3. 更新协方差 (Joseph Form: P = (I-KH)P(I-KH)' + KRK')
    // 这种形式保证了 P 永远对称且正定
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // 4. 最后的稳定性保护
    P_ = 0.5 * (P_ + P_.transpose());
    // 对角线正则化 (防止过拟合/奇异)
    for(int i=0; i<5; ++i) {
        if(P_(i,i) < 1e-9) P_(i,i) = 1e-9;
    }
}

ExtendedKalmanFilter::DebugInfo ExtendedKalmanFilter::getDebugInfo() const {
    DebugInfo info;
    info.is_initialized = is_initialized_;
    info.nis = nis_;
    info.last_mahalanobis_dist = last_mahalanobis_dist_;
    info.last_update_valid = last_update_valid_;
    
    // 构建调试信息字符串
    std::ostringstream oss;
    oss << "EKF Status: ";
    if (is_initialized_) {
        oss << "Initialized, ";
        oss << "NIS=" << std::fixed << std::setprecision(3) << nis_ << ", ";
        oss << "Mahalanobis=" << std::fixed << std::setprecision(3) << last_mahalanobis_dist_ << ", ";
        oss << "Update=" << (last_update_valid_ ? "Valid" : "Invalid");
    } else {
        oss << "Not Initialized";
    }
    info.debug_message = oss.str();
    
    return info;
}

Eigen::Vector4d ExtendedKalmanFilter::getCartesianState() const {
    if (!is_initialized_) {
        return Eigen::Vector4d::Zero();
    }
    
    double v = x_(2);
    double yaw = x_(3);
    
    Eigen::Vector4d cart_state;
    cart_state(0) = x_(0); // px
    cart_state(1) = x_(1); // py
    cart_state(2) = v * cos(yaw); // vx
    cart_state(3) = v * sin(yaw); // vy
    
    return cart_state;
}