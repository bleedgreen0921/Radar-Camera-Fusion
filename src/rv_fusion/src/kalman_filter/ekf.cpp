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
    if (config_.std_a <= 0) {
        throw std::invalid_argument("Linear acceleration noise must be positive");
    }
    
    if (config_.std_yawdd <= 0) {
        throw std::invalid_argument("Angular acceleration noise must be positive");
    }
    
    if (config_.meas_noise.std_px <= 0 || config_.meas_noise.std_py <= 0 ||
        config_.meas_noise.std_vx <= 0 || config_.meas_noise.std_vy <= 0) {
        throw std::invalid_argument("Measurement noise standard deviations must be positive");
    }
    
    // 输出配置警告
    if (config_.std_a < 0.5 || config_.std_a > 5.0) {
        std::cerr << "Warning: Unusual linear acceleration noise: " 
                  << config_.std_a << " m/s²" << std::endl;
    }
}

void ExtendedKalmanFilter::initializeMeasurementNoiseMatrix() {
    R_.setZero();
    R_(0, 0) = config_.meas_noise.std_px * config_.meas_noise.std_px;
    R_(1, 1) = config_.meas_noise.std_py * config_.meas_noise.std_py;
    R_(2, 2) = config_.meas_noise.std_vx * config_.meas_noise.std_vx;
    R_(3, 3) = config_.meas_noise.std_vy * config_.meas_noise.std_vy;
}

double ExtendedKalmanFilter::normalizeAngle(double angle) const {
    // 健壮的角度归一化实现
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
    
    // 改进的方向角初始化：低速时使用保守估计
    double yaw = 0.0;
    bool low_speed_init = false;
    
    if (v > config_.init_params.min_speed_for_yaw) {
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
    // 低速时位置测量更可靠（目标运动慢）
    // 高速时位置测量噪声影响更大
    const double min_factor = 0.8;
    const double max_factor = 1.5;
    
    // 非线性映射：速度越快，位置不确定性越大
    double factor = 1.0 + 0.5 * (speed / 10.0); // 参考速度10m/s
    return std::clamp(factor, min_factor, max_factor);
}

// 辅助函数：计算自适应速度不确定性（速度模长）
double ExtendedKalmanFilter::calculateAdaptiveSpeedUncertainty(double speed) const {
    double base_uncertainty = config_.init_params.initial_std_v;
    
    // 雷达速度测量特性：
    // - 低速时：多普勒效应准确，速度模长测量精确
    // - 高速时：存在拖尾效应，速度模长不确定性增大
    
    if (speed < config_.init_params.min_speed_for_yaw) {
        // 低速情况：速度模长测量准确，减小不确定性
        // 多普勒雷达在低速时能准确测量径向速度
        double low_speed_factor = 0.3;  // 低速时不确定性减小到30%
        return base_uncertainty * low_speed_factor;
    }
    else if (speed > 15.0) {  // 高速阈值可配置
        // 高速情况：存在拖尾效应，增大不确定性
        double high_speed_factor = 1.0 + (speed - 15.0) / 10.0;  // 线性增长
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
    
    // 雷达方向测量特性：
    // - 低速时：方向估计不可靠，不确定性极大
    // - 高速时：方向估计相对可靠，不确定性较小
    
    if (speed < config_.init_params.min_speed_for_yaw) {
        // 低速情况：方向完全不可靠
        // 当速度很小时，从速度矢量计算方向角毫无意义
        return M_PI;  // 180度，表示方向完全不确定
    }
    else if (speed < 2.0) {
        // 较低速：方向估计仍然不可靠，但比极低速稍好
        double speed_ratio = (speed - config_.init_params.min_speed_for_yaw) / 
                            (2.0 - config_.init_params.min_speed_for_yaw);
        double max_uncertainty = M_PI;  // 180度
        double min_uncertainty_at_low_speed = M_PI / 2.0;  // 90度
        
        // 从完全不确定平滑过渡到较大不确定性
        return max_uncertainty - speed_ratio * (max_uncertainty - min_uncertainty_at_low_speed);
    }
    else if (speed > 20.0) {
        // 超高速：可能存在机动不确定性增加
        double high_speed_factor = 1.0 + (speed - 20.0) / 20.0;  // 适度增加
        return base_uncertainty * std::min(high_speed_factor, 2.0);
    }
    else {
        // 正常速度范围：使用基础不确定性
        return base_uncertainty;
    }
}

// 辅助函数：计算位置质量因子（考虑拖尾效应）
double ExtendedKalmanFilter::calculatePositionQualityFactor(double speed) const {
    // 雷达位置测量特性：
    // - 低速时：位置测量相对准确（目标运动慢）
    // - 高速时：存在拖尾效应，位置不确定性增大
    
    const double min_factor = 0.8;   // 最小质量因子
    const double max_factor = 2.5;   // 最大质量因子（考虑拖尾效应）
    
    if (speed < 5.0) {
        // 低速：位置测量质量较好
        return 1.0;
    }
    else if (speed > 20.0) {
        // 高速：拖尾效应显著，位置不确定性增大
        double high_speed_penalty = 1.0 + (speed - 20.0) / 30.0;
        return std::min(max_factor, high_speed_penalty);
    }
    else {
        // 中等速度：线性过渡
        double speed_ratio = (speed - 5.0) / 15.0;  // 5-20m/s范围内
        return 1.0 + speed_ratio * (max_factor - 1.0);
    }
}

// 辅助函数：初始化状态间相关性
void ExtendedKalmanFilter::initializeCrossCorrelations(double speed, double yaw, 
                                                       double speed_std, double yaw_std) {
    if (speed > config_.predict_params.epsilon) { // 使用配置的epsilon阈值
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

void ExtendedKalmanFilter::predict(double dt) {
    // 1. 增强的输入验证
    if (!validatePredictionInputs(dt)) {
        return;
    }

    // 2. 保存预测前的完整状态快照
    PredictionState state_before = createStateSnapshot();
    
    // 3. 状态预测（使用预测前的状态）
    Eigen::VectorXd x_pred = ctrvProcessModel(state_before.x, dt);
    x_pred(3) = normalizeAngle(x_pred(3));  // 预测后立即归一化角度
    
    // 4. 使用预测前的状态计算雅可比和过程噪声
    Eigen::MatrixXd F_j = computeJacobian(state_before, dt);
    Eigen::MatrixXd Q = computeProcessNoise(state_before, dt);
    
    // 5. 协方差预测
    Eigen::MatrixXd P_pred = predictCovariance(F_j, P_, Q);
    
    // 6. 验证预测结果
    if (validatePredictionResults(x_pred, P_pred)) {
        // 更新状态和协方差
        x_ = x_pred;
        P_ = P_pred;
        
        // 记录预测信息
        recordPredictionMetrics(dt, state_before, x_pred);
    } else {
        handlePredictionFailure(dt, state_before);
    }
}

bool ExtendedKalmanFilter::validatePredictionInputs(double dt) const {
    if (!is_initialized_) {
        std::cerr << "EKF prediction failed: Filter not initialized" << std::endl;
        return false;
    }
    
    if (dt <= config_.predict_params.epsilon) {
        std::cerr << "EKF prediction failed: Invalid time step dt=" << dt 
                  << " (must be > " << config_.predict_params.epsilon << ")" << std::endl;
        return false;
    }
    
    if (dt > 1.0) {  // 最大时间步长限制
        std::cerr << "EKF prediction warning: Large time step dt=" << dt 
                  << ", prediction accuracy may degrade" << std::endl;
    }
    
    // 检查状态是否有效
    if (x_.hasNaN() || P_.hasNaN()) {
        std::cerr << "EKF prediction failed: State or covariance contains NaN" << std::endl;
        return false;
    }
    
    return true;
}

struct PredictionState {
    Eigen::VectorXd x;
    double timestamp;
    bool is_turning;
    double effective_yaw_rate;
};

ExtendedKalmanFilter::PredictionState ExtendedKalmanFilter::createStateSnapshot() const {
    PredictionState snapshot;
    snapshot.x = x_;  // 当前状态
    snapshot.timestamp = getCurrentTime();  // 需要实现时间管理
    
    double yawd = x_(4);
    snapshot.is_turning = (std::fabs(yawd) > config_.predict_params.min_yaw_rate);
    
    // 处理接近零的转向率
    if (snapshot.is_turning) {
        snapshot.effective_yaw_rate = yawd;
    } else {
        // 使用符号保持的小值，避免除零
        snapshot.effective_yaw_rate = (yawd >= 0 ? config_.predict_params.min_yaw_rate : 
                                      -config_.predict_params.min_yaw_rate);
    }
    
    return snapshot;
}

Eigen::VectorXd ExtendedKalmanFilter::ctrvProcessModel(const Eigen::VectorXd& x, double dt) const {
    Eigen::VectorXd x_pred(5);
    double px = x(0), py = x(1), v = x(2), yaw = x(3), yawd = x(4);
    
    // 使用数值稳定的转向率处理
    bool is_turning = (std::fabs(yawd) > config_.predict_params.min_yaw_rate);
    double effective_yawd = is_turning ? yawd : 
                           (yawd >= 0 ? config_.predict_params.min_yaw_rate : 
                            -config_.predict_params.min_yaw_rate);

    if (is_turning) {
        // 转弯运动：使用精确的三角函数计算
        double yaw_new = yaw + yawd * dt;
        double delta_yaw = yawd * dt;
        
        // 使用数值稳定的公式
        if (std::fabs(delta_yaw) < 1e-2) {  // 小角度近似
            // 泰勒展开到二阶
            x_pred(0) = px + v * dt * (std::cos(yaw) - 0.5 * delta_yaw * std::sin(yaw));
            x_pred(1) = py + v * dt * (std::sin(yaw) + 0.5 * delta_yaw * std::cos(yaw));
        } else {
            // 精确公式
            x_pred(0) = px + (v / effective_yawd) * (std::sin(yaw_new) - std::sin(yaw));
            x_pred(1) = py + (v / effective_yawd) * (std::cos(yaw) - std::cos(yaw_new));
        }
        
        x_pred(3) = yaw_new;  // 新角度
    } 
    else {
        // 直线运动
        x_pred(0) = px + v * dt * std::cos(yaw);
        x_pred(1) = py + v * dt * std::sin(yaw);
        x_pred(3) = yaw;  // 角度不变
    }

    // 速度和转向率保持不变（CTRV假设）
    x_pred(2) = v;
    x_pred(4) = yawd;
    return x_pred;
}

void ExtendedKalmanFilter::computeProcessNoise(double dt, const PredictionState& state) {
    // 1. 计算自适应噪声参数
    AdaptiveNoiseParams noise_params = computeAdaptiveNoiseParams(dt, state);
    
    // 2. 使用正确的噪声传播模型
    Q_ = computeProcessNoiseMatrix(dt, state, noise_params);
    
    // 3. 确保数值性质
    enforceNoiseMatrixProperties(Q_);
}

struct AdaptiveNoiseParams {
    double effective_std_a;      // 有效的线性加速度噪声
    double effective_std_yawdd;  // 有效的角加速度噪声
    double maneuver_confidence;   // 机动置信度 [0,1]
};

ExtendedKalmanFilter::AdaptiveNoiseParams 
ExtendedKalmanFilter::computeAdaptiveNoiseParams(double dt, const PredictionState& state) const {
    AdaptiveNoiseParams params;
    
    // 基础噪声参数
    params.effective_std_a = config_.std_a;
    params.effective_std_yawdd = config_.std_yawdd;
    
    // 基于运动状态的自适应调整
    params.maneuver_confidence = calculateManeuverConfidence(state, dt);
    
    // 时间相关的调整：长时间预测需要更大的过程噪声
    double time_factor = std::sqrt(std::max(1.0, dt / 0.1)); // 以0.1s为基准
    
    // 转向运动需要更大的噪声
    if (state.is_turning) {
        double turn_intensity = std::min(2.0, 1.0 + std::fabs(state.effective_yaw_rate) * 2.0);
        params.effective_std_a *= time_factor * turn_intensity;
        params.effective_std_yawdd *= time_factor * turn_intensity;
    } else {
        params.effective_std_a *= time_factor;
        params.effective_std_yawdd *= time_factor;
    }
    
    // 基于速度的调整：高速时需要更精确的模型
    double speed = state.x(2);
    if (speed > 10.0) { // 高速行驶
        params.effective_std_a *= 0.8;  // 减小噪声，模型更可信
    } else if (speed < 1.0) { // 低速或静止
        params.effective_std_a *= 1.5;  // 增大噪声，模型不确定性大
    }
    
    return params;
}

double ExtendedKalmanFilter::calculateManeuverConfidence(const PredictionState& state, double dt) const {
    double confidence = 1.0;
    
    // 基于转向率的机动检测
    double yaw_rate = std::fabs(state.x(4));
    if (yaw_rate > 0.5) { // 明显转向
        confidence = std::max(0.3, 1.0 - yaw_rate / M_PI);
    }
    
    // 基于加速度变化的机动检测（需要历史信息）
    // 这里可以扩展为基于多帧数据的机动检测
    
    return confidence;
}

Eigen::MatrixXd ExtendedKalmanFilter::computeProcessNoiseMatrix(double dt, 
                                                                const PredictionState& state,
                                                                const AdaptiveNoiseParams& noise_params) const {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);
    
    double var_a = noise_params.effective_std_a * noise_params.effective_std_a;
    double var_yawdd = noise_params.effective_std_yawdd * noise_params.effective_std_yawdd;
    
    // 方法1: 使用噪声驱动矩阵方法（更理论正确）
    if (use_analytic_noise_model_) {
        Q = computeNoiseByDrivingMatrix(dt, state, var_a, var_yawdd);
    } 
    // 方法2: 使用离散时间积分方法（当前方法的改进版）
    else {
        Q = computeNoiseByDiscreteIntegration(dt, state, var_a, var_yawdd);
    }
    
    return Q;
}

Eigen::MatrixXd ExtendedKalmanFilter::computeNoiseByDiscreteIntegration(double dt,
                                                                       const PredictionState& state,
                                                                       double var_a, double var_yawdd) const {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);
    
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    // 1. 线性加速度噪声贡献（更合理的模型）
    // 加速度噪声通过速度影响位置，使用平均方向的概念
    double avg_yaw = computeAverageYawDuringStep(state, dt);
    double c_avg = std::cos(avg_yaw);
    double s_avg = std::sin(avg_yaw);
    
    // 位置噪声：来自速度积分（更物理的模型）
    Q(0, 0) = (1.0/4.0) * dt4 * var_a * c_avg * c_avg;
    Q(1, 1) = (1.0/4.0) * dt4 * var_a * s_avg * s_avg;
    Q(0, 1) = (1.0/4.0) * dt4 * var_a * c_avg * s_avg;
    Q(1, 0) = Q(0, 1);
    
    // 速度噪声：直接来自加速度积分
    Q(2, 2) = dt2 * var_a;
    
    // 位置-速度协方差
    Q(0, 2) = (1.0/2.0) * dt3 * var_a * c_avg;
    Q(2, 0) = Q(0, 2);
    Q(1, 2) = (1.0/2.0) * dt3 * var_a * s_avg;
    Q(2, 1) = Q(1, 2);
    
    // 2. 角加速度噪声贡献
    Q(3, 3) = (1.0/4.0) * dt4 * var_yawdd;
    Q(4, 4) = dt2 * var_yawdd;
    Q(3, 4) = (1.0/2.0) * dt3 * var_yawdd;
    Q(4, 3) = Q(3, 4);
    
    // 3. 交叉项：线性加速度和角加速度的相关性（通常假设为0）
    // 如果需要，可以在这里添加相关性模型
    
    return Q;
}

double ExtendedKalmanFilter::computeAverageYawDuringStep(const PredictionState& state, double dt) const {
    // 计算预测步长内的平均方向，而不是使用固定方向
    double yaw_start = state.x(3);
    double yaw_rate = state.effective_yaw_rate;
    double yaw_end = yaw_start + yaw_rate * dt;
    
    // 使用角度平均值
    return 0.5 * (yaw_start + yaw_end);
}

Eigen::MatrixXd ExtendedKalmanFilter::computeNoiseByDrivingMatrix(double dt,
                                                                 const PredictionState& state,
                                                                 double var_a, double var_yawdd) const {
    // CTRV模型的噪声驱动矩阵G
    // 噪声向量: [线性加速度噪声, 角加速度噪声]^T
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(5, 2);
    
    double yaw = state.x(3);
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    
    // 线性加速度噪声对状态的影响
    G(0, 0) = 0.5 * dt * dt * c;  // 对位置x的影响
    G(1, 0) = 0.5 * dt * dt * s;  // 对位置y的影响
    G(2, 0) = dt;                 // 对速度的影响
    
    // 角加速度噪声对状态的影响
    G(3, 1) = 0.5 * dt * dt;     // 对角度的影响
    G(4, 1) = dt;                // 对转向率的影响
    
    // 噪声源协方差矩阵
    Eigen::Matrix2d Qv;
    Qv << var_a, 0,
          0, var_yawdd;
    
    // 过程噪声矩阵: Q = G * Qv * G^T
    return G * Qv * G.transpose();
}

void ExtendedKalmanFilter::enforceNoiseMatrixProperties(Eigen::MatrixXd& Q) const {
    // 1. 确保对称
    Q = 0.5 * (Q + Q.transpose());
    
    // 2. 检查正定性
    Eigen::LLT<Eigen::MatrixXd> llt(Q);
    if (llt.info() != Eigen::Success) {
        // 添加小的正则化项
        double regularization = findAppropriateRegularization(Q);
        Q += Eigen::MatrixXd::Identity(Q.rows(), Q.cols()) * regularization;
        
        std::cout << "Process noise matrix regularized with: " << regularization << std::endl;
    }
    
    // 3. 边界检查：防止过小或过大的噪声
    enforceNoiseBounds(Q);
}

double ExtendedKalmanFilter::findAppropriateRegularization(const Eigen::MatrixXd& Q) const {
    // 基于矩阵范数计算合适的正则化量
    double norm = Q.norm();
    double regularization = std::max(1e-10, norm * 1e-6);
    
    // 确保至少比最小特征值大
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(Q);
    if (eigensolver.info() == Eigen::Success) {
        double min_eigenvalue = eigensolver.eigenvalues().minCoeff();
        if (min_eigenvalue < 0) {
            regularization = std::max(regularization, -min_eigenvalue + 1e-10);
        }
    }
    
    return regularization;
}

void ExtendedKalmanFilter::enforceNoiseBounds(Eigen::MatrixXd& Q) const {
    // 设置合理的噪声边界
    const double max_position_noise = 100.0;  // m^2
    const double max_velocity_noise = 100.0;  // (m/s)^2
    const double max_angle_noise = M_PI * M_PI; // rad^2
    
    // 位置噪声边界
    for (int i = 0; i < 2; ++i) {
        if (Q(i, i) > max_position_noise) {
            Q(i, i) = max_position_noise;
        } else if (Q(i, i) < 1e-6) {
            Q(i, i) = 1e-6;  // 最小噪声
        }
    }
    
    // 速度噪声边界
    if (Q(2, 2) > max_velocity_noise) Q(2, 2) = max_velocity_noise;
    if (Q(2, 2) < 1e-6) Q(2, 2) = 1e-6;
    
    // 角度噪声边界
    if (Q(3, 3) > max_angle_noise) Q(3, 3) = max_angle_noise;
    if (Q(3, 3) < 1e-8) Q(3, 3) = 1e-8;
    
    if (Q(4, 4) > max_angle_noise) Q(4, 4) = max_angle_noise;
    if (Q(4, 4) < 1e-8) Q(4, 4) = 1e-8;
}

Eigen::MatrixXd ExtendedKalmanFilter::computeJacobian(const PredictionState& state, double dt) const {
    Eigen::MatrixXd F_j = Eigen::MatrixXd::Identity(5, 5);
    
    double v = state.x(2);
    double yaw = state.x(3);
    double yawd = state.effective_yaw_rate;
    
    if (state.is_turning) {
        double yaw_new = yaw + yawd * dt;
        
        // 使用数值稳定的偏导数计算
        F_j(0, 2) = (std::sin(yaw_new) - std::sin(yaw)) / yawd;  // dpx/dv
        F_j(0, 3) = (v / yawd) * (std::cos(yaw_new) - std::cos(yaw));  // dpx/dyaw
        F_j(0, 4) = computeDpxDyawnumerically(state, dt);  // 数值计算复杂的偏导数
        
        F_j(1, 2) = (std::cos(yaw) - std::cos(yaw_new)) / yawd;  // dpy/dv
        F_j(1, 3) = (v / yawd) * (std::sin(yaw_new) - std::sin(yaw));  // dpy/dyaw
        F_j(1, 4) = computeDpyDyawnumerically(state, dt);  // 数值计算
        
        F_j(3, 4) = dt;  // dyaw/dyawd
    } else {
        // 直线运动的雅可比
        F_j(0, 2) = dt * std::cos(yaw);
        F_j(0, 3) = -v * dt * std::sin(yaw);
        F_j(1, 2) = dt * std::sin(yaw);
        F_j(1, 3) = v * dt * std::cos(yaw);
        F_j(3, 4) = dt;
    }
    
    return F_j;
}

// 数值方法计算复杂偏导数
double ExtendedKalmanFilter::computeDpxDyawnumerically(const PredictionState& state, double dt) const {
    const double h = 1e-6;  // 微分量
    Eigen::VectorXd x_plus = state.x;
    x_plus(4) += h;  // 对yawd进行微小扰动
    
    Eigen::VectorXd x_pred_nominal = ctrvProcessModel(state.x, dt);
    Eigen::VectorXd x_pred_perturbed = ctrvProcessModel(x_plus, dt);
    
    return (x_pred_perturbed(0) - x_pred_nominal(0)) / h;
}

Eigen::MatrixXd ExtendedKalmanFilter::predictCovariance(const Eigen::MatrixXd& F_j, 
                                                       const Eigen::MatrixXd& P, 
                                                       const Eigen::MatrixXd& Q) const {
    // 1. 标准协方差预测
    Eigen::MatrixXd P_pred = F_j * P * F_j.transpose() + Q;
    
    // 2. 确保对称性
    P_pred = 0.5 * (P_pred + P_pred.transpose());
    
    // 3. 检查正定性
    if (!isPositiveDefinite(P_pred)) {
        std::cerr << "Warning: Covariance matrix lost positive definiteness during prediction" << std::endl;
        P_pred = enforcePositiveDefiniteness(P_pred);
    }
    
    // 4. 边界检查
    P_pred = enforceCovarianceBounds(P_pred);
    
    return P_pred;
}

bool ExtendedKalmanFilter::isPositiveDefinite(const Eigen::MatrixXd& matrix) const {
    Eigen::LLT<Eigen::MatrixXd> llt(matrix);
    return (llt.info() == Eigen::Success);
}

Eigen::MatrixXd ExtendedKalmanFilter::enforcePositiveDefiniteness(const Eigen::MatrixXd& matrix) const {
    Eigen::MatrixXd result = matrix;
    
    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(result);
    if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition failed during covariance enforcement");
    }
    
    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    double min_eigenvalue = eigenvalues.minCoeff();
    
    // 如果最小特征值为负，添加正则化
    if (min_eigenvalue < 1e-8) {
        double regularization = std::max(1e-8, -min_eigenvalue + 1e-8);
        result += Eigen::MatrixXd::Identity(result.rows(), result.cols()) * regularization;
    }
    
    return result;
}

bool ExtendedKalmanFilter::validatePredictionResults(const Eigen::VectorXd& x_pred, 
                                                   const Eigen::MatrixXd& P_pred) const {
    // 检查状态是否有效
    if (x_pred.hasNaN()) {
        std::cerr << "Prediction failed: Predicted state contains NaN" << std::endl;
        return false;
    }
    
    // 检查协方差是否有效
    if (P_pred.hasNaN() || !isPositiveDefinite(P_pred)) {
        std::cerr << "Prediction failed: Predicted covariance is invalid" << std::endl;
        return false;
    }
    
    // 检查状态边界（物理合理性）
    if (!isStatePhysicallyValid(x_pred)) {
        std::cerr << "Prediction failed: Predicted state is physically invalid" << std::endl;
        return false;
    }
    
    return true;
}

bool ExtendedKalmanFilter::isStatePhysicallyValid(const Eigen::VectorXd& x) const {
    // 速度非负
    if (x(2) < 0) return false;
    
    // 角度在合理范围内
    double yaw = x(3);
    if (yaw < -2*M_PI || yaw > 2*M_PI) return false;
    
    // 转向率在合理范围内
    double yawd = x(4);
    if (std::fabs(yawd) > 2*M_PI) return false;  // 最大2π rad/s
    
    return true;
}

void ExtendedKalmanFilter::handlePredictionFailure(double dt, const PredictionState& state_before) {
    std::cerr << "Prediction failure detected, applying recovery strategy" << std::endl;
    
    // 策略1：使用简化的恒定速度模型
    try {
        Eigen::VectorXd x_simple = constantVelocityModel(state_before.x, dt);
        Eigen::MatrixXd P_simple = simpleCovariancePrediction(P_, dt);
        
        if (validatePredictionResults(x_simple, P_simple)) {
            x_ = x_simple;
            P_ = P_simple;
            std::cerr << "Recovery successful: Using constant velocity model" << std::endl;
            return;
        }
    } catch (...) {
        // 简化模型也失败
    }
    
    // 策略2：保持当前状态，增大不确定性
    P_ = P_ * 1.5;  // 增大协方差
    std::cerr << "Recovery: Maintaining state with increased uncertainty" << std::endl;
}

void ExtendedKalmanFilter::update(const Eigen::Vector4d& meas) {
    if (!is_initialized_) {
        std::cerr << "EKF not initialized, skipping update" << std::endl;
        return;
    }
    
    // 拆分测量值
    Eigen::Vector2d pos_meas = meas.head(2);
    Eigen::Vector2d vel_meas = meas.tail(2);
    
    // 保存预测状态用于速度更新的线性化
    Eigen::VectorXd x_pred = x_;
    
    // 序贯更新：先位置后速度
    updatePosition(pos_meas);
    updateVelocity(vel_meas);
}

void ExtendedKalmanFilter::updatePosition(const Eigen::Vector2d& pos_meas) {
    // 位置测量模型：H_pos = [I2x2, 0]
    Eigen::MatrixXd H_pos = Eigen::MatrixXd::Zero(2, 5);
    H_pos(0, 0) = 1.0;
    H_pos(1, 1) = 1.0;
    
    // 位置测量预测
    Eigen::Vector2d pos_pred = x_.head(2);
    
    // 位置测量噪声
    Eigen::MatrixXd R_pos = R_.block(0, 0, 2, 2);
    
    // 计算创新协方差
    Eigen::MatrixXd S_pos = H_pos * P_ * H_pos.transpose() + R_pos;
    
    // 测量验证
    Eigen::Vector2d innovation_pos = pos_meas - pos_pred;
    last_update_valid_ = validateMeasurement(pos_meas, pos_pred, S_pos, 
                                          config_.update_params.pos_gating_threshold);
    
    if (last_update_valid_) {
        performUpdate(pos_meas, H_pos, R_pos, innovation_pos);
    } else {
        std::cerr << "Position measurement rejected by gating test" << std::endl;
    }
}

void ExtendedKalmanFilter::updateVelocity(const Eigen::Vector2d& vel_meas) {
    // 速度测量模型：非线性函数
    double v = x_(2);
    double yaw = x_(3);
    
    // 速度测量预测
    Eigen::Vector2d vel_pred = computeVelocityFromState();
    
    // 速度测量雅可比
    Eigen::MatrixXd H_vel = Eigen::MatrixXd::Zero(2, 5);
    H_vel(0, 2) = std::cos(yaw);      // dvx/dv
    H_vel(0, 3) = -v * std::sin(yaw); // dvx/dyaw
    H_vel(1, 2) = std::sin(yaw);      // dvy/dv  
    H_vel(1, 3) = v * std::cos(yaw);  // dvy/dyaw
    
    // 速度测量噪声
    Eigen::MatrixXd R_vel = R_.block(2, 2, 2, 2);
    
    // 计算创新协方差
    Eigen::MatrixXd S_vel = H_vel * P_ * H_vel.transpose() + R_vel;
    
    // 测量验证
    Eigen::Vector2d innovation_vel = vel_meas - vel_pred;
    last_update_valid_ = validateMeasurement(vel_meas, vel_pred, S_vel,
                                          config_.update_params.vel_gating_threshold);
    
    if (last_update_valid_) {
        performUpdate(vel_meas, H_vel, R_vel, innovation_vel);
    } else {
        std::cerr << "Velocity measurement rejected by gating test" << std::endl;
    }
}

Eigen::Vector2d ExtendedKalmanFilter::computeVelocityFromState() const {
    double v = x_(2);
    double yaw = x_(3);
    Eigen::Vector2d vel;
    vel << v * std::cos(yaw), v * std::sin(yaw);
    return vel;
}

bool ExtendedKalmanFilter::validateMeasurement(const Eigen::VectorXd& meas, 
                                             const Eigen::VectorXd& pred,
                                             const Eigen::MatrixXd& S,
                                             double gating_threshold) {
    Eigen::VectorXd innovation = meas - pred;
    
    // 使用LLT分解稳定求逆
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) {
        std::cerr << "Innovation covariance matrix is not positive definite" << std::endl;
        return false;
    }
    
    Eigen::VectorXd temp = llt.matrixL().solve(innovation);
    last_mahalanobis_dist_ = temp.squaredNorm();
    
    double threshold_sq = gating_threshold * gating_threshold;
    return (last_mahalanobis_dist_ <= threshold_sq);
}

void ExtendedKalmanFilter::performUpdate(const Eigen::VectorXd& z_part, 
                                       const Eigen::MatrixXd& H_part, 
                                       const Eigen::MatrixXd& R_part,
                                       const Eigen::VectorXd& innovation) {
    if (!is_initialized_) return;

    int meas_dim = z_part.size();
    int state_dim = x_.size();

    // 1. 计算卡尔曼增益
    Eigen::MatrixXd Ht = H_part.transpose();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd S = H_part * PHt + R_part;

    // 使用稳定的矩阵求逆方法
    Eigen::MatrixXd K;
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() == Eigen::Success) {
        // 使用Cholesky分解求逆
        K = PHt * llt.solve(Eigen::MatrixXd::Identity(meas_dim, meas_dim));
    } else {
        // 回退到LDLT分解
        Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
        if (ldlt.info() == Eigen::Success) {
            K = PHt * ldlt.solve(Eigen::MatrixXd::Identity(meas_dim, meas_dim));
        } else {
            // 最后回退到伪逆（不推荐，但作为保底）
            std::cerr << "Warning: Using pseudo-inverse for Kalman gain calculation" << std::endl;
            K = PHt * S.completeOrthogonalDecomposition().pseudoInverse();
        }
    }

    // 2. 计算NIS（归一化新息平方）
    Eigen::VectorXd S_innovation;
    Eigen::LLT<Eigen::MatrixXd> llt_S(S);
    if (llt_S.info() == Eigen::Success) {
        S_innovation = llt_S.matrixL().solve(innovation);
    } else {
        Eigen::LDLT<Eigen::MatrixXd> ldlt_S(S);
        S_innovation = ldlt_S.solve(innovation);
    }
    
    nis_ = innovation.transpose() * S_innovation;

    // 3. 更新状态
    x_ += K * innovation;
    
    // 归一化角度状态
    x_(3) = normalizeAngle(x_(3));

    // 4. 使用Joseph形式更新协方差（数值更稳定）
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    Eigen::MatrixXd I_KH = I - K * H_part;
    
    P_ = I_KH * P_ * I_KH.transpose() + K * R_part * K.transpose();
    
    // 确保协方差矩阵对称
    P_ = 0.5 * (P_ + P_.transpose());
    
    // 添加小的正则化项防止数值问题
    P_ += Eigen::MatrixXd::Identity(state_dim, state_dim) * 1e-8;
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