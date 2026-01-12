#ifndef RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H
#define RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <iomanip>

class ExtendedKalmanFilter {
public:
    // ==========================================
    // EKF 配置结构体 (针对 nuScenes 优化)
    // ==========================================
    struct EKFConfig {
        int state_dim;
    int meas_dim;

    struct ProcessNoise {
        double std_a;
        double std_yawdd;
        double high_speed_decay_factor;
        double turn_amplification_factor;
    } process_noise;

    struct MeasurementNoise {
        double std_px;
        double std_py;
        double std_vx;
        double std_vy;
    } meas_noise;

    struct InitParams {
        double min_speed_for_yaw;
        double initial_std_v;
        double initial_std_yaw;
        double initial_std_yawrate;
        double position_correlation;
    } init_params;

    struct Thresholds {
        double epsilon_yaw_rate;
        double speed_static;
        double speed_city;
        double speed_highway;
        double pos_gating_threshold;
        double vel_gating_threshold;
    } thresholds;

    struct NoiseBounds {
        double min_covariance;
    } bounds;

    // 2. 添加构造函数进行统一初始化
    EKFConfig() {
        state_dim = 5;
        meas_dim = 4;
        
        // 使用聚合初始化列表
        process_noise = {2.0, 1.5, 0.8, 2.0};
        meas_noise = {0.5, 0.5, 1.0, 1.0};
        init_params = {1.0, 5.0, M_PI, 1.0, 0.0};
        thresholds = {0.001, 1.0, 15.0, 22.0, 4.0, 4.0};
        bounds = {1e-9};
    }
    };

    // ==========================================
    // 构造与析构
    // ==========================================
    explicit ExtendedKalmanFilter(const EKFConfig& config = EKFConfig());
    ~ExtendedKalmanFilter();

    // ==========================================
    // 核心接口
    // ==========================================
    
    /**
     * @brief 初始化滤波器
     * @param meas 初始测量向量 [px, py, vx, vy]
     */
    void init(const Eigen::Vector4d& meas);

    /**
     * @brief 预测阶段 (CTRV模型)
     * @param dt 时间间隔 (秒)
     */
    void predict(double dt);

    /**
     * @brief 更新阶段 (序贯更新)
     * @param meas 测量向量 [px, py, vx, vy]
     */
    void update(const Eigen::Vector4d& meas);

    // ==========================================
    // 状态获取接口
    // ==========================================
    
    // 获取原始状态向量 [px, py, v, yaw, yaw_rate]
    Eigen::VectorXd getState() const { return x_; }
    
    // 获取笛卡尔坐标系状态 [px, py, vx, vy]
    Eigen::Vector4d getCartesianState() const;
    
    // 获取状态协方差矩阵 P
    const Eigen::MatrixXd& getCovariance() const { return P_; }

    // 获取配置
    const EKFConfig& getConfig() const { return config_; }

    // ==========================================
    // 调试与监控
    // ==========================================
    struct DebugInfo {
        bool is_initialized;
        double nis;                     // 归一化新息平方
        double last_mahalanobis_dist;   // 最近一次测量的马氏距离
        bool last_update_valid;         // 最近一次更新是否有效
        std::string debug_message;      // 状态描述
    };
    
    DebugInfo getDebugInfo() const;
    bool isInitialized() const { return is_initialized_; }

private:
    // ==========================================
    // 内部核心逻辑
    // ==========================================

    // --- 初始化辅助 ---
    void initializeMeasurementNoiseMatrix();
    void initializeCovarianceMatrix(double initial_speed, double initial_yaw);
    
    // 自适应协方差计算因子
    double calculatePositionQualityFactor(double speed) const;
    double calculateAdaptiveYawUncertainty(double speed) const;
    double calculateAdaptiveSpeedUncertainty(double speed) const;
    void initializeCrossCorrelations(double speed, double yaw, double speed_std, double yaw_std);
    void ensureCovarianceProperties();

    // --- 预测辅助 ---
    // CTRV 物理模型推演
    Eigen::VectorXd calculateCTRVModel(const Eigen::VectorXd& x, double dt);
    
    // --- 更新辅助 (序贯更新) ---
    void updatePosition(const Eigen::Vector2d& pos_meas);
    void updateVelocity(const Eigen::Vector2d& vel_meas);
    
    // 核心更新算法 (Joseph Form)
    void performUpdate(const Eigen::VectorXd& z_part, 
                      const Eigen::MatrixXd& H_part, 
                      const Eigen::MatrixXd& R_part,
                      const Eigen::VectorXd& innovation);

    // 测量验证 (Gating)
    bool validateMeasurement(const Eigen::VectorXd& innovation, 
                           const Eigen::MatrixXd& S, 
                           double threshold);

    // --- 通用工具 ---
    void validateConfiguration() const;
    double normalizeAngle(double angle) const;

    // ==========================================
    // 成员变量
    // ==========================================
    
    // 配置与状态标志
    EKFConfig config_;
    bool is_initialized_;
    
    // 监控指标
    double nis_;
    double last_mahalanobis_dist_;
    bool last_update_valid_;

    // 核心矩阵
    Eigen::VectorXd x_;     // 状态向量 5x1
    Eigen::MatrixXd P_;     // 协方差矩阵 5x5
    Eigen::MatrixXd Q_;     // 过程噪声矩阵 5x5
    Eigen::MatrixXd R_;     // 测量噪声矩阵 4x4
    
    // 预分配内存 (虽然F和H是动态计算的，但可以保留成员以避免反复重分配，视实现而定)
    // 在本实现中，F和H通常在局部变量中计算以保持无状态性，或者作为成员缓存
    Eigen::MatrixXd Fj_;    // 雅可比缓存
    Eigen::MatrixXd Hj_;    // 雅可比缓存
};

#endif // RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H