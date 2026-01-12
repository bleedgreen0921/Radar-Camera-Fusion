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
        // 状态维度: [px, py, v, yaw, yaw_rate]
        int state_dim = 5;    
        // 测量维度: [px, py, vx, vy]
        int meas_dim = 4;     

        // --- 1. 过程噪声参数 (Process Noise) ---
        struct ProcessNoise {
            // nuScenes城市路况，加减速频繁，设为 2.0 m/s^2
            double std_a = 2.0;             
            // 转弯和变道频繁，角加速度噪声设为 1.5 rad/s^2
            double std_yawdd = 1.5;         
            // 高速行驶时模型更稳定，噪声衰减因子
            double high_speed_decay_factor = 0.8; 
            // 急转弯时模型不准，噪声增强因子
            double turn_amplification_factor = 2.0; 
        } process_noise;
        
        // --- 2. 测量噪声参数 (Measurement Noise) ---
        // 基于 PointPillars/CenterPoint 等检测器在 nuScenes 上的典型误差
        struct MeasurementNoise {
            double std_px = 0.5;    // 激光/雷达位置误差 X (m)
            double std_py = 0.5;    // 激光/雷达位置误差 Y (m)
            double std_vx = 1.0;    // 速度测量误差 X (m/s)
            double std_vy = 1.0;    // 速度测量误差 Y (m/s)
        } meas_noise;
        
        // --- 3. 初始化参数 (Initialization) ---
        struct InitParams {
            // 速度阈值：大于 1.0 m/s 时才信任速度矢量计算的航向角
            double min_speed_for_yaw = 1.0;        
            
            // 初始协方差设置
            double initial_std_v = 5.0;            // 初始速度不确定性
            double initial_std_yaw = M_PI;         // 初始航向不确定性 (静止时为PI)
            double initial_std_yawrate = 1.0;      // 初始转向率不确定性
            
            // 相关性系数
            double position_correlation = 0.0;     // 初始位置-速度相关性
        } init_params;
        
        // --- 4. 阈值与边界 (Thresholds) ---
        struct Thresholds {
            // CTRV模型切换阈值：角速度 < 0.001 rad/s 视为直线
            double epsilon_yaw_rate = 0.001;       
            
            // 速度区间定义 (m/s)
            double speed_static = 1.0;             // 静止/蠕行上限
            double speed_city = 15.0;              // 城市巡航上限
            double speed_highway = 22.0;           // 高速公路下限 (80km/h)

            // 门控阈值 (Chi-square dist, 4-sigma)
            double pos_gating_threshold = 4.0;     
            double vel_gating_threshold = 4.0;     
        } thresholds;

        // --- 5. 数值稳定性边界 ---
        struct NoiseBounds {
            double min_covariance = 1e-9;          // 最小协方差对角线值
        } bounds;
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