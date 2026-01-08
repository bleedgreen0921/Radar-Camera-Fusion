#ifndef RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H
#define RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>

class ExtendedKalmanFilter {
public:
    struct EKFConfig {
        // 状态维度配置
        int state_dim = 5;    // [px, py, v, yaw, yaw_rate]
        int meas_dim = 4;     // [px, py, vx, vy]
        
        // 过程噪声参数
        double std_a = 2.0;      // 线性加速度标准差 (m/s²)
        double std_yawdd = 1.0;   // 角加速度标准差 (rad/s²)
        
        // 测量噪声参数 (基于NuScenes特性)
        struct MeasurementNoise {
            double std_px = 1.0;    // 位置x噪声 (m)
            double std_py = 0.5;    // 位置y噪声 (m) 
            double std_vx = 2.0;    // 速度x噪声 (m/s)
            double std_vy = 2.0;    // 速度y噪声 (m/s)
        } meas_noise;
        
        // 初始化参数
        struct InitParams {
            double min_speed_for_yaw = 0.5;        // 计算方向角的最小速度阈值
            double initial_std_v = 5.0;            // 初始速度标准差
            double initial_std_yaw = M_PI/4;     // 初始方向角标准差
            double initial_std_yawrate = 1.0;      // 初始转向率标准差
            double position_correlation = 0.3;    // 位置-速度相关性
            double speed_yaw_correlation = 0.2;    // 速度-方向角相关性
            double yaw_yawrate_correlation = 0.1; // 方向角-转向率相关性
            double min_speed_for_correlation = 0.1; // 相关性计算的最小速度阈值
        } init_params;
        
        // 预测参数
        struct PredictParams {
            double min_yaw_rate = 0.001;     // 最小转向率阈值 (rad/s)
            double epsilon = 1e-6;           // 数值精度阈值
        } predict_params;
        
        // 更新参数  
        struct UpdateParams {
            double pos_gating_threshold = 3.0;  // 位置门限阈值
            double vel_gating_threshold = 5.0;  // 速度门限阈值
            double nis_adaptation_threshold = 12.0; // NIS自适应阈值
        } update_params;

        struct ProcessNoiseParams {
        double std_a = 2.0;                    // 基础线性加速度噪声
        double std_yawdd = 1.0;                // 基础角加速度噪声
        double time_scaling_base = 0.1;         // 时间缩放基准
        double turn_intensity_factor = 2.0;     // 转向强度因子
        double high_speed_reduction = 0.8;      // 高速时噪声减少
        double low_speed_increase = 1.5;        // 低速时噪声增加
        bool use_analytic_model = true;         // 使用解析噪声模型
    } process_noise;

    // 噪声边界参数
    struct NoiseBounds {
        double max_position_variance = 100.0;   // 最大位置方差
        double max_velocity_variance = 100.0;    // 最大速度方差
        double max_angle_variance = M_PI * M_PI; // 最大角度方差
        double min_variance = 1e-8;             // 最小方差
    } noise_bounds;
    
    };

    explicit ExtendedKalmanFilter(const EKFConfig& config = EKFConfig());
    ~ExtendedKalmanFilter();

    void init(const Eigen::Vector4d& meas);
    void predict(double dt);
    void update(const Eigen::Vector4d& meas);
    void updatePosition(const Eigen::Vector2d& pos_meas);
    void updateVelocity(const Eigen::Vector2d& vel_meas);

    // 状态获取接口
    Eigen::VectorXd getState() const { return x_; }
    Eigen::Vector4d getCartesianState() const;
    const Eigen::MatrixXd& getCovariance() const { return P_; }

    // 滤波器状态
    bool isInitialized() const { return is_initialized_; }
    double getNIS() const { return nis_; }
    const EKFConfig& getConfig() const { return config_; }

    // 调试信息
    struct DebugInfo {
        bool is_initialized;
        double nis;
        double last_mahalanobis_dist;
        bool last_update_valid;
        std::string debug_message;
    };
    
    DebugInfo getDebugInfo() const;

private:
    // --- 配置和状态 ---
    EKFConfig config_;
    bool is_initialized_;
    double nis_;
    
    // --- 状态向量和矩阵 ---
    Eigen::VectorXd x_; // 状态: [px, py, v, yaw, yaw_rate]
    Eigen::MatrixXd P_; // 状态协方差 (5x5)
    Eigen::MatrixXd Q_; // 过程噪声协方差 (5x5)
    Eigen::MatrixXd R_; // 测量噪声协方差 (4x4)
    
    // 雅可比矩阵
    Eigen::MatrixXd Fj_; // 运动模型雅可比 (5x5)
    Eigen::MatrixXd Hj_; // 测量模型雅可比 (4x5)

    // --- 运行时状态 ---
    double last_mahalanobis_dist_;
    bool last_update_valid_;

    // --- 私有方法 ---
    
    // 初始化相关
    void initializeMeasurementNoiseMatrix();
    void initializeCovarianceMatrix(double initial_speed, double initial_yaw);
    
    // 预测相关
    void computeProcessNoise(double dt, double yaw);
    void computeJacobian(double dt, double v, double yaw, double yawd);
    Eigen::VectorXd ctrvProcessModel(const Eigen::VectorXd& x, double dt) const;
    
    // 更新相关
    bool validateMeasurement(const Eigen::VectorXd& meas, 
                           const Eigen::VectorXd& pred,
                           const Eigen::MatrixXd& S,
                           double gating_threshold);
    void performUpdate(const Eigen::VectorXd& z_part, 
                      const Eigen::MatrixXd& H_part, 
                      const Eigen::MatrixXd& R_part,
                      const Eigen::VectorXd& innovation);
    
    // 工具函数
    double normalizeAngle(double angle) const;
    void validateConfiguration() const;
    Eigen::Vector2d computeVelocityFromState() const;
};

#endif // RADAR_PERCEPTION_EXTENDED_KALMAN_FILTER_H