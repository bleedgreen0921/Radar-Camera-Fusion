#ifndef RADAR_PERCEPTION_KALMAN_FILTER_H
#define RADAR_PERCEPTION_KALMAN_FILTER_H

#include <Eigen/Dense>

class Kalman_Filter{
private:
    // --- 核心矩阵 ---
    Eigen::Vector4d x_; // 状态向量
    Eigen::Matrix4d P_; // 状态协方差矩阵 (不确定性)
    Eigen::Matrix4d F_; // 状态转移矩阵 (物理模型)
    Eigen::Matrix4d Q_; // 过程噪声协方差 (外界干扰)
    Eigen::Matrix4d H_; // 观测矩阵 (状态 -> 观测的映射)
    Eigen::Matrix4d R_; // 测量噪声协方差 (雷达误差)

public:
    Kalman_Filter();
    ~Kalman_Filter();

    /**
     * @brief 初始化状态
     * @param x_in 初始状态向量 (通常来自第一次检测)
     */
    void init(const Eigen::Vector4d& x_in);

    /**
     * @brief 预测阶段 (Prediction)
     * 根据时间差 dt 和 运动模型推算下一步状态
     * @param dt 两帧之间的时间间隔 (秒)
     */
    void predict(double dt);

    /**
     * @brief 更新阶段 (Update)
     * 结合雷达观测值修正预测值
     * @param z_meas 雷达测量值 [x, y, vx, vy]
     */
    void update(const Eigen::Vector4d& z_meas);

    // 获取当前最优估计状态
    Eigen::Vector4d getState() const { return x_; }
    
    // 获取当前协方差 (用于计算马氏距离)
    Eigen::Matrix4d getCovariance() const { return P_; }
};

#endif