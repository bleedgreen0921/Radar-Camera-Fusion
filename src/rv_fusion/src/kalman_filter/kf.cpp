#include "kalman_filter/kalman_filter.h"

Kalman_Filter::Kalman_Filter(){
    // 1. 初始化矩阵维度
    x_.setZero();
    P_.setIdentity();
    F_.setIdentity();
    Q_.setIdentity();
    H_.setIdentity(); // 因为观测值 z 直接对应 x，所以 H 是单位阵
    R_.setIdentity();

    // 2. 初始化不确定性 P (初始时刻很不确定，设大一点)
    P_ *= 1000.0;

    // 3. 设置测量噪声 R (雷达参数)
    // 假设：雷达位置误差约 0.5m，速度误差约 1.0m/s
    R_(0, 0) = 0.5 * 0.5; // var_px
    R_(1, 1) = 0.5 * 0.5; // var_py
    R_(2, 2) = 1.0 * 1.0; // var_vx
    R_(3, 3) = 1.0 * 1.0; // var_vy
}

Kalman_Filter::~Kalman_Filter(){}

void Kalman_Filter::init(const Eigen::Vector4d& x_in){
    x_ = x_in;
    P_.setIdentity();
    P_(0, 0) = 1.0; 
    P_(1, 1) = 1.0;
    P_(2, 2) = 10.0; // 初始速度可能不太准，方差给大点
    P_(3, 3) = 10.0;
}

void Kalman_Filter::predict(double dt){
    // 1. 更新状态转移矩阵 F (CV模型: p = p + v*dt)
    F_(0,2) = dt;
    F_(1,3) = dt;

    // 2. 预测状态 x' = F * x
    x_ = F_ * x_;

    // 3. 设置过程噪声 Q (表示物体可能会变加速，违背CV模型)
    // 使用离散白噪声模型
    double noise_ax = 5.0; // 假设最大加速度扰动 5m/s^2
    double noise_ay = 5.0;
    
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    Q_.setZero();
    Q_(0, 0) = dt_4 / 4 * noise_ax;
    Q_(0, 2) = dt_3 / 2 * noise_ax;
    Q_(1, 1) = dt_4 / 4 * noise_ay;
    Q_(1, 3) = dt_3 / 2 * noise_ay;
    Q_(2, 0) = dt_3 / 2 * noise_ax;
    Q_(2, 2) = dt_2 * noise_ax;
    Q_(3, 1) = dt_3 / 2 * noise_ay;
    Q_(3, 3) = dt_2 * noise_ay;

    // 4. 预测协方差 P' = F*P*F^T + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void Kalman_Filter::update(const Eigen::Vector4d& z_meas){
    Eigen::Vector4d z_pred = H_ * x_;
    Eigen::Vector4d y = z_meas - z_pred; // 误差 (Innovation)
    
    Eigen::Matrix4d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix4d K = P_ * H_.transpose() * S.inverse(); // 卡尔曼增益

    // 更新状态
    x_ = x_ + (K * y);
    
    // 更新协方差 P = (I - K*H) * P
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P_ = (I - K * H_) * P_;
}