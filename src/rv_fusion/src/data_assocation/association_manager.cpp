/*
 * File: src/object_tracker/association_manager.cpp
 * 功能: 数据关联实现 (马氏距离权重 + 匈牙利算法全局匹配)
 */

#include "data_association/association_manager.h"
#include "utils/hungarian.h"
#include <limits>
#include <cmath>
#include <iostream>

namespace rv_fusion {

AssociationManager::AssociationManager() {
    // 马氏距离遵循卡方分布 (Chi-Square Distribution)
    // 4个自由度 (x, y, vx, vy) 下，99% 的置信区间阈值大约是 13.27
    // 95% 的置信区间是 9.48。
    // 意味着：如果一个检测点落在预测椭圆的 9X% 概率范围之外，我们认为它不是同一个物体。
    dist_threshold_ = 30.0;
}

AssociationManager::~AssociationManager() {}

// 计算加权欧氏距离 (Weighted Euclidean Distance) - 近似马氏距离
// D = sqrt( (z - Hx)^T * W * (z - Hx) )
double AssociationManager::calculateDistance(const Eigen::Vector4d& track_state,
                                             const Eigen::Vector4d& det_meas,
                                             const Eigen::MatrixXd& P_matrix){
    // 1. 计算残差 (Innovation) y = z - Hx
    // track_state 已经是笛卡尔坐标系下的 (px, py, vx, vy)
    Eigen::Vector4d y = det_meas - track_state;

    // 2. 计算权重
    // 标准马氏距离需要计算 S = HPH' + R 的逆矩阵 S^-1。
    // 在工程初期，直接求逆可能会因为矩阵奇异导致崩溃。
    // 这里采用 "对角线加权法"，利用 P 矩阵对角线元素（方差）的倒数作为权重。
    // 物理意义：方差越大（越不确定），权重越小（容忍度越高）。
    
    // P 矩阵是 5x5 (px, py, v, yaw, yaw_rate) [CTRV模型]
    // 我们需要取对应分量的方差。
    // 注意：EKF 状态是 v 和 yaw，而观测是 vx, vy。
    // 这里为了简化，我们假设位置的不确定性直接取 P(0,0) 和 P(1,1)
    // 速度的不确定性简单取 P(2,2) (速度模长的方差)
    
    double var_x = P_matrix(0, 0);
    double var_y = P_matrix(1, 1);
    double var_v = P_matrix(2, 2); // 速度模长的方差

    // 避免除以 0，加一个小量 epsilon
    // 核心思想：方差越大，权重越小 -> 容忍度越高
    double w_x = 1.0 / (var_x + 0.5); 
    double w_y = 1.0 / (var_y + 0.5);
    // vx 和 vy 共用速度模长的方差权重 (近似处理)
    double w_v = 1.0 / (var_v + 1.0); 

    // 3. 计算加权平方和
    double dist_sq = w_x * y(0)*y(0) + 
                     w_y * y(1)*y(1) + 
                     w_v * y(2)*y(2) + 
                     w_v * y(3)*y(3);

    return std::sqrt(dist_sq);
    }


void AssociationManager::associate(
        // 输入
        const std::vector<Track>& tracks, // 已经完成 EKF 预测的轨迹集合
        const std::vector<Eigen::Vector4d>& detections, // 当前帧传感器检测 

        // 输出
        std::vector<std::pair<int, int>>& matches, // 通过关联验证的 Track–Detection 对
        std::vector<int>& unmatched_tracks, // 交给后续生命周期管理
        std::vector<int>& unmatched_detections) {
    // 清空输出
    matches.clear();
    unmatched_tracks.clear();
    unmatched_detections.clear();
    
    // 边界条件处理（防止无意义计算）
    if (tracks.empty()) {
        for (size_t i = 0; i < detections.size(); ++i) unmatched_detections.push_back(i);
        return;
    }
    if (detections.empty()) {
        for (size_t i = 0; i < tracks.size(); ++i) unmatched_tracks.push_back(i);
        return;
    }

    size_t n_tracks = tracks.size();
    size_t n_dets = detections.size();

    // ==========================================
    // 1. 构建代价矩阵 (Cost Matrix)
    // ==========================================
    // HungarianAlg 需要扁平化的 vector (Row-Major)
    distMatrix_t cost_matrix(n_tracks * n_dets);

    // [新增] 调试计数器
    int debug_print_count = 0;

    for (size_t i = 0; i < n_tracks; ++i) {
        // 基于 EKF 预测状态准备关联量（预测一致性）
        Eigen::Vector4d track_pred = tracks[i].ekf.getCartesianState();
        const Eigen::MatrixXd& P = tracks[i].ekf.getCovariance();

        for (size_t j = 0; j < n_dets; ++j) {
            // 代价度量设计（不确定性感知距离）
            double dist = calculateDistance(track_pred, detections[j], P);
            
            // 填入扁平化数组: index = row * ncols + col
            // 构建全局代价矩阵（问题建模）
            cost_matrix[i * n_dets + j] = dist;

            // [新增] 只有当前几帧或特定ID时打印，防止刷屏
            // 打印 Track 0 和 Detection 0 之间的距离，看看离谱不离谱
            if (i == 0 && j == 0 && debug_print_count < 10) {
                std::cout << "[DEBUG] Track-Det Dist: " << dist 
                          << " | Pred: " << track_pred(0) << "," << track_pred(1) 
                          << " | Meas: " << detections[j](0) << "," << detections[j](1) 
                          << " | P(0,0): " << P(0,0) << std::endl;
                debug_print_count++;
            }
        }
    }

    // ==========================================
    // 2. 运行匈牙利算法 (Global Optimization)
    // ==========================================
    AssignmentProblemSolver solver;
    assignments_t assignment; // assignment[i] = j (Track i -> Det j)
    
    // 调用 Solve
    // 参数: cost_matrix, rows, cols, output_assignment, method
    solver.Solve(cost_matrix, n_tracks, n_dets, assignment, AssignmentProblemSolver::optimal);

    // ==========================================
    // 3. 解析结果并应用门限 (Gating)
    // ==========================================
    std::vector<bool> det_matched_flags(n_dets, false);

    for (size_t i = 0; i < n_tracks; ++i) {
        int assigned_det_idx = assignment[i];

        if (assigned_det_idx != -1) {
            // 统计门控（物理合理性验证）
            // 从代价矩阵中取回刚才算好的距离
            double cost = cost_matrix[i * n_dets + assigned_det_idx];

            if (cost < dist_threshold_) {
                matches.push_back({(int)i, assigned_det_idx});
                det_matched_flags[assigned_det_idx] = true;
            } else {
                // 距离过大，即使匈牙利匹配上了，也认为是错误关联
                unmatched_tracks.push_back(i); 
            }
        } else {
            // 匈牙利算法认为该 Track 无匹配 (当 Tracks > Dets 时)
            unmatched_tracks.push_back(i); 
        }
    }

    // 4. 收集未匹配的检测
    for (size_t j = 0; j < n_dets; ++j) {
        // 未匹配项整理（结果输出）
        if (!det_matched_flags[j]) {
            unmatched_detections.push_back(j);
        }
    }
}

} // namespace rv_fusion