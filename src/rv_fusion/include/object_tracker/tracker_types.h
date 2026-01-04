#ifndef RADAR_PERCEPTION_TRACKER_TYPES_H
#define RADAR_PERCEPTION_TRACKER_TYPES_H

#include <Eigen/Dense>
#include "kalman_filter/ekf.h"
#include <deque>
#include <vector>

namespace rv_fusion {

// 轨迹的生命周期状态
enum TrackState {
    TENTATIVE,  // 疑似目标 (刚出现，还不够稳定)
    CONFIRMED,  // 确认目标 (连续多帧检测到)
    DELETED     // 待删除 (连续多帧丢失)
};

// 目标类别 (预留给 Fusion)
enum ObjectClass {
    UNKNOWN = 0,
    CAR = 1,
    PEDESTRIAN = 2,
    CYCLIST = 3
};

// 核心轨迹结构体
struct Track {
    int id;                 // 唯一 ID
    double timestamp;       // [新增] 最后更新时间 (用于计算 dt)
    
    // --- 计数器 ---
    int age;                // 存活的总帧数
    int coast_cycles;       // 连续丢失帧数
    int hit_streak;         // 连续匹配成功帧数
    
    // --- 状态与属性 ---
    TrackState state;       // 生命周期状态
    ObjectClass class_id;   // [新增] 类别 (目前雷达默认为 UNKNOWN)
    double score;           // [新增] 轨迹质量分数 (可基于连续匹配次数计算)
    
    ExtendedKalmanFilter ekf; // 该轨迹绑定的滤波器

    // --- 可视化与调试 ---
    // 新增] 历史轨迹缓存 (只存位置，用于 Rviz 显示尾迹)
    // 使用 deque 方便从头部删除旧数据
    std::deque<Eigen::Vector2d> history; 
    const size_t max_history_size = 20; // 限制缓存长度

    // 构造函数
    Track(int _id, double _timestamp) 
        : id(_id), timestamp(_timestamp), 
          age(0), coast_cycles(0), hit_streak(0), 
          state(TENTATIVE), class_id(UNKNOWN), score(0.0) {}

    // --- 辅助函数：更新历史记录 ---
    void updateHistory() {
        Eigen::Vector4d x = ekf.getCartesianState();
        history.push_back(Eigen::Vector2d(x(0), x(1)));
        if (history.size() > max_history_size) {
            history.pop_front();
        }
    }
};

} // namespace radar_perception

#endif // RADAR_PERCEPTION_TRACKER_TYPES_H