#ifndef RADAR_PERCEPTION_ASSOCIATION_MANAGER_H
#define RADAR_PERCEPTION_ASSOCIATION_MANAGER_H

#include <vector>
#include <Eigen/Dense>
#include "object_tracker/tracker_types.h"

namespace rv_fusion {

class AssociationManager {
public:
    AssociationManager();
    ~AssociationManager();

    /**
     * @brief 执行数据关联
     * @param tracks 预测后的轨迹列表
     * @param detections 当前帧的观测列表 [px, py, vx, vy]
     * @param matches [输出] 匹配成功的索引对 <track_idx, det_idx>
     * @param unmatched_tracks [输出] 未匹配的轨迹索引
     * @param unmatched_detections [输出] 未匹配的观测索引
     */
    void associate(const std::vector<Track>& tracks,
                   const std::vector<Eigen::Vector4d>& detections,
                   std::vector<std::pair<int, int>>& matches,
                   std::vector<int>& unmatched_tracks,
                   std::vector<int>& unmatched_detections);

private:
    // 计算两个向量的欧氏距离 (仅位置 + 速度权重)
    double calculateDistance(const Eigen::Vector4d& track_state, 
                             const Eigen::Vector4d& det_meas);

    // 关联阈值 (马氏距离或欧氏距离阈值)
    double dist_threshold_; 
};

} // namespace radar_perception

#endif // RADAR_PERCEPTION_ASSOCIATION_MANAGER_H