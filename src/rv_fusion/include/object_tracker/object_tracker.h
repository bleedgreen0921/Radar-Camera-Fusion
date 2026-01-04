#ifndef RADAR_PERCEPTION_OBJECT_TRACKER_H
#define RADAR_PERCEPTION_OBJECT_TRACKER_H

#include <vector>
#include <memory>
#include "object_tracker/tracker_types.h"
#include "object_tracker/association_manager.h"

namespace rv_fusion {

class ObjectTracker {
public:
    ObjectTracker();
    ~ObjectTracker();

    /**
     * @brief 核心业务接口
     * @param detections 当前帧聚类结果 [px, py, vx, vy]
     * @param dt 时间差
     * @return 活跃的轨迹列表 (用于 ROS 发布)
     */
    std::vector<Track> update(const std::vector<Eigen::Vector4d>& detections, double dt, double timestamp);
private:
    // --- 内部逻辑 ---
    void createNewTrack(const Eigen::Vector4d& det, double timestamp);
    void pruneDeadTracks();
    void updateTrackStatus(Track& track);

    // --- 成员变量 ---
    std::vector<Track> tracks_; // 轨迹池
    int next_id_;               // ID 计数器

    // 模块实例
    std::unique_ptr<AssociationManager> associator_;

    // 调参参数
    int max_coast_cycles_;      // 允许最大连续丢失帧数 (比如 5)
    int min_hits_to_confirm_;   // 确认为真目标的帧数 (比如 3)
};

} // namespace radar_perception

#endif // RADAR_PERCEPTION_OBJECT_TRACKER_H