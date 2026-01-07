#ifndef RADAR_PERCEPTION_OBJECT_TRACKER_H
#define RADAR_PERCEPTION_OBJECT_TRACKER_H

#include <vector>
#include <memory>
#include "object_tracker/tracker_types.h"
#include "data_association/association_manager.h"

namespace rv_fusion {

    struct TrackerStats {
    long long total_created_ids = 0;   // 总共生成的 ID 数量 (分母：确认率)
    long long confirmed_tracks = 0;    // 成功转为 CONFIRMED 的数量 (分子：确认率 / 分母：存活比)
    long long long_lived_tracks = 0;   // 存活超过阈值的轨迹数量 (分子：存活比)
    
    long long total_frames_accum = 0;  // 所有确认轨迹的累计帧数 (分母：滑行占比)
    long long coasted_frames_accum = 0;// 其中处于滑行状态的帧数 (分子：滑行占比)
};

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
    void printPerformanceMetrics();

    // --- 成员变量 ---
    std::vector<Track> tracks_; // 轨迹池
    int next_id_;               // ID 计数器

    // 模块实例
    std::unique_ptr<AssociationManager> associator_;

    // 调参参数
    int max_coast_cycles_;      // 允许最大连续丢失帧数 (比如 5)
    int min_hits_to_confirm_;   // 确认为真目标的帧数 (比如 3)

    TrackerStats stats_;
};

} // namespace radar_perception

#endif