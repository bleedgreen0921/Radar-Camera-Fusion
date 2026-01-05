#include "data_association/association_manager.h"
#include <limits>
#include <cmath>

namespace rv_fusion {

AssociationManager::AssociationManager() {
    dist_threshold_ = 3.0; // 3米内的才认为是同一个目标
}

AssociationManager::~AssociationManager() {}

double AssociationManager::calculateDistance(const Eigen::Vector4d& track_state, 
                                             const Eigen::Vector4d& det_meas) {
    // 简单计算位置欧氏距离 (忽略速度差异，或者你可以给速度加权重)
    double dx = track_state(0) - det_meas(0);
    double dy = track_state(1) - det_meas(1);
    return std::sqrt(dx*dx + dy*dy);
}

void AssociationManager::associate(const std::vector<Track>& tracks,
                                   const std::vector<Eigen::Vector4d>& detections,
                                   std::vector<std::pair<int, int>>& matches,
                                   std::vector<int>& unmatched_tracks,
                                   std::vector<int>& unmatched_detections) {
    matches.clear();
    unmatched_tracks.clear();
    unmatched_detections.clear();

    if (tracks.empty()) {
        for (size_t i = 0; i < detections.size(); ++i) unmatched_detections.push_back(i);
        return;
    }

    std::vector<bool> det_matched(detections.size(), false);
    std::vector<bool> track_matched(tracks.size(), false);

    // 简单的双重循环最近邻匹配 (实际工程建议用匈牙利算法)
    for (size_t i = 0; i < tracks.size(); ++i) {
        double min_dist = std::numeric_limits<double>::max();
        int best_det_idx = -1;

        // 获取 EKF 预测状态 (只取 x, y, vx, vy)
        Eigen::Vector4d track_pred = tracks[i].ekf.getCartesianState();

        for (size_t j = 0; j < detections.size(); ++j) {
            if (det_matched[j]) continue; // 已经被匹配过的跳过

            double dist = calculateDistance(track_pred, detections[j]);
            if (dist < dist_threshold_ && dist < min_dist) {
                min_dist = dist;
                best_det_idx = j;
            }
        }

        if (best_det_idx != -1) {
            matches.push_back({i, best_det_idx});
            det_matched[best_det_idx] = true;
            track_matched[i] = true;
        }
    }

    // 收集未匹配项
    for (size_t i = 0; i < tracks.size(); ++i) {
        if (!track_matched[i]) unmatched_tracks.push_back(i);
    }
    for (size_t j = 0; j < detections.size(); ++j) {
        if (!det_matched[j]) unmatched_detections.push_back(j);
    }
}

} // namespace rv_fusion