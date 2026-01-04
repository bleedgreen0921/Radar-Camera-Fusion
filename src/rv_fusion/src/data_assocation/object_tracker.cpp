#include "object_tracker/object_tracker.h"
#include <iostream>

namespace rv_fusion {

ObjectTracker::ObjectTracker() {
    next_id_ = 0;
    
    // 初始化子模块
    associator_ = std::make_unique<AssociationManager>();

    // 参数配置
    max_coast_cycles_ = 5;  // 丢失 5 帧后删除
    min_hits_to_confirm_ = 3; // 连续匹配 3 帧才认为是真车 (防鬼影)
}

ObjectTracker::~ObjectTracker() {}

std::vector<Track> ObjectTracker::update(const std::vector<Eigen::Vector4d>& detections, double dt, double timestamp) {
    
    // --- 1. 预测阶段 (Predict) ---
    for (auto& track : tracks_) {
        track.ekf.predict(dt);
    }

    // --- 2. 关联阶段 (Associate) ---
    std::vector<std::pair<int, int>> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;

    associator_->associate(tracks_, detections, matches, unmatched_tracks, unmatched_detections);

    // --- 3. 更新阶段 (Update) ---
    
    // 3.1 处理匹配成功的轨迹
    for (const auto& match : matches) {
        int track_idx = match.first;
        int det_idx = match.second;

        Track& track = tracks_[track_idx];
        
        // EKF 更新
        track.ekf.update(detections[det_idx]);

        // [新增] 更新轨迹的时间戳
        track.timestamp = timestamp; 
        // [新增] 更新历史轨迹缓存 (如果你采纳了 history 的话)
        track.updateHistory();

        // 状态计数器更新
        track.hit_streak++;
        track.coast_cycles = 0;
        
        updateTrackStatus(track);
    }

    // 3.2 处理未匹配的轨迹 (丢失)
    for (int track_idx : unmatched_tracks) {
        Track& track = tracks_[track_idx];
        track.coast_cycles++;
        track.hit_streak = 0; // 连续匹配中断
        
        // 如果只是 TENTATIVE (疑似) 且没匹配上，直接删掉，防止噪点
        if (track.state == TENTATIVE) {
            track.state = DELETED; 
        } else if (track.state == CONFIRMED) {
            // 如果是确认目标，允许短暂丢失 (Coast)
            // 这里不需要做 EKF update，保持 predict 的状态即可
        }
    }

    // 3.3 处理未匹配的检测 (新目标)
    for (int det_idx : unmatched_detections) {
        createNewTrack(detections[det_idx], timestamp);
    }

    // --- 4. 剪枝阶段 (Prune) ---
    pruneDeadTracks();

    // --- 5. 输出构建 ---
    // 只输出 Confirmed 的轨迹，或者根据需求输出所有非 Deleted 的
    std::vector<Track> output_tracks;
    for (const auto& track : tracks_) {
        if (track.state == CONFIRMED || (track.state == TENTATIVE && track.hit_streak > 1)) {
            output_tracks.push_back(track);
        }
    }

    return output_tracks;
}

void ObjectTracker::createNewTrack(const Eigen::Vector4d& det, double timestamp) {
    Track new_track(next_id_++, timestamp);
    new_track.ekf.init(det); // 使用 CTRV EKF 初始化
    new_track.updateHistory();
    tracks_.push_back(new_track);
}

void ObjectTracker::updateTrackStatus(Track& track) {
    // 简单的状态机流转
    if (track.state == TENTATIVE && track.hit_streak >= min_hits_to_confirm_) {
        track.state = CONFIRMED;
        std::cout << "[Tracker] Track ID " << track.id << " Confirmed!" << std::endl;
    }
}

void ObjectTracker::pruneDeadTracks() {
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        if (it->state == DELETED || it->coast_cycles > max_coast_cycles_) {
            // std::cout << "[Tracker] Track ID " << it->id << " Deleted." << std::endl;
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

}