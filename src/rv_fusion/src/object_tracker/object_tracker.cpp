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

    // --- [新增] 指标3统计：滑行占比 (Coasting Ratio) ---
    // 每一帧都遍历一次所有 CONFIRMED 轨迹，累加状态
    for (const auto& track : tracks_) {
        if (track.state == CONFIRMED) {
            stats_.total_frames_accum++;
            // 如果 coast_cycles > 0，说明这一帧没有匹配到观测，完全靠预测（滑行）
            if (track.coast_cycles > 0) {
                stats_.coasted_frames_accum++;
            }
        }
    }

    // --- 4. 剪枝阶段 (Prune) ---
    pruneDeadTracks();

    // --- [新增] 定时打印报告 (每100帧，约5-10秒) ---
    static int print_timer = 0;
    if (++print_timer % 100 == 0) {
        printPerformanceMetrics();
    }

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
    // [新增] 指标1分母：总创建 ID 数
    stats_.total_created_ids++;
}

void ObjectTracker::updateTrackStatus(Track& track) {
    // [新增] 指标1分子 / 指标2分母：确认轨迹数
        // 只有当状态真正发生改变的那一刻才计数
    if (track.state == TENTATIVE && track.hit_streak >= min_hits_to_confirm_) {
        if (track.state != CONFIRMED) {
            stats_.confirmed_tracks++;
        }
        track.state = CONFIRMED;
        // std::cout << "[Tracker] Track ID " << track.id << " Confirmed!" << std::endl;
    }
}

void ObjectTracker::pruneDeadTracks() {
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        if (it->state == DELETED || it->coast_cycles > max_coast_cycles_) {
            // [新增] 指标2分子：存活比统计
            // 只有当一个 CONFIRMED 轨迹结束生命周期时，检查它活了多久
            // 假设 20Hz 运行，60帧大约是 3秒
            if (it->state == CONFIRMED && it->age > 60) {
                stats_.long_lived_tracks++;
            }
            // std::cout << "[Tracker] Track ID " << it->id << " Deleted." << std::endl;
            it = tracks_.erase(it);
        } else {
            it->age++; 
            ++it;
        }
    }
}

// [新增] 打印函数实现
void ObjectTracker::printPerformanceMetrics() {
    double confirm_rate = 0.0;
    double survival_ratio = 0.0;
    double coasting_ratio = 0.0;

    // 1. 确认率 (抗噪能力)
    if (stats_.total_created_ids > 0) {
        confirm_rate = 100.0 * (double)stats_.confirmed_tracks / stats_.total_created_ids;
    }

    // 2. 存活比 (稳定性)
    // 包含两部分：已经结束的长寿命轨迹 + 当前还活着的长寿命轨迹
    // 这样数据会更实时，不会因为最好的轨迹还没死而导致数据偏低
    long long active_long_lived = 0;
    for(const auto& t : tracks_) {
        if(t.state == CONFIRMED && t.age > 60) active_long_lived++;
    }
    long long total_long_lived = stats_.long_lived_tracks + active_long_lived;

    if (stats_.confirmed_tracks > 0) {
        survival_ratio = 100.0 * (double)total_long_lived / stats_.confirmed_tracks;
    }

    // 3. 滑行占比 (预测质量)
    if (stats_.total_frames_accum > 0) {
        coasting_ratio = 100.0 * (double)stats_.coasted_frames_accum / stats_.total_frames_accum;
    }

    // 打印表格
    printf("\n=== [Radar Tracker Performance Report] ===\n");
    printf("| Metric                 | Value   | Target  | Diagnosis                     |\n");
    printf("|------------------------|---------|---------|-------------------------------|\n");
    printf("| 1. Confirmation Rate   | %5.1f%%  | >20%%    | <10%%: Too much clutter/noise |\n", confirm_rate);
    printf("| 2. Survival Ratio (>3s)| %5.1f%%  | >50%%    | <30%%: Severe ID switching    |\n", survival_ratio);
    printf("| 3. Coasting Ratio      | %5.1f%%  | <15%%    | >30%%: Tracking unstable      |\n", coasting_ratio);
    printf("|------------------------|---------|---------|-------------------------------|\n");
    printf("  Total IDs: %lld | Confirmed: %lld | Active Now: %lu\n\n", 
           stats_.total_created_ids, stats_.confirmed_tracks, tracks_.size());
}

}