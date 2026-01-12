#include "cluster/dbscan.h"
#include <cmath>

// =========================================================================
// [新增] 关键修复：引入 PCL 模板实现文件 (.hpp)
// 必须包含这些文件，编译器才能为自定义点类型生成 KdTree 代码
// =========================================================================
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>

// 构造函数：初始化参数和 KD-Tree 指针
Dbscan::Dbscan(const Config& config) : config_(config) {
    tree_.reset(new pcl::search::KdTree<rv_fusion::PointRadar>());
    search_cloud_.reset(new pcl::PointCloud<rv_fusion::PointRadar>());
    statistics_ = {0, 0, 0, 0, 0.0};
}

Dbscan::~Dbscan() {}

// 1. 数据预处理 (根据配置决定是否拍扁 Z 轴)
void Dbscan::setInputCloud(pcl::PointCloud<rv_fusion::PointRadar>::Ptr cloud) {
    input_cloud_ = cloud;
    if (input_cloud_->empty()) return;

    // 清空内部搜索云
    search_cloud_->clear();
    search_cloud_->points.reserve(input_cloud_->points.size());

    for(const auto& pt : *input_cloud_) {
        auto pt_processed = pt;
        if(!config_.use_z) {
            pt_processed.z = 0.0f;
        }
        // [新增] 置信度过滤
        if(config_.use_confidence && pt.confidence < config_.confidence_thresh) {
            continue;
        }
        search_cloud_->push_back(pt_processed);
    }
    // 基于处理后的点云重建 KD-Tree
    tree_->setInputCloud(search_cloud_);
}

// 2. 核心聚类逻辑 (DBSCAN 主流程)
void Dbscan::extract(std::vector<ClusterInfo>& clusters) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    clusters.clear();
    if (!search_cloud_ || search_cloud_->empty()) return;

    const size_t n_points = search_cloud_->size();
    std::vector<int> labels(n_points, 0);
    int cluster_id = 0;

    // [改进] 使用BFS队列替代动态vector，提高性能
    for (size_t i = 0; i < n_points; ++i) {
        if (labels[i] != 0) continue;

        std::vector<int> neighbors;
        getNeighbors(static_cast<int>(i), neighbors);

        if (neighbors.size() < config_.min_pts) {
            labels[i] = -1;
            statistics_.noise_points++;
            continue;
        }

        cluster_id++;
        labels[i] = cluster_id;

        ClusterInfo cluster;
        cluster.indices.indices.push_back(static_cast<int>(i));

        // [改进] 使用队列进行BFS，避免动态插入的性能问题
        std::queue<int> expand_queue;
        std::unordered_set<int> visited;
        
        for (int neighbor : neighbors) {
            expand_queue.push(neighbor);
            visited.insert(neighbor);
        }

        while (!expand_queue.empty()) {
            int current_idx = expand_queue.front();
            expand_queue.pop();

            if (labels[current_idx] == -1) {
                labels[current_idx] = cluster_id;
                cluster.indices.indices.push_back(current_idx);
            }

            if (labels[current_idx] != 0) continue;

            labels[current_idx] = cluster_id;
            cluster.indices.indices.push_back(current_idx);

            std::vector<int> current_neighbors;
            getNeighbors(current_idx, current_neighbors);

            if (current_neighbors.size() >= config_.min_pts) {
                for (int neighbor : current_neighbors) {
                    if (visited.find(neighbor) == visited.end()) {
                        expand_queue.push(neighbor);
                        visited.insert(neighbor);
                    }
                }
            }
        }

        // [新增] 计算簇的统计信息
        calculateClusterInfo(cluster);
        clusters.push_back(cluster);
        statistics_.cluster_count++;
    }

    statistics_.total_points = n_points;
    statistics_.clustered_points = n_points - statistics_.noise_points;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    statistics_.processing_time_ms = 
        std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

// 3. 带速度维度的邻域搜索
void Dbscan::getNeighbors(int index, std::vector<int>& neighbors) {
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    
    const auto& search_point = search_cloud_->points[index];
    tree_->radiusSearch(search_point, config_.eps_dist, k_indices, k_sqr_distances);

    neighbors.clear();
    neighbors.reserve(k_indices.size());

    for (size_t i = 0; i < k_indices.size(); ++i) {
        int idx = k_indices[i];
        float dist_sqr = k_sqr_distances[i];

        if(config_.use_vel) {
            const auto& target_point = search_cloud_->points[idx];
            float vel_diff = std::abs(target_point.velocity - search_point.velocity);
            
            // [改进] 更稳健的距离计算，避免除零
            float eps_dist_sqr = config_.eps_dist * config_.eps_dist;
            float eps_vel_sqr = config_.eps_vel * config_.eps_vel;
            
            float normalized_dist = (dist_sqr / eps_dist_sqr) + 
                                   (vel_diff * vel_diff) / eps_vel_sqr;

            if (normalized_dist <= 1.0f) {
                neighbors.push_back(idx);
            }
        } else {
            neighbors.push_back(idx);
        }
    }
}

// [新增] 计算簇的详细信息
void Dbscan::calculateClusterInfo(ClusterInfo& info) {
    info.point_count = info.indices.indices.size();
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    Eigen::Vector3f avg_velocity = Eigen::Vector3f::Zero();

    for (int idx : info.indices.indices) {
        const auto& pt = search_cloud_->points[idx];
        centroid += Eigen::Vector4f(pt.x, pt.y, pt.z, 1.0f);
        avg_velocity += Eigen::Vector3f(pt.vx_comp, pt.vy_comp, 0.0f);
    }

    if (info.point_count > 0) {
        centroid /= static_cast<float>(info.point_count);
        avg_velocity /= static_cast<float>(info.point_count);
    }

    info.centroid = centroid;
    info.velocity = avg_velocity;
}