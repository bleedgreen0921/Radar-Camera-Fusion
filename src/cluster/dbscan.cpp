#include "radar_converter/dbscan.h"

// 构造函数：初始化参数和 KD-Tree 指针
Dbscan::Dbscan(double eps_dist, double eps_vel, int min_pts, bool use_z, bool use_vel)
    : eps_dist_(eps_dist), eps_vel_(eps_vel), min_pts_(min_pts), use_z_(use_z), use_vel_(use_vel) {
    
    tree_.reset(new pcl::search::KdTree<PointType>());
    search_cloud_.reset(new pcl::PointCloud<PointType>());
}

Dbscan::~Dbscan() {}

// 1. 数据预处理 (根据配置决定是否拍扁 Z 轴)
void Dbscan::setInputCloud(pcl::PointCloud<PointType>::Ptr cloud) {
    input_cloud_ = cloud;
    if (input_cloud_->empty()) return;

    // 清空内部搜索云
    search_cloud_->clear();
    search_cloud_->points.reserve(input_cloud_->points.size());

    for(const auto& pt : input_cloud_->points){
        PointType pt_internal = pt;

        if(!use_z_){
            pt_internal.z = 0.0f;
        }
        // 如果 use_z_ 为 true (LiDAR模式)，保留原始 Z，KD-Tree 将进行 3D 搜索
        search_cloud_->push_back(pt_internal);
    }
    // 基于处理后的点云重建 KD-Tree
    tree_->setInputCloud(search_cloud_);
}

// 2. 核心聚类逻辑 (DBSCAN 主流程)
void Dbscan::extract(std::vector<pcl::PointIndices>& cluster_indices) {
    cluster_indices.clear();
    if (!input_cloud_ || input_cloud_->empty()) return;

    int n_points = input_cloud_->points.size();
    
    // labels 用于标记每个点的状态：
    // 0: 未处理 (Unvisited)
    // -1: 噪声 (Noise)
    // >0: 簇 ID (Cluster ID)
    std::vector<int> labels(n_points, 0); 
    int cluster_id = 0;

    // --- 外层循环：遍历每一个点 ---
    for (int i = 0; i < n_points; ++i) {
        if (labels[i] != 0) continue; // 如果已经归类或者是已知的噪声，跳过

        // 寻找当前点的所有“合格”邻居
        std::vector<int> neighbors;
        getNeighbors(i, neighbors); 

        // 密度判断：如果邻居太少，标记为噪声
        if (neighbors.size() < min_pts_) {
            labels[i] = -1; 
            continue;
        }

        // --- 发现核心点，开始建立新簇 ---
        cluster_id++;
        labels[i] = cluster_id;

        pcl::PointIndices current_cluster;
        current_cluster.indices.push_back(i);

        // --- 内层循环：区域生长 (BFS) ---
        // 注意：neighbors 列表会在循环中动态增长
        for (size_t k = 0; k < neighbors.size(); ++k) {
            int neighbor_idx = neighbors[k];

            // 情况 A: 之前被标记为噪声的点
            // (说明它虽不是核心点，但在当前核心点的邻域内 -> 它是边缘点)
            if (labels[neighbor_idx] == -1) {
                labels[neighbor_idx] = cluster_id; // 归入当前簇
                current_cluster.indices.push_back(neighbor_idx);
            }

            // 情况 B: 已经处理过的点，跳过
            if (labels[neighbor_idx] != 0) continue;

            // 情况 C: 全新的点
            labels[neighbor_idx] = cluster_id; // 归入当前簇
            current_cluster.indices.push_back(neighbor_idx);

            // 检查这个新点是否也是核心点？
            std::vector<int> sub_neighbors;
            getNeighbors(neighbor_idx, sub_neighbors); // 递归式搜索

            // 如果它也是核心点，把它发现的邻居加入大部队，继续向外扩
            if (sub_neighbors.size() >= min_pts_) {
                neighbors.insert(neighbors.end(), sub_neighbors.begin(), sub_neighbors.end());
            }
        }
        
        // 一个簇生长完毕，保存结果
        cluster_indices.push_back(current_cluster);
    }
}

// 3. 带速度维度的邻域搜索 (这是你方案 3 的灵魂所在)
void Dbscan::getNeighbors(int index, std::vector<int>& neighbors) {
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    
    PointType searchPoint = search_cloud_->points[index];

    // [第一步] 空间粗筛：利用 KD-Tree 快速找出空间距离 < eps_dist 的点
    // 如果 use_z_=false，这里算的就是 2D 欧氏距离
    // 如果 use_z_=true， 这里算的就是 3D 欧氏距离
    tree_->radiusSearch(searchPoint, eps_dist_, k_indices, k_sqr_distances);

    neighbors.clear();
    neighbors.reserve(k_indices.size());

    // [第二步] 速度/反射率精筛：遍历空间邻居，计算加权距离
    for (size_t i = 0; i < k_indices.size(); ++i) {
        int idx = k_indices[i];
        
        // 计算空间距离平方 (已经由 KD-Tree 算出)
        float dist_sqr = k_sqr_distances[i]; 

        if(use_vel_){
            // 假设 intensity 存储速度，进行加权距离计算
            float vel_diff = std::abs(search_cloud_->points[idx].intensity - searchPoint.intensity);

            // 归一化椭球距离公式
            float normalized_dist = (dist_sqr / (eps_dist_ * eps_dist_)) + 
                                    (vel_diff * vel_diff) / (eps_vel_ * eps_vel_);

            if (normalized_dist <= 1.0f) {
                neighbors.push_back(idx);
            }
        }
        else{
            // --- LiDAR 模式 (纯空间聚类) ---
            // 直接接受 KD-Tree 的结果 (因为 KD-Tree 已经保证了 dist < eps_dist)
            // 此时忽略 intensity (反射率)，防止把同一物体不同反射率的部分切开
            neighbors.push_back(idx);
        }
    }
}