#ifndef RADAR_CONVERTER_DBSCAN_H
#define RADAR_CONVERTER_DBSCAN_H

#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "common/point_types.h"

typedef rv_fusion::PointRadar PointType;

class Dbscan {
public:
    struct Config {
        double eps_dist = 1.0;      // 空间搜索半径
        double eps_vel = 2.0;       // 速度搜索半径
        int min_pts = 3;           // 最小邻居数
        bool use_z = false;         // 使用Z轴信息
        bool use_vel = true;        // 使用速度约束
        bool use_confidence = false;// [新增] 使用置信度约束
        float confidence_thresh = 0.5f; // [新增] 置信度阈值
    };
    
    struct ClusterInfo {
        pcl::PointIndices indices;
        Eigen::Vector4f centroid;   // [新增] 簇中心
        Eigen::Vector3f velocity;   // [新增] 平均速度
        size_t point_count;         // [新增] 点数
    };

    explicit Dbscan(const Config& config);
    ~Dbscan();

    void setInputCloud(pcl::PointCloud<rv_fusion::PointRadar>::Ptr cloud);
    
    // [改进] 返回更丰富的聚类信息
    void extract(std::vector<ClusterInfo>& clusters);
    
    // [新增] 获取统计信息
    struct Statistics {
        size_t total_points;
        size_t clustered_points;
        size_t noise_points;
        size_t cluster_count;
        double processing_time_ms;
    };
    Statistics getStatistics() const { return statistics_; }

private:
    void getNeighbors(int index, std::vector<int>& neighbors);
    void calculateClusterInfo(ClusterInfo& info); // [新增] 计算簇信息
    
    pcl::search::KdTree<rv_fusion::PointRadar>::Ptr tree_;
    pcl::PointCloud<rv_fusion::PointRadar>::Ptr input_cloud_;
    pcl::PointCloud<rv_fusion::PointRadar>::Ptr search_cloud_;
    
    Config config_;
    Statistics statistics_; // [新增] 统计信息
};

#endif