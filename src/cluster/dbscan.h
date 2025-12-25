#ifndef RADAR_CONVERTER_DBSCAN_H
#define RADAR_CONVERTER_DBSCAN_H

#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// 约定：使用 PointXYZI
// - 对于 Radar: Intensity 存储 "速度 (Velocity)"
// - 对于 LiDAR: Intensity 存储 "反射率 (Reflectivity)"
typedef pcl::PointXYZI PointType;

class Dbscan {
public:
    /**
     * @brief 构造函数
     * @param eps_dist 空间搜索半径 (米)
     * @param eps_vel  速度搜索半径 (米/秒)
     * @param min_pts  成为核心点所需的最小邻居数
     * @param use_z    是否使用 Z 轴信息 (Radar选 false, LiDAR选 true)
     * @param use_vel  是否使用速度约束 (Radar选 true, LiDAR通常选 false)
     */
    Dbscan(double eps_dist, double eps_vel, int min_pts, bool use_z, bool use_vel);

    ~Dbscan();

    /**
     * @brief 设置输入点云并构建 KD-Tree
     * 内部会自动根据 use_z 配置决定是否对点云进行 2D 拍扁处理
     */
    void setInputCloud(pcl::PointCloud<PointType>::Ptr cloud);

    /**
     * @brief 执行聚类的主函数
     * @param cluster_indices 返回聚类结果（每个簇包含的点索引列表）
     */
    void extract(std::vector<pcl::PointIndices>& cluster_indices);

private:
    /**
     * @brief 核心：寻找邻居 (融合了空间和速度)
     * @param index 当前点的索引
     * @param neighbors 返回找到的邻居索引列表
     * 根据 use_vel 决定是否启用径向速度参数
     */
    void getNeighbors(int index, std::vector<int>& neighbors);

    // 成员变量
    pcl::search::KdTree<PointType>::Ptr tree_; // 用于加速空间搜索
    pcl::PointCloud<PointType>::Ptr input_cloud_; // 数据指针，原始数据引用
    pcl::PointCloud<PointType>::Ptr search_cloud_; // 内部搜索用数据 (可能已拍扁)
    
    double eps_dist_; // 空间阈值
    double eps_vel_;  // 速度阈值
    int min_pts_;     // 密度阈值
    bool use_z_;   // Z轴开关
    bool use_vel_; // 速度开关
};

#endif // RADAR_CONVERTER_DBSCAN_H