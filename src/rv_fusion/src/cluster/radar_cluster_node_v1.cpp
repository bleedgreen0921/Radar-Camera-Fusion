/*
雷达聚类节点：
 * 1. 订阅 /radar/surround_pointcloud
 * 2. 进行欧式聚类 (Euclidean Clustering)
 * 3. 发布 Bounding Boxes (MarkerArray) 到 Rviz
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h> // 用于发布方框

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

#include "cluster/dbscan.h"
#include "object_tracker/object_tracker.h"

typedef pcl::PointXYZI PointType;

// 辅助函数：将 HSV 转换为 RGB，用于生成不同颜色的方框
void hsv2rgb(float h, float s, float v, float& r, float& g, float& b) {
    int i = int(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
}

class RadarClusterNode{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_markers_;
    std::shared_ptr<Dbscan> dbscan_;
    pcl::PointCloud<PointType>::Ptr cloud_raw_;

public:
    RadarClusterNode():nh_("~"){
        double eps_dist, eps_vel;
        int min_pts;
        bool use_z, use_vel;

        // 参数配置 (可以在 launch 文件中修改)
        nh_.param("eps_dist", eps_dist, 1.5);
        nh_.param("eps_vel", eps_vel, 2.0);
        nh_.param("min_pts", min_pts, 3);
        nh_.param("use_z_axis", use_z, false);        // 默认忽略高度
        nh_.param("use_velocity", use_vel, true);     // 默认使用速度聚类

        dbscan_ = std::make_shared<Dbscan>(eps_dist, eps_vel, min_pts, use_z, use_vel);
        cloud_raw_.reset(new pcl::PointCloud<PointType>);

        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/radar/surround_pointcloud", 10, &RadarClusterNode::cloudCallback, this
        );
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/detection_boxes", 10);

        ROS_INFO("Universal Cluster Node Started. Z-Axis: %s, Velocity: %s", 
                 use_z ? "ON" : "OFF", use_vel ? "ON" : "OFF");
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // 1. 转为 PCL 格式
        pcl::fromROSMsg(*msg, *cloud_raw_);
        if (cloud_raw_->empty()) return;

        // 1. 直接输入原始点云 (算法内部会处理 Z 轴拍扁逻辑)
        dbscan_->setInputCloud(cloud_raw_);

        // 2. 执行聚类
        std::vector<pcl::PointIndices> clusters;
        dbscan_->extract(clusters);

        // 3. 可视化
        publishMarkers(clusters, cloud_raw_, msg->header);
    }

    void publishMarkers(const std::vector<pcl::PointIndices>& indices, 
                        pcl::PointCloud<PointType>::Ptr cloud, 
                        std_msgs::Header header){
        visualization_msgs::MarkerArray marker_array;
        
        // ---------------------------------------------------------
        // [修复点] 1. 清空上一帧 (DELETEALL)
        // ---------------------------------------------------------
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header; // 最好带上 header
        // 【关键】必须与下面生成方框时的 ns 完全一致！
        clear_marker.ns = "detected_objects"; 
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        int id = 0;
        int cluster_count = indices.size(); // 获取总簇数
        for (const auto& cluster : indices) {
            PointType min_pt, max_pt;
            pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>);
            for (int idx : cluster.indices) temp->push_back(cloud->points[idx]);
            pcl::getMinMax3D(*temp, min_pt, max_pt);

            visualization_msgs::Marker marker;
            marker.header = header;
            // 【关键】这里的 ns 必须是 "detected_objects"
            marker.ns = "detected_objects";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
            marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
            marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
            
            marker.scale.x = std::max(0.5f, max_pt.x - min_pt.x);
            marker.scale.y = std::max(0.5f, max_pt.y - min_pt.y);
            marker.scale.z = std::max(0.5f, max_pt.z - min_pt.z);
            
            marker.pose.orientation.w = 1.0;
            
            // 【修改重点】根据簇 ID 生成不同的颜色
            float r, g, b;
            // 核心思路：将 360度的色相环平分给每个簇
            // 加上 id*0.618 是为了让相邻 ID 的颜色差异更大（黄金分割法）
            float hue = fmod(id * 0.618034, 1.0); 
            float saturation = 0.8; // 饱和度高一点，颜色鲜艳
            float value = 1.0;      // 亮度最高

            hsv2rgb(hue, saturation, value, r, g, b);

            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.5; // 半透明

            // marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(marker);
            id++;
        }
        pub_markers_.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "universal_cluster_node");
    RadarClusterNode node;
    ros::spin();
    return 0;
}