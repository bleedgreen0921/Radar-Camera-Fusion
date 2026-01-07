/*
雷达聚类节点：
 * 1. 订阅 /radar/surround_pointcloud (接收包含 vx/vy 的自定义点云)
 * 2. 使用 DBSCAN 进行聚类 (基于 距离 + 速度差)
 * 3. 发布检测结果可视化 (Box + ID)到 Rviz
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h> // 用于发布方框

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include "cluster/dbscan.h"
#include "object_tracker/object_tracker.h"
#include "common/point_types.h"

typedef rv_fusion::PointRadar PointType;

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
    ros::Publisher pub_markers_; // 发布聚类结果 (红色，调试用)
    ros::Publisher pub_tracks_;  // 发布跟踪结果 (绿色，最终输出)
    std::shared_ptr<Dbscan> dbscan_;
    std::unique_ptr<rv_fusion::ObjectTracker> tracker_; // 跟踪器实例
    pcl::PointCloud<PointType>::Ptr cloud_raw_;
    double last_timestamp_;

public:
    RadarClusterNode():nh_("~"){
        double eps_dist, eps_vel;
        int min_pts;
        nh_.param("eps_dist", eps_dist, 1.5);
        nh_.param("eps_vel", eps_vel, 2.0);
        nh_.param("min_pts", min_pts, 3);

        // 初始化算法
        dbscan_ = std::make_shared<Dbscan>(eps_dist, eps_vel, min_pts, false, true);
        tracker_ = std::make_unique<rv_fusion::ObjectTracker>(); // 初始化 Tracker

        cloud_raw_.reset(new pcl::PointCloud<PointType>);

        // 订阅与发布
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/radar/surround_pointcloud", 10, &RadarClusterNode::cloudCallback, this);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/raw_clusters", 10);
        pub_tracks_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/tracked_objects", 10);
        ROS_INFO("Radar Perception System Started (Cluster + Tracker).");
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // 1. 转为 PCL 格式
        // [关键] 此时 PCL 会自动查找 vx_comp/vy_comp 并填入 cloud_raw_
        pcl::fromROSMsg(*msg, *cloud_raw_);
        if (cloud_raw_->empty()) return;

        double current_time = msg->header.stamp.toSec();
        double dt = (last_timestamp_ == 0) ? 0.1 : (current_time - last_timestamp_);
        last_timestamp_ = current_time;
        
        if(dt > 1.0) dt = 0.1;
        if(dt < 0.001) dt = 0.001;

        // 2. 直接输入原始点云 (算法内部会处理 Z 轴拍扁逻辑)
        dbscan_->setInputCloud(cloud_raw_);

        // 3. 执行聚类
        std::vector<pcl::PointIndices> cluster_indices;
        dbscan_->extract(cluster_indices);

        // --- 2. 提取测量值 (Detections) ---
        std::vector<Eigen::Vector4d> detections;
        for (const auto& indices : cluster_indices) {
            if (indices.indices.empty()) continue;

            double sum_x = 0, sum_y = 0, sum_vx = 0, sum_vy = 0;
            int count = indices.indices.size();

            for (int idx : indices.indices) {
                const auto& pt = cloud_raw_->points[idx];
                sum_x += pt.x;
                sum_y += pt.y;
                sum_vx += pt.vx_comp;
                sum_vy += pt.vy_comp;
            }
            detections.push_back(Eigen::Vector4d(sum_x/count, sum_y/count, sum_vx/count, sum_vy/count));
        }

        // --- 3. EKF 跟踪更新 ---
        std::vector<rv_fusion::Track> tracks = tracker_->update(detections, dt, current_time);

        // --- 4. 可视化发布 ---
        publishTrackMarkers(tracks, msg->header);
    }

    void publishTrackMarkers(const std::vector<rv_fusion::Track>& tracks, std_msgs::Header header) {
        visualization_msgs::MarkerArray marker_array;
        
        // DELETEALL
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "tracks";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (const auto& track : tracks) {
            Eigen::Vector4d state = track.ekf.getCartesianState(); 
            double speed = std::sqrt(state(2)*state(2) + state(3)*state(3));

            // 绿色方框 (Track)
            visualization_msgs::Marker box;
            box.header = header;
            box.ns = "tracks";
            box.id = track.id;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = state(0);
            box.pose.position.y = state(1);
            box.pose.position.z = 0.0;
            box.scale.x = 2.0; 
            box.scale.y = 1.0;
            box.scale.z = 1.0;
            box.color.r = 0.0; box.color.g = 1.0; box.color.b = 0.0; box.color.a = 0.6;
            box.lifetime = ros::Duration(0.15);
            marker_array.markers.push_back(box);

            // ID 文字
            visualization_msgs::Marker text;
            text.header = header;
            text.ns = "tracks_text";
            text.id = track.id;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = state(0);
            text.pose.position.y = state(1);
            text.pose.position.z = 1.5;
            text.scale.z = 0.8;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "ID:%d\nV:%.1f", track.id, speed);
            text.text = buffer;
            text.lifetime = ros::Duration(0.15);
            marker_array.markers.push_back(text);
        }
        pub_tracks_.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "universal_cluster_node");
    RadarClusterNode node;
    ros::spin();
    return 0;
}