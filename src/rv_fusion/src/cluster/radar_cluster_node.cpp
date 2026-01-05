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
            "/radar/surround_pointcloud", 10, &RadarClusterNode::cloudCallback, this);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/detection_boxes", 10);

        ROS_INFO("Universal Cluster Node Started. Z-Axis: %s, Velocity: %s", 
                 use_z ? "ON" : "OFF", use_vel ? "ON" : "OFF");
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // 1. 转为 PCL 格式
        // [关键] 此时 PCL 会自动查找 vx_comp/vy_comp 并填入 cloud_raw_
        pcl::fromROSMsg(*msg, *cloud_raw_);
        if (cloud_raw_->empty()) return;

        // 2. 直接输入原始点云 (算法内部会处理 Z 轴拍扁逻辑)
        dbscan_->setInputCloud(cloud_raw_);

        // 3. 执行聚类
        std::vector<pcl::PointIndices> cluster_indices;
        dbscan_->extract(cluster_indices);

        // 4. 可视化
        // 将原始点云传入，以便在可视化时读取点的 vx_comp/vy_comp
        publishMarkers(cluster_indices, cloud_raw_, msg->header);

        // --- 5. (预留) 调用 Tracker ---
        // tracker_->update(cluster_indices, cloud_raw_, msg->header.stamp);
    }

    void publishMarkers(const std::vector<pcl::PointIndices>& clusters, 
                        pcl::PointCloud<PointType>::Ptr cloud, 
                        std_msgs::Header header){
        visualization_msgs::MarkerArray marker_array;
        
        // [重要] 1. 清除上一帧的 Markers
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "detected_objects";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // 清除速度箭头 namespace
        visualization_msgs::Marker clear_arrow;
        clear_arrow.header = header;
        clear_arrow.ns = "velocity_arrows";
        clear_arrow.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_arrow);

        int marker_id = 0;

        // 2. 遍历每个簇生成可视化
        for (const auto& indices : clusters) {
            if (indices.indices.empty()) continue;

            // --- A. 计算包围盒 (AABB) 和 平均速度 ---
            PointType min_pt, max_pt;
            min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
            max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

            float sum_vx = 0.0f;
            float sum_vy = 0.0f;
            int point_count = indices.indices.size();

            for (int idx : indices.indices) {
                const auto& pt = cloud->points[idx];
                
                // 更新 AABB
                if (pt.x < min_pt.x) min_pt.x = pt.x;
                if (pt.y < min_pt.y) min_pt.y = pt.y;
                if (pt.z < min_pt.z) min_pt.z = pt.z;
                if (pt.x > max_pt.x) max_pt.x = pt.x;
                if (pt.y > max_pt.y) max_pt.y = pt.y;
                if (pt.z > max_pt.z) max_pt.z = pt.z;

                // 累加速度分量 (这就是我们自定义 PointRadar 的好处！)
                sum_vx += pt.vx_comp;
                sum_vy += pt.vy_comp;
            }

            // 计算中心点和平均速度
            float center_x = (min_pt.x + max_pt.x) / 2.0;
            float center_y = (min_pt.y + max_pt.y) / 2.0;
            float center_z = (min_pt.z + max_pt.z) / 2.0;
            
            float avg_vx = sum_vx / point_count;
            float avg_vy = sum_vy / point_count;
            float speed = std::sqrt(avg_vx*avg_vx + avg_vy*avg_vy);

            // --- B. 生成 Box Marker ---
            visualization_msgs::Marker box_marker;
            box_marker.header = header;
            box_marker.ns = "detected_objects";
            box_marker.id = marker_id++;
            box_marker.type = visualization_msgs::Marker::CUBE;
            box_marker.action = visualization_msgs::Marker::ADD;
            box_marker.lifetime = ros::Duration(0.15); // 稍微长一点避免闪烁

            box_marker.pose.position.x = center_x;
            box_marker.pose.position.y = center_y;
            box_marker.pose.position.z = center_z;

            // 尺寸 (至少给一个最小值，防止扁平物体看不见)
            box_marker.scale.x = std::max(0.5f, max_pt.x - min_pt.x);
            box_marker.scale.y = std::max(0.5f, max_pt.y - min_pt.y);
            box_marker.scale.z = std::max(0.5f, max_pt.z - min_pt.z);

            box_marker.pose.orientation.w = 1.0;
            
            // 颜色生成 (基于 Cluster ID)
            float r, g, b;
            float hue = fmod(marker_id * 0.618034, 1.0);
            hsv2rgb(hue, 0.7, 1.0, r, g, b);
            box_marker.color.r = r;
            box_marker.color.g = g;
            box_marker.color.b = b;
            box_marker.color.a = 0.5;

            marker_array.markers.push_back(box_marker);
            
            // --- C. 生成 Text Marker (显示 ID 和 速度) ---
            visualization_msgs::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "detected_objects";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.lifetime = ros::Duration(0.15);

            text_marker.pose.position.x = center_x;
            text_marker.pose.position.y = center_y;
            text_marker.pose.position.z = center_z + box_marker.scale.z/2.0 + 0.5; // 悬浮在上方
            
            text_marker.scale.z = 0.5; // 字体大小
            text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
            
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "ID:%d\nV:%.1f", (marker_id/3), speed);
            text_marker.text = buffer;
            
            marker_array.markers.push_back(text_marker);

            // --- D. 生成 Velocity Arrow Marker (速度矢量) ---
            // 只有当速度大于一定阈值时才显示箭头
            if (speed > 0.5) {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.header = header;
                arrow_marker.ns = "velocity_arrows";
                arrow_marker.id = marker_id++;
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;
                arrow_marker.lifetime = ros::Duration(0.15);

                arrow_marker.pose.position.x = center_x;
                arrow_marker.pose.position.y = center_y;
                arrow_marker.pose.position.z = center_z;

                // 计算四元数朝向
                double yaw = std::atan2(avg_vy, avg_vx);
                arrow_marker.pose.orientation.z = sin(yaw / 2.0);
                arrow_marker.pose.orientation.w = cos(yaw / 2.0);

                arrow_marker.scale.x = speed; // 长度代表速度大小
                arrow_marker.scale.y = 0.2;   // 箭头宽
                arrow_marker.scale.z = 0.2;   // 箭头高
                
                arrow_marker.color.r = 1.0; // 红色箭头
                arrow_marker.color.a = 0.8;
                
                marker_array.markers.push_back(arrow_marker);
            } else {
                marker_id++; // 保持 ID 计数同步
            }
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