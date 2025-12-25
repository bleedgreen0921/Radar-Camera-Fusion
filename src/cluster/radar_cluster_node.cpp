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
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h> // 聚类核心库
#include <pcl/common/common.h> // 用于计算 Min/Max

typedef pcl::PointXYZ PointType;

class RadarClusterNode{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_clusters_;

    // 聚类参数
    double cluster_tolerance_; // 判定为同一物体的最大距离
    int min_cluster_size_;     // 最小点数 (雷达点稀疏，设小一点)
    int max_cluster_size_;     // 最大点数

public:
    RadarClusterNode():nh_("~"){
        // 参数配置 (可以在 launch 文件中修改)
        nh_.param("cluster_tolerance", cluster_tolerance_, 1.5); // 雷达点比较散，这个值要比激光雷达(0.2)大得多
        nh_.param("min_cluster_size", min_cluster_size_, 3);     // 至少3个点才算物体
        nh_.param("max_cluster_size", max_cluster_size_, 100);

        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/radar/surround_pointcloud", 10, &RadarClusterNode::cloudCallback, this
        );

        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/detection_boxes", 10);
        pub_clusters_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar/cluster_points", 10);

        ROS_INFO("Radar Clustering Node Started. Tolerance: %.2f", cluster_tolerance_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // 1. 转为 PCL 格式
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // --- 预处理建议 ---
        // 雷达的 Z 轴（高度）通常很不准。为了防止同一辆车的点因为高度差被分成两类，
        // 在聚类前最好把所有点的 Z 轴“拍扁” (Flatten)，或者由算法自动处理。
        // 这里为了演示简单，直接用 3D 聚类，但容差(tolerance)设大一点。

        // 2. 建立 KdTree (加速搜索)
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
        tree->setInputCloud(cloud);

        // 3. 执行欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(cluster_tolerance_); 
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 4. 生成可视化方框
        visualization_msgs::MarkerArray marker_array;

        // 先添加一个 DELETEALL 指令，清空上一帧的所有方框
        visualization_msgs::Marker clear_marker;
        clear_marker.header = msg->header;
        clear_marker.ns = "radar_objects";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        int cluster_id = 0;

        // 遍历每一个聚类簇
        for (const auto& indices : cluster_indices)
        {
            // 提取当前簇的点
            pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
            for (const auto& idx : indices.indices)
                cloud_cluster->push_back((*cloud)[idx]);

            // 计算 AABB (Axis-Aligned Bounding Box)
            PointType min_pt, max_pt;
            pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

            // 创建 Marker
            visualization_msgs::Marker marker;
            marker.header = msg->header; // 继承原始数据的时间和坐标系
            marker.ns = "radar_objects";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::CUBE; // 立方体
            marker.action = visualization_msgs::Marker::ADD;

            // 设置方框中心位置
            marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
            marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
            marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;

            // 设置四元数，防止 Rviz 报错
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;

            // 设置方框尺寸
            marker.scale.x = (max_pt.x - min_pt.x);
            marker.scale.y = (max_pt.y - min_pt.y);
            marker.scale.z = (max_pt.z - min_pt.z);
            
            // 如果尺寸太小（比如只有一个点），给一个最小尺寸，方便观察
            if (marker.scale.x < 0.1) marker.scale.x = 0.5;
            if (marker.scale.y < 0.1) marker.scale.y = 0.5;
            if (marker.scale.z < 0.1) marker.scale.z = 0.5;

            // 设置颜色 (绿色，半透明)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;

            // 设置生命周期 (0.1秒后自动消失，防止残留)
            marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(marker);
        }

        // 5. 发布
        if (!marker_array.markers.empty()) {
            pub_markers_.publish(marker_array);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_cluster_node");
    RadarClusterNode node;
    ros::spin();
    return 0;
}