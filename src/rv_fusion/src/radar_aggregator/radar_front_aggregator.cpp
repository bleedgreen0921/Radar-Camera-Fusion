/*
雷达数据聚合节点：
 * 1. 订阅 nuscenes2bag::RadarObjects
 * 2. 使用 TF2 + Eigen 将点变换到车身坐标系
 * 3. 使用双缓冲 (Double Buffering) 无锁/低延迟聚合
 * 4. 修正输出时间戳
 * 5. 将位置 (P) 和 速度向量 (V) 变换到 base_link
 * 6. 发布包含 vx_comp, vy_comp 的自定义 PCL 点云
*/

#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <nuscenes2bag/RadarObjects.h> // 要订阅的、来自 nuscenes2bag 包的自定义 RadarObjects 消息

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

// TF2 核心头文件
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>             // 用于将 TF2 变换转为 Eigen 矩阵
#include <geometry_msgs/TransformStamped.h>

// 线程与同步
#include <mutex>
#include "common/point_types.h"

typedef rv_fusion::PointRadar PointType;

class RadarAggregator {
private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subs_;
    ros::Publisher pub_front_cloud_;
    ros::Timer timer_;

    // 
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 
    pcl::PointCloud<PointType>::Ptr accumulated_cloud_;
    ros::Time latest_stamp_;
    std::mutex mutex_;

    const float MIN_SPEED = 0.5f;  
    const float MIN_Z = -0.5f;
    const float MAX_Z = 2.0f;

public:
    RadarAggregator() : nh_("~"), tf_listener_(tf_buffer_) {
    pub_front_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar/front_pointcloud", 10);
    std::vector<std::string> radar_objects = {
        "/radar_front", "/radar_front_left", "/radar_front_right"
    };

    // 预分配内存
    accumulated_cloud_.reset(new pcl::PointCloud<PointType>());
    accumulated_cloud_->points.reserve(2000);

    for(const auto& topic : radar_objects){
        subs_.push_back(nh_.subscribe<nuscenes2bag::RadarObjects>(
            topic, 10, boost::bind(&RadarAggregator::radarCallback, this, _1, topic)
        ));
    }

    timer_ = nh_.createTimer(ros::Duration(0.05), &RadarAggregator::timerCallback, this);
    ROS_INFO("Radar Front Aggregator Started.");
    }

    void radarCallback(const nuscenes2bag::RadarObjects::ConstPtr& msg, std::string topic_name){
        if (msg->objects.empty()) return;

        // 1. 获取 TF2 变换
        geometry_msgs::TransformStamped transform_stamped;
        try{
            transform_stamped = tf_buffer_.lookupTransform("base_link", msg->header.frame_id, 
                                                         msg->header.stamp, ros::Duration(0.01));
        } catch (tf2::TransformException &ex) {
            ROS_DEBUG_THROTTLE(1.0, "TF2 Error: %s", ex.what());
            return;
        }

        // 2. 准备变换矩阵
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);
        Eigen::Matrix3d rotation_matrix = transform_eigen.rotation();
        
        pcl::PointCloud<PointType> cloud_transformed;
        cloud_transformed.points.reserve(msg->objects.size());

        for(const auto& object:msg->objects){
            if(object.invalid_state != 0) continue;
            if(!object.is_quality_valid) continue;
            
            PointType pt;

            // --- A. 位置变换 (旋转 + 平移) ---
            // P_new = T * P_old
            Eigen::Vector3d pos_sensor(object.pose.x, object.pose.y, object.pose.z);
            Eigen::Vector3d pos_base = transform_eigen * pos_sensor; 
            
            pt.x = pos_base.x();
            pt.y = pos_base.y();
            pt.z = pos_base.z();

            // --- B. 速度变换 (仅旋转，不平移) ---
            Eigen::Vector3d vel_sensor(object.vx_comp, object.vy_comp, 0.0);
            Eigen::Vector3d vel_base = rotation_matrix * vel_sensor;

            pt.vx_comp = vel_base.x();
            pt.vy_comp = vel_base.y();
            
            // 模长 (用于可视化 intensity)
            pt.velocity = vel_base.norm(); 
            if(pt.velocity < MIN_SPEED) continue;
            if(pos_base.z() < MIN_Z || pos_base.z() > MAX_Z) continue;

            cloud_transformed.push_back(pt);
        }

        // 3. 写入 Buffer (加锁)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            *accumulated_cloud_ += cloud_transformed;
            
            // 维护最新时间戳
            if (msg->header.stamp > latest_stamp_) {
                latest_stamp_ = msg->header.stamp;
            }
        }
    }

    void timerCallback(const ros::TimerEvent&){
        pcl::PointCloud<PointType>::Ptr cloud_to_publish(new pcl::PointCloud<PointType>());
        ros::Time pub_stamp;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(accumulated_cloud_->empty()) return;

            // Swap 指针 (O(1))
            accumulated_cloud_.swap(cloud_to_publish);
            pub_stamp = latest_stamp_;
            latest_stamp_ = ros::Time(0);

            // 再次预留内存
            accumulated_cloud_->points.reserve(2000);
        }

        // 锁外执行深拷贝和序列化
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_to_publish, output_msg);

        output_msg.header.frame_id = "base_link";
        // 发布时使用最新时间戳
        output_msg.header.stamp = (pub_stamp == ros::Time(0)) 
                        ? ros::Time::now()  // 无数据时用当前时间
                        : pub_stamp;         // 有数据时用最新时间

        pub_front_cloud_.publish(output_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar__fusion_aggregator_node");
    RadarAggregator node;
    ros::spin();
    return 0;
}