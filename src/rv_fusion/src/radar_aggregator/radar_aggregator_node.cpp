/*
雷达数据聚合节点 (TF2 深度优化版)：
 * 1. 订阅 nuscenes2bag::RadarObjects
 * 2. 使用 TF2 + Eigen 将点变换到车身坐标系
 * 3. 使用双缓冲 (Double Buffering) 无锁/低延迟聚合
 * 4. 修正输出时间戳
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

typedef pcl::PointXYZI PointType;

class RadarAggregator {
private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subs_;
    ros::Publisher pub_surround_cloud_;
    ros::Timer timer_;

    // 
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 
    pcl::PointCloud<PointType>::Ptr accumulated_cloud_;
    ros::Time latest_stamp_;
    std::mutex mutex_;

public:
    // 初始化列表：先初始化 tf_listener_，传入 buffer
    RadarAggregator() : nh_("~"), tf_listener_(tf_buffer_) {
    pub_surround_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar/surround_pointcloud", 10);

    std::vector<std::string> radar_objects = {
        "/radar_front", "/radar_front_left", "/radar_front_right",
            "/radar_back_left", "/radar_back_right"
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
    ROS_INFO("Optimized Radar Aggregator (TF2 Version) Started.");
    }

    void radarCallback(const nuscenes2bag::RadarObjects::ConstPtr& msg, std::string topic_name){
        // 1. 提取点云 (解析自定义消息)
        pcl::PointCloud<PointType> temp_cloud;
        temp_cloud.points.reserve(msg->objects.size());

        for (const auto& object : msg->objects){
            PointType pt;
            pt.x = object.pose.x;
            pt.y = object.pose.y;
            pt.z = object.pose.z;
            float vx = object.vx_comp;
            float vy = object.vy_comp;
            pt.intensity = std::sqrt(vx*vx + vy*vy);
            temp_cloud.push_back(pt);
        }

        if (temp_cloud.empty()) return;

        // 2. 获取 TF2 变换
        // --- 变化点 2: 使用 tf_buffer_ ---
        geometry_msgs::TransformStamped transform_stamped;
        try{
            // lookupTransform(target, source, time, timeout)
            // 0.01s 超时，防止阻塞
            transform_stamped = tf_buffer_.lookupTransform("base_link", msg->header.frame_id, 
                                                         msg->header.stamp, ros::Duration(0.01));
        } catch (tf2::TransformException &ex) {
            ROS_DEBUG_THROTTLE(1.0, "TF2 Error: %s", ex.what());
            return;
        }

        // 3. 转换并变换点云
        // --- 变化点 3: 使用 tf2_eigen ---
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);
        
        pcl::PointCloud<PointType> cloud_transformed;
        pcl::transformPointCloud(temp_cloud, cloud_transformed, transform_eigen);

        // 4. 写入 Buffer (临界区)
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

        // [优化] 极速交换，最小化锁时间
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
        // 修正时间戳
        output_msg.header.stamp = (pub_stamp == ros::Time(0)) ? ros::Time::now() : pub_stamp;

        pub_surround_cloud_.publish(output_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_aggregator_node");
    RadarAggregator node;
    ros::spin();
    return 0;
}