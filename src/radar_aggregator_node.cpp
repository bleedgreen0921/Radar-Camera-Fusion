/*
雷达数据聚合节点：
功能:
 * 1. 订阅五个毫米波雷达话题
 * 2. 将所有点变换到车身坐标系
 * 3. 合并为一个点云
 * 4. 发布到 /radar/surround_pointcloud 话题，供 Rviz 可视化
*/

#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <nuscenes2bag/RadarObjects.h> // 要订阅的、来自 nuscenes2bag 包的自定义 RadarObjects 消息

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

// TF
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

class RadarAggregator
{
private:
    ros::NodeHandle nh_;       // ROS 节点句柄
    std::vector<ros::Subscriber> subs_;
    ros::Publisher pub_surround_cloud_;
    ros::Timer timer_;

    tf::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_; // 聚合池
    boost::mutex mutex_; // 线程锁

public:
    RadarAggregator() : nh_("~")
    {
        pub_surround_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar/surround_pointcloud", 10);

        std::vector<std::string> radar_topics = {
            "/radar_front", "/radar_front_left", "/radar_front_right",
            "/radar_back_left", "/radar_back_right"
        };

        // 订阅所有雷达
        for(const auto& topic:radar_topics){
            ros::Subscriber sub = nh_.subscribe<nuscenes2bag::RadarObjects>(
                topic, 10, boost::bind(&RadarAggregator::radarCallback, this, _1, topic)
            );
            subs_.push_back(sub);
        }

        timer_ = nh_.createTimer(ros::Duration(0.05), &RadarAggregator::timerCallback, this);

        ROS_INFO("Radar Aggregator Initialized. Listening to 5 radars...");
    }

    // 通用回调函数：处理任意一个雷达的消息
    void radarCallback(const nuscenes2bag::RadarObjects::ConstPtr& msg, std::string topic_name){
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;

        for (const auto& object : msg->objects){
            pcl::PointXYZ pt;
            pt.x = object.pose.x;
            pt.y = object.pose.y;
            pt.z = object.pose.z;
            temp_cloud.push_back(pt);
        }

        // 获取 TF 变换：从 "雷达坐标系" -> "base_link" (车身)
        tf::StampedTransform transform;
        try{
            tf_listener_.lookupTransform("base_link", msg->header.frame_id,
            ros::Time(0), transform);
        }catch(tf::TransformException& ex){
            return;
        }
        // 执行坐标变换
        Eigen::Affine3d transform_eigin;
        tf::transformTFToEigen(transform, transform_eigin);
        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        pcl::transformPointCloud(temp_cloud, cloud_transformed, transform_eigin);

        // 加入到全局缓冲池 (加锁保护，防止多线程冲突)
        boost::mutex::scoped_lock lock(mutex_);
        accumulated_cloud_ += cloud_transformed;
    }

    void timerCallback(const ros::TimerEvent&){
        boost::mutex::scoped_lock lock(mutex_);

        if(accumulated_cloud_.empty()) return;

        // 1. 转为 ROS 消息
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(accumulated_cloud_, output_msg);

        // 2. 填写 Header
        output_msg.header.frame_id = "base_link";
        output_msg.header.stamp = ros::Time::now();

        // 3. 发布
        pub_surround_cloud_.publish(output_msg);

        // 4. 清空缓冲池，准备下一轮聚合
        accumulated_cloud_.clear();
    }
};


// 4. (入口) C++ 的 main 函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_aggregator_node");

    RadarAggregator node;

    // (关键) ros::spin()
    // 让节点“活”过来，开始等待和处理回调函数 (Callback)
    // 节点会一直“卡”在这里，直到你按下 Ctrl+C
    ros::spin();

    return 0;
}