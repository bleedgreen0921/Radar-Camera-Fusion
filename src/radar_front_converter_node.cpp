/*
雷达数据转换器节点
功能:
 * 1. 订阅 /radar_front 话题 (自定义的 nuscenes2bag/RadarObjects 消息)
 * 2. 提取 x, y, z 坐标
 * 3. 重新打包成标准的 sensor_msgs/PointCloud2
 * 4. 发布到 /radar_front/pointcloud 话题，供 Rviz 可视化
*/

// ROS 核心库
#include <ros/ros.h> 
// PCL (点云库) 的核心定义
#include<pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL-ROS 转换库
#include <pcl_conversions/pcl_conversions.h>
// Rviz 认识的标准 sensor_msgs::PointCloud2 消息的定义
#include <sensor_msgs/PointCloud2.h>
// 自定义雷达消息。(我们的“输入”格式)
// 你要订阅的、来自 nuscenes2bag 包的自定义 RadarObjects 消息的定义
#include <nuscenes2bag/RadarObjects.h>

class RadarConverter
{
public:
    // 构造函数：当这个节点启动时，它会运行一次
    RadarConverter()
    {
        // 告诉 ROS Master，我们要“发布”一个新话题
        // 话题名: "/radar_front/pointcloud"
        // 消息类型: sensor_msgs::PointCloud2
        // 队列大小: 10
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_front/pointcloud", 10);

        // 告诉 ROS Master，我们要“订阅”一个旧话题
        // 话题名: "/radar_front" (来自 rosbag)
        // 队列大小: 1
        // 回调函数: 当收到消息时，调用 radarCallback
        sub_ = nh_.subscribe("/radar_front", 1, &RadarConverter::radarCallback, this);

        ROS_INFO("Radar Converter Node has started.");
    }

private:
    // 3. (核心) 回调函数 (我们之前讨论的“工作手册”)
    void radarCallback(const nuscenes2bag::RadarObjects::ConstPtr& msg)
    {
        // --- 积木 3 & 4：创建 PCL 点云 ---
        // 创建一个“简单”的 PCL 点云对象 (用 XYZ 类型的点)
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // --- 积木 3：遍历和提取 ---
        // 遍历自定义消息中的 'objects' 数组
        for (const auto& object : msg->objects)
        {
            // 从“富矿”数据中提取 x, y, z
            // (注意：我们暂时只用了 x,y,z。我们未来还可以用 vx, vy, rcs！)
            pcl_cloud->emplace_back(
                object.pose.x,
                object.pose.y,
                object.pose.z
            );
        }

        // 设置 PCL 点云的元数据 (metadata)
        pcl_cloud->width = pcl_cloud->points.size();
        pcl_cloud->height = 1;      // "1" 表示这是一个“无序”点云
        pcl_cloud->is_dense = true;

        // --- 积木 4 & 5：创建、转换并发布 ---

        // 1. 创建一个“复杂”的、用于发布的 ROS 标准消息
        sensor_msgs::PointCloud2 output_msg;

        // 2. (魔法!) 调用 PCL-ROS 转换库
        // 把“简单”的 pcl_cloud 转换成“复杂”的 output_msg
        pcl::toROSMsg(*pcl_cloud, output_msg);

        // 3. (关键!) 设置 output_msg 的 Header (坐标系和时间戳)
        // Rviz 必须知道这个点云是在哪个坐标系(frame_id)和哪个时间(stamp)
        // 我们直接从输入消息 (msg->header) 复制过来就行！
        output_msg.header = msg->header;

        // 4. (发布!) 把打包好的“标准快递盒”放到新的传送带上
        pub_.publish(output_msg);
    }

protected:
    ros::NodeHandle nh_;       // ROS 节点句柄 (和 ROS Master 通信)
    ros::Subscriber sub_;      // 我们的“订阅者”
    ros::Publisher  pub_;      // 我们的“发布者”
};


// 4. (入口) C++ 的 main 函数
int main(int argc, char** argv)
{
    // 初始化 ROS，并给我们的节点起个名字 "radar_converter_node"
    ros::init(argc, argv, "radar_converter_node");

    // 创建我们的 RadarConverter 对象
    RadarConverter converter;

    // (关键) ros::spin()
    // 让节点“活”过来，开始等待和处理回调函数 (Callback)
    // 节点会一直“卡”在这里，直到你按下 Ctrl+C
    ros::spin();

    return 0;
}