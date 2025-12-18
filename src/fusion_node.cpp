/**
 * @file fusion_node.cpp
 * @brief 雷达与相机的时空同步节点
 * @details 使用 message_filters::ApproximateTime 策略实现“拉链式”数据对齐
 */

 // 1. ROS 核心头文件
#include <ros/ros.h>

// 2. 消息类型头文件 (你的两个输入)
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// 3. MessageFilters 库头文件 (同步的神器)
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL (点云处理)
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV (图像处理)
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// TF & Eigen (坐标变换与数学运算)
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>

// 相机参数消息类型
#include <sensor_msgs/CameraInfo.h>

namespace radar_fusion{
    
// 定义类型别名
typedef sensor_msgs::Image ImageMsg;
typedef sensor_msgs::PointCloud2 RadarMsg;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, RadarMsg> SyncPolicy;

class Fusion_node{
private:
    ros::NodeHandle nh_;

    message_filters::Subscriber<ImageMsg> sub_image_;
    message_filters::Subscriber<RadarMsg> sub_radar_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 发布结果
    ros::Publisher pub_fusion_image_;

    // TF监听器（用来查外参）
    tf::TransformListener tf_listener_;

    // 相机内参矩阵
    Eigen::Matrix3d camera_intrinsic_;

    ros::Subscriber sub_cam_info_; // 专门订阅参数
    bool has_intrinsics_; // 标志位：是否接收到了参数

public:
    Fusion_node() : nh_("~"){
        // 订阅话题
        sub_image_.subscribe(nh_,"/cam_front/raw", 1);
        sub_radar_.subscribe(nh_, "/radar_front/pointcloud", 1);

        // 初始化同步器
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(10), sub_image_, sub_radar_));
        sync_->registerCallback(boost::bind(&Fusion_node::syncCallback, 
            this, _1, _2));

        // 初始化结果发布者
        pub_fusion_image_ = nh_.advertise<sensor_msgs::Image>("/fusion/image_projected", 1);

        // 动态订阅相机内参矩阵
        sub_cam_info_ = nh_.subscribe("/cam_front/camera_info", 1,
                                      &Fusion_node::camInfoCallback, this);

        ROS_INFO("Fusion Node Initialized. Ready to Project!");

    }

    void syncCallback(const ImageMsg::ConstPtr& img,
                      const RadarMsg::ConstPtr& radar){
        // 安全检查：如果还没收到内参，就不能投影
        if (!has_intrinsics_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for camera intrinsics...");
            return;
        }

        // -------------------------------------------------------
        // Step 1: 获取坐标变换 (Extrinsics) T_cam_radar
        // -------------------------------------------------------
        tf::StampedTransform transform;
        try{
            // 核心逻辑：询问 TF 树 "请给我从 radar_front 到 cam_front 的变换"
            // 参数1: 目标坐标系 (Target) -> cam_front
            // 参数2: 源坐标系 (Source)   -> radar_front
            // 参数3: 时间 (ros::Time(0) 代表取最新可用的变换)
            tf_listener_.lookupTransform("cam_front", "radar_front",
            ros::Time(0), transform);
        }
        catch (tf::TransformException& ex){
            ROS_WARN_THROTTLE(1.0, "Could not get transform: %s", ex.what());
            return;
        }

        // 把 ROS 的 TF 格式转换为 Eigen 的矩阵格式 (Isometry3d)
        Eigen::Isometry3d T_cam_radar;
        tf::transformTFToEigen(transform, T_cam_radar);

        // -------------------------------------------------------
        // Step 2: 准备图像 (ROS -> OpenCV)
        // -------------------------------------------------------
        cv_bridge::CvImagePtr cv_ptr;
        try{
            // 把 ROS 图像消息转成 OpenCV 的 cv::Mat 格式
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // -------------------------------------------------------
        // Step 3: 处理点云 & 投影核心数学公式
        // -------------------------------------------------------
        // 把 ROS 点云消息转成 PCL 格式，方便遍历
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*radar, pcl_cloud);

        for(const auto& pt : pcl_cloud.points){
            // A. 取出雷达点 P_radar (x, y, z)
            Eigen::Vector3d p_radar(pt.x, pt.y, pt.z);

            // B. 刚体变换: P_radar -> P_cam (相机坐标系下的点)
            Eigen::Vector3d p_cam = T_cam_radar * p_radar;

            // C. 过滤: 如果点在相机背面 (z < 0)，就不画
            if(p_cam.z() < 0.5) continue;

            // D. 内参投影: P_cam -> P_pixel (像素坐标)
            // 公式: P_pixel = K * P_cam
            Eigen::Vector3d p_pixel = camera_intrinsic_ * p_cam;

            // E. 归一化 (透视除法): u = x/z, v = y/z
            // 也就是把 "锥形" 的光束压扁到 "平面" 上
            int u = static_cast<int>(p_pixel.x() / p_pixel.z());
            int v = static_cast<int>(p_pixel.y() / p_pixel.z());

            // -------------------------------------------------------
            // Step 4: 在图上画圈 (Visualization)
            // -------------------------------------------------------
            // 检查 u, v 是否跑到了图像外面
            if(u >= 0 && u < cv_ptr->image.cols && 
               v >= 0 && cv_ptr->image.rows){
                // 画一个红色的实心圆
                // 参数: 图, 圆心(u,v), 半径, 颜色(BGR), 线宽
                cv::circle(cv_ptr->image, cv::Point(u,v), 5, CV_RGB(255, 0, 0), -1);

                // (可选) 在旁边写上距离
                std::string dist_str = std::to_string(int(p_cam.z())) + "m";
                cv::putText(cv_ptr->image, dist_str, cv::Point(u+5,v ),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 2);
            }
        }
        // -------------------------------------------------------
        // Step 5: 发布结果
        // -------------------------------------------------------
        pub_fusion_image_.publish(cv_ptr->toImageMsg());
        ROS_INFO_STREAM("Projected " << pcl_cloud.points.size() 
        << " radar points to image.");
    }

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
        if(has_intrinsics_) return;

        camera_intrinsic_ << msg->K[0], msg->K[1], msg->K[2],
                             msg->K[3], msg->K[4], msg->K[5],
                             msg->K[6], msg->K[7], msg->K[8];

        has_intrinsics_ = true;
        ROS_INFO_STREAM("\033[1;32m[Intrinsics Received]\033[0m fx: " 
            << msg->K[0] << " cx: " << msg->K[2]);
        
        // 既然拿到了内参，我们可以取消订阅以节省资源（可选）
        sub_cam_info_.shutdown();
    }
};

}

int main(int argc, char** argv){
    ros::init(argc, argv, "radar_camera_fusion_node");
    radar_fusion::Fusion_node node;
    ros::spin();
    return 0;
}
