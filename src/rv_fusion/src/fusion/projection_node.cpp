#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>

class ProjectionNode
{
public:
    ProjectionNode()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;

        sub_image_ = nh.subscribe("/yolo_image", 1,
            &ProjectionNode::imageCallback, this);

        sub_lidar_ = nh.subscribe("/lidar_top", 1,
            &ProjectionNode::lidarCallback, this);

        sub_radar_ = nh.subscribe("/radar_front/pointcloud", 1,
            &ProjectionNode::radarCallback, this);

        sub_caminfo_ = nh.subscribe("/cam_front/camera_info", 1,
            &ProjectionNode::cameraInfoCallback, this);

        pub_image_ = nh.advertise<sensor_msgs::Image>(
            "/debug/projected_image", 1);

        ROS_INFO("Projection node started.");
    }

private:
    /* ---------------- ROS ---------------- */
    ros::Subscriber sub_image_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_radar_;
    ros::Subscriber sub_caminfo_;
    ros::Publisher  pub_image_;

    /* ---------------- TF ---------------- */
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /* ---------------- Data Cache ---------------- */
    sensor_msgs::ImageConstPtr last_image_;
    sensor_msgs::PointCloud2ConstPtr last_lidar_;
    sensor_msgs::PointCloud2ConstPtr last_radar_;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool cam_ready_ = false;

    /* ---------------- Callbacks ---------------- */

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        cam_ready_ = true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        last_image_ = msg;
        tryProject();
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        last_lidar_ = msg;
    }

    void radarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        last_radar_ = msg;
    }

    /* --- 辅助函数：根据深度生成伪彩色 (Jet Colormap) --- */
    cv::Scalar getDepthColor(double depth, double max_depth = 60.0)
    {
        // 归一化深度 [0, 1]
        double val = std::min(std::max(depth / max_depth, 0.0), 1.0);
        
        // 简单的热力图映射：近=红 -> 黄 -> 绿 -> 蓝=远
        // OpenCV 的 Hue 范围是 0-180
        // 我们让近处(0m)为红色(0)，远处(max)为蓝色(120)
        // 注意：OpenCV HSV 中 H=0是红, H=60是绿, H=120是蓝
        
        // 这里使用更直观的 BGR 转换逻辑
        int r, g, b;
        if (val < 0.5) {
            // 红 -> 绿
            double interp = val * 2.0;
            b = 0;
            g = static_cast<int>(255 * interp);
            r = static_cast<int>(255 * (1 - interp));
        } else {
            // 绿 -> 蓝
            double interp = (val - 0.5) * 2.0;
            b = static_cast<int>(255 * interp);
            g = static_cast<int>(255 * (1 - interp));
            r = 0;
        }
        // 特殊处理：极近处为亮红
        if(val < 0.1) return cv::Scalar(0, 0, 255);
        
        return cv::Scalar(b, g, r); // OpenCV 使用 BGR
    }

    /* ---------------- Core Logic ---------------- */

    void tryProject()
    {
        if (!last_image_ || !last_lidar_ || !last_radar_ || !cam_ready_)
            return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(
                last_image_, sensor_msgs::image_encodings::BGR8);
        }
        catch (...)
        {
            ROS_WARN("cv_bridge failed.");
            return;
        }

        cv::Mat image = cv_ptr->image;
        // 1. 投影 LiDAR (开启高度过滤 + 深度上色)
        if (last_lidar_)
            projectCloud(last_lidar_, "cam_front", image, cv::Scalar(0,255,0), 1, true);

        // 2. 投影 Radar (红色，不开启深度上色，因为雷达点少且需要醒目)
        if (last_radar_)
            projectCloud(last_radar_, "cam_front", image, cv::Scalar(0,0,255), 4, false);

        sensor_msgs::Image out;
        cv_bridge::CvImage(last_image_->header, "bgr8", image).toImageMsg(out);
        pub_image_.publish(out);
    }

    void projectCloud(
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
        const std::string& target_frame,
        cv::Mat& image,
        const cv::Scalar& base_color,
        int radius,
        bool is_lidar)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        geometry_msgs::TransformStamped tf;
        try
        {
            tf = tf_buffer_.lookupTransform(
                target_frame,
                cloud_msg->header.frame_id,
                ros::Time(0),
                ros::Duration(0.1));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        Eigen::Affine3d T = tf2::transformToEigen(tf);

        for (const auto& p : cloud.points)
        {
            if (is_lidar && p.z < -1.5) {
                continue; 
            }

            Eigen::Vector3d pt(p.x, p.y, p.z);
            Eigen::Vector3d cam_pt = T * pt;

            if (cam_pt.z() <= 0.1)
                continue;

            int u = static_cast<int>(fx_ * cam_pt.x() / cam_pt.z() + cx_);
            int v = static_cast<int>(fy_ * cam_pt.y() / cam_pt.z() + cy_);

            if (u >= 0 && u < image.cols && v >= 0 && v < image.rows)
            {
                cv::Scalar draw_color = base_color;
                
                // --- 深度上色 ---
                if (is_lidar) {
                    draw_color = getDepthColor(cam_pt.z());
                }

                cv::circle(image, cv::Point(u,v), radius, draw_color, -1);
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "projection_node");
    ProjectionNode node;
    ros::spin();
    return 0;
}
