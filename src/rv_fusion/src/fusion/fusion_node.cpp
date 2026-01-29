/**
 * fusion_node.cpp
 * 功能：多传感器后融合 (Camera + LiDAR + Radar)
 * 特性：
 * 1. 4路时间同步 (Image, YOLO, LiDAR, Radar)
 * 2. 视锥体关联 (Frustum Association)
 * 3. 状态估计 (LiDAR定位置, Radar定速度)
 * 4. 重绘法 (Redraw): 在原始图像上绘制 3D 边界框
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <ultralytics_ros/YoloResult.h> 
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "common/point_types.h"

struct FusedObject{
    std::string class_id;
    double x, y, z; 
    double vx, vy; 
    double width, height, length;
};

// 定义尺寸结构体
struct BoxDim {
    double length; // x 方向
    double width;  // y 方向
    double height; // z 方向
};

// 辅助函数：根据类别ID获取经验尺寸
BoxDim getDimByClass(const std::string& class_id) {
    BoxDim dim = {1.0, 1.0, 1.0}; 

    // 注意：class_id 可能是 "0" (int转string) 或者 "person" (标签名)，取决于你的实现
    int id = -1;
    try {
        id = std::stoi(class_id);
    } catch (...) {}

    if (id == 0 || class_id == "person") {
        return {0.5, 0.5, 1.7}; // 人
    } 
    else if (id == 1 || class_id == "bicycle" || id == 3 || class_id == "motorcycle") {
        return {1.5, 0.6, 1.2}; // 两轮车
    }
    else if (id == 2 || class_id == "car") {
        return {4.5, 2.0, 1.6}; // 轿车
    }
    else if (id == 5 || class_id == "bus") {
        return {12.0, 3.0, 3.2}; // 巴士
    }
    else if (id == 7 || class_id == "truck") {
        return {8.0, 2.5, 3.0}; // 卡车
    }
    
    return dim;
}

class FusionNode{

public:
    FusionNode() : tf_listener_(tf_buffer_) {
        ros::NodeHandle nh;

        // 1. 订阅 Camera Info (获取内参)
        cam_info_sub_ = nh.subscribe("/cam_front/camera_info", 1, &FusionNode::camInfoCallback, this);

        // 2. 订阅传感器数据
        yolo_sub_.subscribe(nh, "/yolo_result", 1);
        lidar_sub_.subscribe(nh, "/lidar_top", 1); 
        radar_sub_.subscribe(nh, "/radar/front_pointcloud", 1); // 聚合后的雷达点云

        // 3. 定义同步策略 (容差 0.1s)
        typedef message_filters::sync_policies::ApproximateTime<
            ultralytics_ros::YoloResult, 
            sensor_msgs::PointCloud2, 
            sensor_msgs::PointCloud2
        > MySyncPolicy;

        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(
            MySyncPolicy(10), yolo_sub_, lidar_sub_, radar_sub_
        ));
        
        sync_->registerCallback(boost::bind(&FusionNode::fusionCallback, this, _1, _2, _3));

        // 4. 发布融合结果
        pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/fusion/3d_markers", 1);

        ROS_INFO("Fusion Node Started. Waiting for data...");
    }
    
private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    ros::Subscriber cam_info_sub_;
    message_filters::Subscriber<ultralytics_ros::YoloResult> yolo_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub_;

    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        ultralytics_ros::YoloResult, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>>> sync_;

    ros::Publisher pub_markers_;

    // 相机内参
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool cam_ready_ = false;

    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        fx_ = msg->K[0]; fy_ = msg->K[4]; cx_ = msg->K[2]; cy_ = msg->K[5];
        cam_ready_ = true;
    }

    void fusionCallback(
        const ultralytics_ros::YoloResultConstPtr& yolo_msg,
        const sensor_msgs::PointCloud2ConstPtr& lidar_msg,
        const sensor_msgs::PointCloud2ConstPtr& radar_msg){
            if (!cam_ready_) return;

            // --- 1. 数据解析与坐标系准备 ---
            pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

            pcl::PointCloud<rv_fusion::PointRadar>::Ptr radar_cloud(new pcl::PointCloud<rv_fusion::PointRadar>);
            pcl::fromROSMsg(*radar_msg, *radar_cloud);

            // C. 获取 TF 变换
            // 1. 用于投影判断： Point_Source -> Cam_Front (用于判断点是否在图像框内)
            // 2. 用于统一输出： Point_Source -> Base_Link (用于输出最终车辆坐标)
            
            Eigen::Affine3d T_base2cam; 
            if (!getTransform("base_link", "cam_front", T_base2cam)) return;

            Eigen::Affine3d T_lidar2cam, T_lidar2base;
            if (!getTransform(lidar_msg->header.frame_id, "cam_front", T_lidar2cam)) return;
            if (!getTransform(lidar_msg->header.frame_id, "base_link", T_lidar2base)) return;

            // --- 2. 遍历 YOLO 检测框 ---
            std::vector<FusedObject> fused_objects;
            
            if (yolo_msg->detections.detections.empty()) return;

            for (const auto& det : yolo_msg->detections.detections) {
                
                // 解析 2D 框
                double cx = det.bbox.center.x;
                double cy = det.bbox.center.y;
                double w  = det.bbox.size_x;
                double h  = det.bbox.size_y;
                int u_min = cx - w / 2; int u_max = cx + w / 2;
                int v_min = cy - h / 2; int v_max = cy + h / 2;

                // --- 3. 关联 LiDAR (为了精准的位置) ---
                std::vector<Eigen::Vector3d> valid_lidar_points_base; // 存 base_link 下的坐标
                
                for (const auto& pt : lidar_cloud->points) {
                    // 3.1 投影判断 (用 cam 坐标系)
                    Eigen::Vector3d pt_lidar(pt.x, pt.y, pt.z);
                    Eigen::Vector3d pt_cam = T_lidar2cam * pt_lidar;
                    
                    if (pt_cam.z() <= 1.0) continue; // 过滤太近的点

                    cv::Point2f uv = project3Dto2D(pt_cam);
                    if (uv.x >= u_min && uv.x <= u_max && uv.y >= v_min && uv.y <= v_max) {
                        // 3.2 如果在框内，转到 base_link 保存
                        valid_lidar_points_base.push_back(T_lidar2base * pt_lidar);
                    }
                }

                // --- 4. 关联 Radar (为了精准的速度矢量) ---
                std::vector<Eigen::Vector3d> valid_radar_vels; // 存速度矢量 (vx, vy, 0)
                std::vector<Eigen::Vector3d> valid_radar_points_base; // 存位置 (备用)

                for (const auto& pt : radar_cloud->points) {
                    // 4.1 投影判断
                    // 聚合后的雷达点 pt.x, pt.y, pt.z 已经在 base_link 下
                    Eigen::Vector3d pt_base(pt.x, pt.y, pt.z);
                    Eigen::Vector3d pt_cam = T_base2cam * pt_base; // 转到相机投影

                    if (pt_cam.z() <= 1.0) continue;

                    cv::Point2f uv = project3Dto2D(pt_cam);
                    if (uv.x >= u_min && uv.x <= u_max && uv.y >= v_min && uv.y <= v_max) {
                        // 4.2 提取速度矢量 (关键！)
                        // pt.vx_comp 和 pt.vy_comp 已经在 base_link 下 (由聚合节点保证)
                        valid_radar_vels.push_back(Eigen::Vector3d(pt.vx_comp, pt.vy_comp, 0));
                        valid_radar_points_base.push_back(pt_base);
                    }
                }

                // --- 5. 融合决策 ---
                FusedObject obj;
                // 获取类别和ID
                obj.class_id = det.results.empty() ? "Obj" : std::to_string(det.results[0].id); // 这里存的是 Class ID
                // 如果 YOLO 输出了跟踪 ID (Track ID)，通常在 id 字段，这里假设 results[0].id 是类别ID
                // 如果 ultralytics_ros 的实现中 result[0].id 是 track_id，则直接使用
                
                // A. 位置融合 (优先 LiDAR)
                if (!valid_lidar_points_base.empty()) {
                    // 计算 LiDAR 点云质心 (Base Link)
                    Eigen::Vector3d centroid(0,0,0);
                    for(auto& p : valid_lidar_points_base) centroid += p;
                    centroid /= valid_lidar_points_base.size();
                    
                    obj.x = centroid.x();
                    obj.y = centroid.y();
                    obj.z = centroid.z();
                } 
                else if (!valid_radar_points_base.empty()) {
                    // 降级：使用 Radar 质心
                    Eigen::Vector3d centroid(0,0,0);
                    for(auto& p : valid_radar_points_base) centroid += p;
                    centroid /= valid_radar_points_base.size();
                    
                    obj.x = centroid.x();
                    obj.y = centroid.y();
                    obj.z = centroid.z();
                } else {
                    continue; // 只有视觉，没有深度，跳过
                }

                // B. 速度融合 (使用 Radar 矢量平均)
                if (!valid_radar_vels.empty()) {
                    Eigen::Vector3d avg_vel(0,0,0);
                    for(auto& v : valid_radar_vels) avg_vel += v;
                    avg_vel /= valid_radar_vels.size();
                    
                    obj.vx = avg_vel.x();
                    obj.vy = avg_vel.y();
                } else {
                    obj.vx = 0;
                    obj.vy = 0;
                }

                fused_objects.push_back(obj);
            }

            // 6. 发布可视化 Markers
            publishMarkers(fused_objects);
        }

    bool getTransform(const std::string& source, const std::string& target, Eigen::Affine3d& T) {
        try {
            geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
                target, source, ros::Time(0), ros::Duration(0.1));
            T = tf2::transformToEigen(tf);
            return true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5, "TF Error (%s -> %s): %s", source.c_str(), target.c_str(), ex.what());
            return false;
        }
    }

    cv::Point2f project3Dto2D(const Eigen::Vector3d& pt) {
        double u = fx_ * pt.x() / pt.z() + cx_;
        double v = fy_ * pt.y() / pt.z() + cy_;
        return cv::Point2f(u, v);
    }

    void publishMarkers(const std::vector<FusedObject>& objects) {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        for (const auto& obj : objects) {
            // --- 获取动态尺寸 ---
            BoxDim dim = getDimByClass(obj.class_id);

            // 1. 3D Bounding Box
            visualization_msgs::Marker box;
            box.header.frame_id = "base_link"; // 结果都在 base_link 下
            box.header.stamp = ros::Time::now();
            box.ns = "fusion_box";
            box.id = id++;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            
            box.pose.position.x = obj.x;
            box.pose.position.y = obj.y;
            box.pose.position.z = obj.z;
            box.pose.orientation.w = 1.0;
            
            // // 检测框大小不可修改，不可取
            // box.scale.x = 4.5; // 假定车长
            // box.scale.y = 2.0; // 假定车宽
            // box.scale.z = 1.5; 

            // 使用查表得到的尺寸
            box.scale.x = dim.length; 
            box.scale.y = dim.width;  
            box.scale.z = dim.height;
            
            // 根据类别设置不同颜色
            if (obj.class_id == "0" || obj.class_id == "person") {
                box.color.r = 1.0; box.color.g = 1.0; box.color.b = 0.0; // 人显示黄色
            } else {
                box.color.r = 0.0; box.color.g = 1.0; box.color.b = 0.0; // 车显示绿色
            }
            box.color.a = 0.5;
            box.lifetime = ros::Duration(0.1);

            // 2. 速度箭头 (Visualization of Velocity Vector)
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "base_link";
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "fusion_velocity";
            arrow.id = id++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            
            // 起点
            geometry_msgs::Point p_start;
            p_start.x = obj.x; p_start.y = obj.y; p_start.z = obj.z + dim.height / 2.0 + 0.5;
            arrow.points.push_back(p_start);
            
            // 终点 (根据速度矢量延伸)
            geometry_msgs::Point p_end;
            p_end.x = obj.x + obj.vx; 
            p_end.y = obj.y + obj.vy; 
            p_end.z = obj.z + 1.0;
            arrow.points.push_back(p_end);
            
            arrow.scale.x = 0.2; // 轴径
            arrow.scale.y = 0.4; // 箭头径
            arrow.color.r = 1.0; arrow.color.g = 0.0; arrow.color.b = 0.0; arrow.color.a = 1.0;
            arrow.lifetime = ros::Duration(0.1);

            // 3. 文字信息
            visualization_msgs::Marker text;
            text.header.frame_id = "base_link";
            text.header.stamp = ros::Time::now();
            text.ns = "fusion_text";
            text.id = id++;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            
            text.pose.position.x = obj.x;
            text.pose.position.y = obj.y;
            text.pose.position.z = obj.z + 2.0;
            
            double speed = sqrt(obj.vx*obj.vx + obj.vy*obj.vy);
            std::stringstream ss;
            ss << "ID:" << obj.class_id << "\n"
               << "Vx:" << std::fixed << std::setprecision(1) << obj.vx << " Vy:" << obj.vy << "\n"
               << "Speed:" << speed << "m/s";
            
            text.text = ss.str();
            text.scale.z = 0.5; 
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(box);
            marker_array.markers.push_back(arrow);
            marker_array.markers.push_back(text);
        }
        pub_markers_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    FusionNode node;
    ros::spin();
    return 0;
}
