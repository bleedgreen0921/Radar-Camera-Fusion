# 基于Nuscenes v1.0-mini数据集的雷视融合项目

## 一、毫米波雷达数据预处理与聚合

### 1. **核心功能**

- 订阅多个雷达话题（5个方向的5个雷达数据）
- 将各雷达坐标系下的点云统一转换到车身坐标系（base_link）
- 聚合所有雷达数据并发布为单个点云
- 双缓冲模式：实现无锁/低延迟数据处理，优化性能，降低延迟

### 2. **数据流管理**

```cpp
// 双缓冲机制的关键组件
pcl::PointCloud<PointType>::Ptr accumulated_cloud_;  // 当前收集缓冲区
ros::Time latest_stamp_;  // 时间戳管理
std::mutex mutex_;  // 线程同步
```

**工作流程**：

1. **生产者**（radarCallback）：接收数据 → 坐标变换 → 加到**缓冲池**
2. **消费者**（timerCallback）：**交换指针** → 发布 → 重置缓冲区

### 3. **TF2 变换优化**

```cpp
// 使用 TF2 而不是传统 TF
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;

// 高效的变换获取
transform_stamped = tf_buffer_.lookupTransform("base_link", 
                                             msg->header.frame_id, 
                                             msg->header.stamp, 
                                             ros::Duration(0.01));
```

**优势**：

- 使用 `tf2_eigen`直接转换为 Eigen 矩阵
- 指定超时时间（0.01s），避免阻塞
- 时间戳精确对应，减少插值误差

### 4. **点云处理优化**

```cpp
// 预处理：预留内存
accumulated_cloud_->points.reserve(2000);

// 高效交换，避免深拷贝
accumulated_cloud_.swap(cloud_to_publish);
```

- 内存预分配减少动态分配开销
- 指针交换（O(1)复杂度）替代深拷贝

### 5. **双缓冲设计**

```cpp
// 生产者（回调函数）
{
    std::lock_guard<std::mutex> lock(mutex_);
    *accumulated_cloud_ += cloud_transformed;  // 快速添加
    if (msg->header.stamp > latest_stamp_) {
        latest_stamp_ = msg->header.stamp;  // 更新时间戳
    }
}

// 消费者（定时器）
{
    std::lock_guard<std::mutex> lock(mutex_);
    accumulated_cloud_.swap(cloud_to_publish);  // 极速交换
    accumulated_cloud_->points.reserve(2000);  // 重新预留
}
```

**优势**：锁占用时间极短（仅指针交换和内存预留）

### 6. **内存优化**

- 每次回调使用栈上临时点云
- 聚合时使用 `+=`操作符（PCL优化过的拼接）
- 定期清理并重新预分配

### 7. **时间戳管理**

```cpp
output_msg.header.stamp = (pub_stamp == ros::Time(0)) 
                          ? ros::Time::now() 
                          : pub_stamp;
```

- 优先使用最新雷达数据的时间戳
- 无数据时使用当前时间，保证连续性

------

## 二、目标检测与聚类算法

运行步骤：
终端1：
roscore
rosparam set use_sim_time true

2：rosrun rv_fusion node

3. rosbag play --clock bag --loop

