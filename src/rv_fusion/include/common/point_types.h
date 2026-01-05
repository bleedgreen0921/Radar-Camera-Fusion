#ifndef RV_FUSION_POINT_TYPES_H
#define RV_FUSION_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace rv_fusion {
// 定义与 Aggregator 输出一致的自定义点云结构
struct PointRadar {
    PCL_ADD_POINT4D;                  // x, y, z, padding
    float velocity;                   // 速度模长 (对应 intensity)
    float vx_comp;                    // 补偿后的 X 轴速度
    float vy_comp;                    // 补偿后的 Y 轴速度
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 内存对齐
} EIGEN_ALIGN16;
} // namespace rv_fusion

// 注册点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(rv_fusion::PointRadar,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, velocity, intensity)      // 映射: velocity成员 -> intensity字段
    (float, vx_comp, vx_comp)         
    (float, vy_comp, vy_comp)
)

#endif // RV_FUSION_POINT_TYPES_H