#ifndef RV_FUSION_POINT_TYPES_H
#define RV_FUSION_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace rv_fusion {

struct PointRadar {
    PCL_ADD_POINT4D;                  // x, y, z, padding
    float velocity;                   // 速度模长
    float vx_comp;                    // 补偿后的 X 轴速度
    float vy_comp;                    // 补偿后的 Y 轴速度
    float confidence;                 // [新增] 置信度
    uint32_t timestamp;               // [新增] 时间戳(ms)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // [新增] 工具方法
    inline float getSpeed() const { return velocity; }
    inline float getVelocityX() const { return vx_comp; }
    inline float getVelocityY() const { return vy_comp; }
    inline float getDistance() const { return sqrt(x*x + y*y + z*z); }
} EIGEN_ALIGN16;

} // namespace rv_fusion

// 注册点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(rv_fusion::PointRadar,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, velocity, velocity)
    (float, vx_comp, vx_comp)
    (float, vy_comp, vy_comp)
    (float, confidence, confidence)
    (std::uint32_t, timestamp, timestamp)
)

#endif