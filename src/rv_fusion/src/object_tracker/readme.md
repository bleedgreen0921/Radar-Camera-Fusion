# 多目标跟踪（MOT）节点

## 1. 整体架构概述

这是一个基于**扩展卡尔曼滤波(EKF)**和**匈牙利算法**的多目标跟踪系统，主要针对雷达点云数据进行目标跟踪。系统采用经典的**预测-关联-更新**跟踪范式。

------

## 2. 轨迹数据结构和状态定义

### 2.1. 轨迹状态管理 (TrackState)

**状态机设计：**

```c++
// 轨迹的生命周期状态
enum TrackState {
    TENTATIVE,  // 疑似目标 (刚出现，还不够稳定)
    CONFIRMED,  // 确认目标 (连续多帧检测到)
    DELETED     // 待删除 (连续多帧丢失)
};
```

```c++
TENTATIVE → CONFIRMED → DELETED
    ↑          ↓
    └──────────┘ (状态回退机制)
```

**状态转换逻辑：**

- **TENTATIVE(疑似目标)**：新创建的轨迹，需要连续`min_hits_to_confirm_`帧匹配成功才能确认
- **CONFIRMED(确认目标)**：稳定跟踪的目标，具有较高的可信度
- **DELETED(待删除)**：连续`max_coast_cycles_`帧丢失匹配，等待清理

### 2.2. Track 数据结构分析

#### 核心标识信息

```cpp
int id;                 // 唯一标识符，确保轨迹可追溯
double timestamp;       // 关键设计：支持异步传感器数据的时间同步
```

#### 生命周期计数器

```cpp
int age;                // 总存活帧数 → 衡量轨迹稳定性
int coast_cycles;       // 连续丢失计数 → 决定是否删除
int hit_streak;         // 连续匹配计数 → 决定是否确认
```

**设计优势**：清晰的计数分离，便于状态判断和参数调优

#### 状态与属性

```cpp
TrackState state;       // 当前生命周期状态
ObjectClass class_id;   // 预留扩展：支持多类别目标跟踪
double score;           // 轨迹质量评估 → 可用于输出筛选
```

#### 滤波与状态估计

```cpp
ExtendedKalmanFilter ekf; // 核心状态估计器
```

**关键技术点**：

- 每个轨迹独立维护EKF实例
- 支持非线性运动模型（扩展卡尔曼滤波）
- 状态向量包含位置、速度等运动信息

#### 可视化与调试支持

```cpp
std::deque<Eigen::Vector2d> history; // 历史轨迹缓存
const size_t max_history_size = 20;  // 内存控制
```

**设计特点**：

- 使用`deque`实现FIFO缓存，高效管理历史数据
- 限制缓存长度，避免内存无限增长
- 专为ROS Rviz可视化优化

### 2.3. 辅助功能分析

#### 3.1 历史更新机制

```cpp
void updateHistory() {
    Eigen::Vector4d x = ekf.getCartesianState();  // 获取笛卡尔坐标
    history.push_back(Eigen::Vector2d(x(0), x(1))); // 仅存储位置
    if (history.size() > max_history_size) {
        history.pop_front();  // 自动清理旧数据
    }
}
```

**优化设计**：

- 只存储位置信息，减少内存占用
- 自动维护固定长度，避免内存泄漏

### 2.4. 与ObjectTracker的协作关系

#### 4.1 状态管理参数

```cpp
// ObjectTracker中的相关参数
int max_coast_cycles_;      // 允许最大连续丢失帧数，与Track::coast_cycles配合
int min_hits_to_confirm_;   // 确认为真目标的帧数，与Track::hit_streak配合
```

#### 4.2 数据流设计

```
检测输入 → 数据关联 → Track状态更新 → 输出有效轨迹
    ↓
EKF预测/更新 → history维护 → 可视化输出
```

### 2.5. 潜在改进建议

1. **序列化支持**：可添加序列化接口，便于轨迹数据持久化
2. **轨迹质量评估**：score字段可结合更多因素（如轨迹平滑度）
3. **多模型支持**：可扩展支持不同运动模型的EKF实例

------

## 3. 关联算法实现

### 1. 整体架构设计

这是一个**多目标跟踪数据关联模块**，采用**匈牙利算法 + 马氏距离门控**的经典架构：

```markdown
输入: 轨迹预测状态 + 当前检测 → 代价矩阵构建 → 匈牙利算法全局匹配 → 门限过滤 → 输出关联结果
```

### 2. 距离计算算法分析

#### 2.1 马氏距离近似实现

```
double calculateDistance(const Eigen::Vector4d& track_state,
                        const Eigen::Vector4d& det_meas,
                        const Eigen::MatrixXd& P_matrix)
```

**核心思想**：用**加权欧氏距离**近似**标准马氏距离**，避免矩阵求逆的不稳定性。

**标准马氏距离公式**：

```
D_mahalanobis = √[(z - Hx)ᵀ × S⁻¹ × (z - Hx)]
其中 S = HPHᵀ + R
```

**当前近似实现**：

```
// 对角线加权法：用方差倒数作为权重
double w_x = 1.0 / (var_x + 0.5);  // 位置x权重
double w_y = 1.0 / (var_y + 0.5);  // 位置y权重  
double w_v = 1.0 / (var_v + 1.0);   // 速度权重
```

#### 2.2 方差提取策略

```
double var_x = P_matrix(0, 0);  // 位置x方差
double var_y = P_matrix(1, 1);  // 位置y方差
double var_v = P_matrix(2, 2);  // 速度模长方差（近似处理）
```

**技术难点**：CTRV模型状态向量为`[px, py, v, yaw, yaw_rate]`，但观测是`[px, py, vx, vy]`，存在坐标系转换。

#### 2.3 数值稳定性处理

```
// 避免除以0：方差加小量平滑
1.0 / (var_x + 0.5)  // 位置分量
1.0 / (var_v + 1.0)  // 速度分量
```

### 3. 数据关联流程分析

#### 3.1 代价矩阵构建

```
// 扁平化存储：row-major顺序
distMatrix_t cost_matrix(n_tracks * n_dets);
for (size_t i = 0; i < n_tracks; ++i) {
    for (size_t j = 0; j < n_dets; ++j) {
        cost_matrix[i * n_dets + j] = calculateDistance(...);
    }
}
```

**设计特点**：内存连续访问，优化缓存性能。

#### 3.2 匈牙利算法调用

```
AssignmentProblemSolver solver;
assignments_t assignment; // assignment[i] = j (Track i -> Det j)
solver.Solve(cost_matrix, n_tracks, n_dets, assignment, 
             AssignmentProblemSolver::optimal);
```

**算法选择**：使用**最优分配**而非贪心算法，确保全局最优解。

#### 3.3 门限过滤机制

```
if (cost < dist_threshold_) {
    matches.push_back({(int)i, assigned_det_idx});
} else {
    unmatched_tracks.push_back(i); // 距离过大，拒绝关联
}
```

**关键参数**：`dist_threshold_ = 12.0`基于4自由度卡方分布99%置信区间。

### 4. 输出结果分类

关联结果分为三类：

- 

  **matches**: 成功关联的轨迹-检测对

- 

  **unmatched_tracks**: 未匹配的轨迹（可能目标消失）

- 

  **unmatched_detections**: 未匹配的检测（可能新目标出现）

### 5. 算法优势与局限

#### 5.1 优势

1. 

   **全局最优**：匈牙利算法确保整体匹配代价最小

2. 

   **不确定性感知**：马氏距离考虑估计不确定性

3. 

   **数值稳定**：避免直接矩阵求逆，增强鲁棒性

4. 

   **边界条件完善**：空输入处理完备

#### 5.2 局限性及改进建议

**问题1：速度分量近似不精确**

```
// 当前：速度分量共用var_v权重
double w_v = 1.0 / (var_v + 1.0);
```

**改进建议**：

```
// 应计算vx,vy在笛卡尔坐标系的实际方差
Eigen::Matrix2d speed_cov = jacobian * P.block<2,2>(2,2) * jacobian.transpose();
```

**问题2：缺乏创新协方差矩阵S**

```
// 标准方法应计算S = HPHᵀ + R，然后求逆
Eigen::Matrix4d S = H * P * H.transpose() + R;
Eigen::Matrix4d S_inv = S.inverse();
double dist = sqrt(y.transpose() * S_inv * y);
```

**问题3：实时性考虑**

- 

  匈牙利算法复杂度O(n³)，目标数量多时可能成为瓶颈

- 

  可先进行门控预筛选，减少代价矩阵规模

### 6. 与系统其他模块的协作

```
// 输入依赖：EKF预测状态
Eigen::Vector4d track_pred = tracks[i].ekf.getCartesianState();
const Eigen::MatrixXd& P = tracks[i].ekf.getCovariance();

// 输出供给：ObjectTracker状态管理
// matches → 更新已关联轨迹
// unmatched_tracks → coast_cycles++ 
// unmatched_detections → 创建新轨迹
```

### 7. 总结

这是一个**工程实用型**的数据关联实现，在**精度**和**稳定性**之间取得了良好平衡。虽然对标准马氏距离进行了简化，但通过加权策略和门控机制，在实际系统中能够提供可靠的关联结果。对于雷达感知这种对实时性要求较高的场景，这种折衷设计是合理的。