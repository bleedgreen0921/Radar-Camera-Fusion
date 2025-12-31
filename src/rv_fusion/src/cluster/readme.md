# DBSCAN聚类代码详解

```cpp
Dbscan::Dbscan(double eps_dist, double eps_vel, int min_pts, bool use_z, bool use_vel)
    : eps_dist_(eps_dist), eps_vel_(eps_vel), min_pts_(min_pts), use_z_(use_z), use_vel_(use_vel) {
    
    tree_.reset(new pcl::search::KdTree<PointType>());
    search_cloud_.reset(new pcl::PointCloud<PointType>());
}
```

| 参数          | 类型           | 默认值 | 作用                       |
| :------------ | :------------- | :----- | -------------------------- |
| eps_dist      | double         | 必需   | 空间邻域半径（米）         |
| eps_vel       | double         | 必需   | 速度相似性阈值（m/s）      |
| min_pts       | int            | 必需   | 形成簇的最小点数           |
| use_z         | bool           | false  | 是否使用Z轴（3D/2D模式）   |
| use_vel       | bool           | false  | 是否使用速度维度           |
| tree_         | KdTree指针     |        | 空间索引结构，加速邻域搜索 |
| search_cloud_ | PointCloud指针 |        | 预处理后的点云（用于搜索） |

- **双重阈值**：同时支持空间距离和速度相似性
- **灵活模式**：通过`use_z`和`use_vel`切换2D/3D、雷达/激光雷达模式
- **智能指针**：使用`reset()`管理动态内存

------

## 数据预处理

```cpp
void Dbscan::setInputCloud(pcl::PointCloud<PointType>::Ptr cloud) {
    input_cloud_ = cloud; // 保存原始点云指针
    if (input_cloud_->empty()) return;

    // 清空内部搜索云
    search_cloud_->clear();
    search_cloud_->points.reserve(input_cloud_->points.size());

    for(const auto& pt : input_cloud_->points){
        PointType pt_internal = pt;

        if(!use_z_){
            pt_internal.z = 0.0f; // 2D模式：拍扁Z轴
        }
        // 如果 use_z_ 为 true (LiDAR模式)，保留原始 Z，KD-Tree 将进行 3D 搜索
        search_cloud_->push_back(pt_internal);
    }
    // 基于处理后的点云重建 KD-Tree
    tree_->setInputCloud(search_cloud_);
}
```

------

## 核心聚类算法

```cpp
void Dbscan::extract(std::vector<pcl::PointIndices>& cluster_indices)
```

- **作用**：执行DBSCAN聚类，识别点云中的簇
- **输入**：`input_cloud_`（通过`setInputCloud`设置）
- **输出**：`cluster_indices`，包含每个簇的点索引列表
- **算法类型**：基于密度的空间聚类

```cpp
开始
  ↓
初始化标签数组（全0）
  ↓
遍历每个点 i
  ↓
if 点i已标记 → 跳过
else ↓
获取点i的邻居列表
  ↓
if 邻居数 < min_pts → 标记为噪声(-1)
else ↓
创建新簇，标记点i为核心点
  ↓
初始化BFS队列 = 点i的邻居
  ↓
while BFS队列不为空
  ├─ 取出点j
  ├─ if 点j是噪声 → 加入当前簇
  ├─ if 点j已处理 → 跳过
  ├─ else ↓
  │   标记点j，加入当前簇
  │   ↓
  │   获取点j的邻居
  │   ↓
  │   if 点j是核心点 → 扩展BFS队列
  └─ 继续循环
  ↓
保存当前簇
  ↓
继续外层循环
```

------

```cpp
// labels 用于标记每个点的状态：
// 0: 未处理 (Unvisited)
// -1: 噪声 (Noise)
// >0: 簇 ID (Cluster ID)
std::vector<int> labels(n_points, 0);
```

### 簇ID管理

```cpp
int cluster_id = 0;
// ...
cluster_id++;
labels[i] = cluster_id;
```

- 初始`cluster_id = 0`
- 发现新簇时先自增：`cluster_id++`，然后分配
- 因此第一个簇的ID是1，第二个是2，依此类推
- 最终簇数量 = `cluster_id`

### 外层循环：遍历所有点

- 顺序遍历所有点
- 通过`labels[i] != 0`跳过已处理的点
- 确保每个点只被处理一次

```cpp
for (int i = 0; i < n_points; ++i) {
        if (labels[i] != 0) continue; // 如果已经归类或者是已知的噪声，跳过

        // 寻找当前点的所有“合格”邻居
        std::vector<int> neighbors;
        getNeighbors(i, neighbors); 

        // 密度判断：如果邻居太少，标记为噪声
        if (neighbors.size() < min_pts_) {
            labels[i] = -1; 
            continue;
        }
```

- **核心点**：邻居数 ≥ `min_pts_`
- **噪声点**：邻居数 < `min_pts_`，且后续没有被任何核心点连接
- **边缘点**：邻居数 < `min_pts_`，但在某个核心点的邻域内

### 内层循环：区域生长（BFS）

```cpp
// --- 发现核心点，开始建立新簇 ---
        cluster_id++;
        labels[i] = cluster_id;

        pcl::PointIndices current_cluster;
        current_cluster.indices.push_back(i);
```

- 分配新簇ID
- 将当前核心点标记为该簇
- 创建簇容器，加入核心点索引

```cpp
for (size_t k = 0; k < neighbors.size(); ++k) {
            int neighbor_idx = neighbors[k];

            // 情况 A: 之前被标记为噪声的点
            // (说明它虽不是核心点，但在当前核心点的邻域内 -> 它是边缘点)
            if (labels[neighbor_idx] == -1) {
                labels[neighbor_idx] = cluster_id; // 归入当前簇
                current_cluster.indices.push_back(neighbor_idx);
            }

            // 情况 B: 已经处理过的点，跳过
            if (labels[neighbor_idx] != 0) continue;

            // 情况 C: 全新的点
            labels[neighbor_idx] = cluster_id; // 归入当前簇
            current_cluster.indices.push_back(neighbor_idx);

            // 检查这个新点是否也是核心点？
            std::vector<int> sub_neighbors;
            getNeighbors(neighbor_idx, sub_neighbors); // 递归式搜索
        }
```

**BFS实现特点**：

- 使用`vector`模拟队列，但通过下标遍历
- 循环中会动态扩展`neighbors`列表
- 这种设计允许在遍历时添加新元素

### 动态扩展机制

```cpp
// 如果它也是核心点，把它发现的邻居加入大部队，继续向外扩
            if (sub_neighbors.size() >= min_pts_) {
                neighbors.insert(neighbors.end(), sub_neighbors.begin(), sub_neighbors.end());
            }
```

- 将新核心点的所有邻居加入`neighbors`列表
- 使用`insert`在末尾添加，避免影响当前遍历
- 新增的邻居会在后续迭代中被处理

**潜在问题**：

1. **重复点**：`sub_neighbors`可能包含已在`neighbors`中的点
2. **内存增长**：`neighbors`可能变得很大
3. **顺序问题**：BFS变成类似DFS，取决于插入位置

## 邻域搜索算法

```cpp
void Dbscan::getNeighbors(int index, std::vector<int>& neighbors)
```

- **输入**：`index`- 查询点在`search_cloud_`中的索引
- **输出**：`neighbors`- 符合条件的邻居点索引列表
- **作用**：找到与查询点在空间和速度上都相似的点

```cpp
空间粗筛 (快速) → 速度精筛 (精确) → 最终邻居
     ↓                  ↓
  KD-Tree搜索     归一化椭球距离
  O(log n)复杂度   O(k)复杂度
```

```cpp
std::vector<int> k_indices;
std::vector<float> k_sqr_distances;
PointType searchPoint = search_cloud_->points[index];
```

**变量说明**：

- `k_indices`：临时存储空间邻居的索引
- `k_sqr_distances`：临时存储到查询点的平方距离
- `searchPoint`：查询点的副本

**内存管理**：

- 局部变量，每次调用创建，函数结束销毁
- 潜在优化：可复用外部传入的容器，减少内存分配

### KD-Tree空间粗筛

```cpp
tree_->radiusSearch(searchPoint, eps_dist_, k_indices, k_sqr_distances);

// PCL KD-Tree 内部实现简化
class KdTree {
public:
    void radiusSearch(const PointType& point, double radius,
                     std::vector<int>& indices, std::vector<float>& distances) {
        // 1. 从根节点开始搜索
        // 2. 递归遍历树，比较坐标轴分割
        // 3. 剪枝：如果点到分割面的距离 > radius，跳过该子树
        // 4. 收集所有距离 < radius 的点
    }
};
```

**搜索半径说明**：

- 如果`use_z_ = false`：搜索2D圆盘半径

  ```cpp
  // 实际距离公式
  distance = sqrt((x1-x2)² + (y1-y2)²)
  ```

- 如果`use_z_ = true`：搜索3D球体半径

  ```cpp
  // 实际距离公式
  distance = sqrt((x1-x2)² + (y1-y2)² + (z1-z2)²)
  ```

**计算优化**：

```cpp
// KD-Tree返回的是平方距离，避免开方运算
// 比较时：dist_sqr < eps_dist_ * eps_dist_
// 而不是：sqrt(dist_sqr) < eps_dist_
```

**性能优势**：

- 避免开方运算，CPU周期节约约10-20倍
- 比较平方值，计算更快

------

### 速度精筛

```cpp
neighbors.clear();
neighbors.reserve(k_indices.size());
```

**内存预分配优化**：

- 知道空间邻居的最大数量`k_indices.size()`
- 预分配内存，避免vector动态扩容
- 减少内存碎片和分配开销

```cpp
if(use_vel_){
            // 假设 intensity 存储速度，进行加权距离计算
            float vel_diff = std::abs(search_cloud_->points[idx].intensity - searchPoint.intensity);

            // 归一化椭球距离公式
            float normalized_dist = (dist_sqr / (eps_dist_ * eps_dist_)) + 
                                    (vel_diff * vel_diff) / (eps_vel_ * eps_vel_);

            if (normalized_dist <= 1.0f) {
                neighbors.push_back(idx);
            }
        }
        else{
            // --- LiDAR 模式 (纯空间聚类) ---
            // 直接接受 KD-Tree 的结果 (因为 KD-Tree 已经保证了 dist < eps_dist)
            // 此时忽略 intensity (反射率)，防止把同一物体不同反射率的部分切开
            neighbors.push_back(idx);
        }
```

------

# 雷达聚类节点代码详解