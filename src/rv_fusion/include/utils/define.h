#ifndef DEFINES_H
#define DEFINES_H

#include <vector>
#include <iostream>
#include <cmath>

typedef double track_t;

// 3. 定义核心容器
// 匈牙利算法需要的两个核心类型
typedef std::vector<int> assignments_t;
typedef std::vector<track_t> distMatrix_t;

#endif // DEFINES_H