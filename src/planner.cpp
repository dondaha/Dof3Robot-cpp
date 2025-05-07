#include "planner.h"
#include <math.h>

Planner::Planner(double L1, double L2, double L3, double x, double y, double r, double q1, double q2, double q3)
{
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->circle_x = x;
    this->circle_y = y;
    this->circle_r = r;
    this->q = std::vector<double>{q1, q2, q3}; // Initialize q with 3 joint angles
}

Planner::~Planner()
{
    // Destructor
}

std::vector<std::vector<double>> Planner::pointsSampler(double step)
{
    std::vector<std::vector<double>> points;
    // 添加第-1个点
    points.push_back(std::vector<double>{L1 * cos(q[0]) + L2 * cos(q[0] + q[1]) + L3 * cos(q[0] + q[1] + q[2]), L1 * sin(q[0]) + L2 * sin(q[0] + q[1]) + L3 * sin(q[0] + q[1] + q[2])});
    // 如果机械臂碰不到圆心直接返回
    if (L1 - L2 - L3 < 0)
    { // 先考虑工作空间为圆
        if (sqrt(pow(circle_x, 2) + pow(circle_y, 2)) > circle_r + L1 + L2 + L3)
        { // 圆心距离大于半径之和，无法到达
            return points;
        }
    }
    else
    { // 工作空间为圆环
      // TODO 暂时不考虑圆在工作空间圆环内部
    }
    // 计算最远可达点
    double alpha = atan2(circle_y, circle_x);             // 计算圆心的角度
    double r = sqrt(pow(circle_x, 2) + pow(circle_y, 2)); // 计算圆心的距离
    if (r + circle_r > L1 + L2 + L3)
    {                                                                                                              // 工作空间与圆的重叠区域为圆弧
        double beta = acos(-(r * r + circle_r * circle_r - (L1 + L2 + L3) * (L1 + L2 + L3)) / (2 * r * circle_r)); // 一个中间变量，代表 圆心->可达最远处 向量与 原点->圆心 向量的夹角
        for (double theta = alpha + beta; theta <= alpha + beta + (2 * M_PI - 2 * beta); theta += step)
        {
            double x = circle_x + circle_r * cos(theta);
            double y = circle_y + circle_r * sin(theta);
            points.push_back({x, y});
        }
    }
    else
    {
        // 工作空间与圆的重叠区域为整个圆
        for (double theta = alpha; theta < alpha + 2 * M_PI; theta += step)
        {
            double x = circle_x + circle_r * cos(theta);
            double y = circle_y + circle_r * sin(theta);
            points.push_back({x, y});
        }
    }
    return points;
}