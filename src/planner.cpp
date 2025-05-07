#include "planner.h"
#include <iostream>


Planner::Planner(double L1, double L2, double L3, double x, double y, double r, double q1, double q2, double q3)
{
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->circle_x = x;
    this->circle_y = y;
    this->circle_r = r;
    this->q = {q1, q2, q3}; // Initialize q with 3 joint angles
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

std::vector<std::vector<double>> Planner::planTrajectoryNewton(std::vector<std::vector<double>> points){
    std::vector<std::vector<double>> trajectory;
    
    for (const auto &point : points)
    {
        Eigen::Vector2d p_d(point[0], point[1]); // 目标点
        Eigen::Vector2d p = kinematics();       // 当前点
        Eigen::Vector2d p_error = p_d - p; // 计算当前点与目标点的误差
        double distance_error = sqrt((p_error).dot(p_error)); // 计算当前点与目标点的距离误差
        if (distance_error < 1e-10)
        { // 如果误差小于容忍度，直接返回
            trajectory.push_back({q[0], q[1], q[2]});
            continue;
        }
        uint64_t iter = 0; // 迭代次数
        // 使用牛顿-拉夫森法求解逆运动学
        // 需要求解方程：J delta_q = p_error 求出 delta_q 那么 q = q + delta_q
        // J 是 3x2 的雅可比矩阵
        while (distance_error > Tolerance && iter < Max_Iter)
        {
            Eigen::Matrix<double, 2, 3> J = J_matrix(); // 雅可比矩阵
            p_error = p_d - p; // 计算当前点与目标点的误差
            // std::cout << "J: " << J << std::endl;
            Eigen::Matrix<double, 3, 1> delta_q = J.completeOrthogonalDecomposition().solve(p_error);
            q += delta_q; // 更新角度
            p = kinematics(); // 更新当前点
            distance_error = (p_error).dot(p_error); // 更新距离误差
            iter++;
        }
        trajectory.push_back({q[0], q[1], q[2]}); // 将当前点添加到轨迹中
    }
    return trajectory;
}

Eigen::Matrix<double, 2, 3> Planner::J_matrix(){
    // 计算雅可比矩阵
    Eigen::Matrix<double, 2, 3> J;
    J(0, 0) = -L1 * sin(q[0]) - L2 * sin(q[0] + q[1]) - L3 * sin(q[0] + q[1] + q[2]);
    J(0, 1) = -L2 * sin(q[0] + q[1]) - L3 * sin(q[0] + q[1] + q[2]);
    J(0, 2) = -L3 * sin(q[0] + q[1] + q[2]);
    J(1, 0) = L1 * cos(q[0]) + L2 * cos(q[0] + q[1]) + L3 * cos(q[0] + q[1] + q[2]);
    J(1, 1) = L2 * cos(q[0] + q[1]) + L3 * cos(q[0] + q[1] + q[2]);
    J(1, 2) = L3 * cos(q[0] + q[1] + q[2]);
    return J;
}

Eigen::Vector2d Planner::kinematics(){
    // 计算正运动学
    Eigen::Vector2d p;
    p(0) = L1 * cos(q[0]) + L2 * cos(q[0] + q[1]) + L3 * cos(q[0] + q[1] + q[2]);
    p(1) = L1 * sin(q[0]) + L2 * sin(q[0] + q[1]) + L3 * sin(q[0] + q[1] + q[2]);
    return p;
}

double calculate_total_q_distance(const std::vector<std::vector<double>> &q)
{
    double total_q_length = 0; // q在状态空间中的轨迹长度
    for (size_t i = 1; i < q.size() - 1; i++)
    {
        double q_length = sqrt(pow(q[i + 1][0] - q[i][0], 2) + pow(q[i + 1][1] - q[i][1], 2) + pow(q[i + 1][2] - q[i][2], 2));
        total_q_length += q_length;
    }
    return total_q_length;
}