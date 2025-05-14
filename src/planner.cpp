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
        { // 如果误差小于容忍度，直接返回（不优化第一个点的位置，因为这是机械臂的初始位置）
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

std::vector<std::vector<double>> Planner::planTrajectoryOpitmization(std::vector<std::vector<double>> points) {
    int num_points = points.size();
    // 使用优化方法，梯度下降
    // eta = c + k * d 为优化变量，c是每个q对应的末端坐标与其对应目标的距离的平方和，d是q空间路径长度，k是权重
    // 先使用牛顿法求解初始值
    std::vector<std::vector<double>> qp = planTrajectoryNewton(points); // q是类的私有变量Eigen::Vector3d，qp第i行代表第i个点对应的q^T
    std::vector<std::vector<double>> dq(qp.size(), std::vector<double>(3, 0)); // dq是每个点的增量
    uint64_t iter = 0; // 迭代次数
    while (iter<Max_Iter){
        for (int i = 1; i < num_points; i++)
        {
            q[0] = qp[i][0];
            q[1] = qp[i][1];
            q[2] = qp[i][2];
            auto fqi = kinematics();
            Eigen::Vector3d c = 2 * J_matrix().transpose() * (fqi - Eigen::Vector2d(points[i][0], points[i][1])); // 与目标点的距离的梯度
            Eigen::Vector3d d;
            if (i == 1){
                d = Eigen::Vector3d(2*(qp[i][0]-qp[i+1][0]), 2*(qp[i][1]-qp[i+1][1]), 2*(qp[i][2]-qp[i+1][2]));
            }
            else if (i>1&&i<num_points-1)
            {
                Eigen::Vector3d qi = {qp[i][0], qp[i][1], qp[i][2]};
                Eigen::Vector3d qi_rear = {qp[i-1][0], qp[i-1][1], qp[i-1][2]};
                Eigen::Vector3d qi_front = {qp[i+1][0], qp[i+1][1], qp[i+1][2]};
                d = 2*(qi-qi_rear) + 2*(qi-qi_front);
            }
            else{
                d = Eigen::Vector3d(2*(qp[num_points-1][0]-qp[num_points-2][0]), 2*(qp[num_points-1][1]-qp[num_points-2][1]), 2*(qp[num_points-1][2]-qp[num_points-2][2]));
            }
            // 更新dq
            double k = 1.0; // d所占的权重
            dq[i][0] = c[0] + k*d[0];
            dq[i][1] = c[1] + k*d[1];
            dq[i][2] = c[2] + k*d[2];
        }
        // 更新qp
        double lr = 0.001;
        for (int i = 0; i < num_points; i++)
        {
            qp[i][0] -= lr*dq[i][0];
            qp[i][1] -= lr*dq[i][1];
            qp[i][2] -= lr*dq[i][2];
        }
        printf("iter: %d, qp: %.2f %.2f %.2f\n", iter, qp[0][0], qp[0][1], qp[0][2]);
        iter++;
    }
    return qp;
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