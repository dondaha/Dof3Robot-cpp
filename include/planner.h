#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <vector>
#include <math.h>
#include <Eigen/Dense>

#define Tolerance 1e-2 // Tolerance for numerical stability
#define Max_Iter 10000 // Maximum number of iterations for optimization

class Planner
{
private:
    double L1, L2, L3; // Lengths of the arms
    double circle_x, circle_y; // Center of the circle
    double circle_r; // Radius of the circle
    Eigen::Vector3d q; // Array to store current joint angles
    int num_points; // Number of points in the trajectory
public:
    Planner(double L1, double L2, double L3, double x, double y, double r, double q1 = 0, double q2 = 0, double q3 = 0);
    ~Planner();
    std::vector<std::vector<double>> planTrajectoryNewton(std::vector<std::vector<double>> points); // 逐点牛顿法迭代q
    std::vector<std::vector<double>> planTrajectorySampling(std::vector<std::vector<double>> points); // 基于采样的方法
    std::vector<std::vector<double>> planTrajectoryOptimization(std::vector<std::vector<double>> points); // 基于优化的方法
    std::vector<std::vector<double>> planTrajectoryAnalytical(std::vector<std::vector<double>> points); // 基于解析的方法
    std::vector<std::vector<double>> pointsSampler(double step);
    Eigen::Matrix<double, 2, 3> J_matrix();
    Eigen::Vector2d kinematics();
};

double calculate_total_q_distance(const std::vector<std::vector<double>> &q);

#endif