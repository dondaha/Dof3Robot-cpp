#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <vector>
#include <math.h>
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <ompl/config.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <iostream>
#include <Eigen/Sparse> // Include the Eigen Sparse module
#include "main.h"
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define Tolerance 1e-2 // Tolerance for numerical stability
#define Max_Iter 1000  // Maximum number of iterations for optimization
#define Collision_Epsilon 10.0 // 碰撞阈值
#define Collision_K 1.0    // 碰撞惩罚系数

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Point {
    double x;
    double y;
};

struct Circle {
    double x;
    double y;
    double r;
};

// 节点结构体用于图搜索
struct Node {
    int i; // 轨迹点索引
    int j; // theta1索引
    double cost; // 从起点到该节点的代价
    int prev; // 前驱节点索引
    Eigen::Vector3d q; // 关节角度
    
    Node(int i, int j, double cost, int prev, const Eigen::Vector3d& q)
        : i(i), j(j), cost(cost), prev(prev), q(q) {}
    
    // 用于优先队列比较
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};


class Planner
{
private:
    double L1, L2, L3;         // Lengths of the arms
    double circle_x, circle_y; // Center of the circle
    double circle_r;           // Radius of the circle
    Eigen::Vector3d q;         // Array to store current joint angles
    std::vector<CircleObstacle> obstacles; // 存储障碍物的容器
public:
Planner(double L1, double L2, double L3, double x, double y, double r, const std::vector<CircleObstacle>& obs, double q1 = 0, double q2 = 0, double q3 = 0); 
    ~Planner();
    std::vector<std::vector<double>> planTrajectoryNewton(std::vector<std::vector<double>> points);
    std::vector<std::vector<double>> planTrajectoryOpitmization(std::vector<std::vector<double>> points);
    std::vector<std::vector<double>> planTrajectoryRRTStar(std::vector<std::vector<double>> points);
    std::vector<std::vector<double>> planTrajectoryBruteForce(std::vector<std::vector<double>> points);
    std::vector<std::vector<double>> pointsSampler(double step);
    Eigen::Matrix<double, 2, 3> J_matrix(const Eigen::Vector3d &q_val);
    Eigen::Vector2d kinematics(const Eigen::Vector3d &q_val);
    bool Planner::checkCollision(const Eigen::Vector3d& q);
    Eigen::Vector3d ikinematics(const Eigen::Vector2d &p_d, const Eigen::Vector3d &q_initial = Eigen::Vector3d(0, 0, 0));
};


double calculate_total_q_distance(const std::vector<std::vector<double>> &q);
double normalizeAngle(double angle);
std::vector<Point> computeCircleIntersection(const Circle& c1, const Circle& c2);

#endif