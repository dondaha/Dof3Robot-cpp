#include "stdio.h"
#include "visualization.h"
#include "planner.h"
#include "main.h"

int main()
{
    int thread_num = Eigen::nbThreads();
    printf("[Main] Program started, thread num: %d\n", thread_num);
    // 定义机械臂长度
    const double L1 = 110;
    const double L2 = 145;
    const double L3 = 180;
    // 定义圆心坐标和半径
    const double circle_x = 300;
    const double circle_y = 0;
    const double circle_r = 80;
    // 定义障碍物
    std::vector<CircleObstacle> obstacles;
#ifdef USE_CIRCLE_OBSTACLE
    // obstacles.push_back(CircleObstacle(200, 200, 30));  // 障碍物1
    obstacles.push_back(CircleObstacle(400, -100, 40)); // 障碍物2
    obstacles.push_back(CircleObstacle(60, 120, 60)); // 障碍物3
#endif
    // 创建一个可视化模型
    Visualization *vis = new Visualization(L1, L2, L3, circle_x, circle_y, circle_r, obstacles);
    // 创建一个路径规划模型
    Planner *planner = new Planner(L1, L2, L3, circle_x, circle_y, circle_r, obstacles, 1);
    // 规划路径
    std::vector<std::vector<double>> points = planner->pointsSampler(0.1);
    // 规划轨迹
    // std::vector<std::vector<double>> q = planner->planTrajectoryBruteForce(points);
    std::vector<std::vector<double>> q = planner->planTrajectoryOpitmization(points);
    // std::vector<std::vector<double>> q = planner->planTrajectoryNewton(points);
    // std::vector<std::vector<double>> q = planner->planTrajectoryRRTStar(points);
    // 计算轨迹q空间的总长度
    double total_q_length = calculate_total_q_distance(q);
    printf("Total q length: %.2f\n", total_q_length);
    // 可视化
    // vis->testSinusoidalMotion();
    vis->visualize(q);
    printf("Program finished\n");
    return 0;
}