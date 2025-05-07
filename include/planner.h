#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <vector>

class Planner
{
private:
    double L1, L2, L3; // Lengths of the arms
    double circle_x, circle_y; // Center of the circle
    double circle_r; // Radius of the circle
    std::vector<double> q; // Array to store current joint angles
    int num_points; // Number of points in the trajectory
public:
    Planner(double L1, double L2, double L3, double x, double y, double r, double q1 = 0, double q2 = 0, double q3 = 0);
    ~Planner();
    std::vector<std::vector<double>> planTrajectory();
    std::vector<std::vector<double>> pointsSampler(double step);
};

#endif