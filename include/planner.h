#ifndef __PLANNER_H__
#define __PLANNER_H__

class Planner
{
public:
    Planner(double L1, double L2, double L3, double x, double y, double r);
    ~Planner();
    void planTrajectory();
    void pointsSampler();
};

#endif