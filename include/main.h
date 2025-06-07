#ifndef __MAIN_H__
#define __MAIN_H__

#define USE_CIRCLE_OBSTACLE // Define if you want to use circle obstacle avoidance

class CircleObstacle {
public:
    double x; // Circle center x-coordinate
    double y; // Circle center y-coordinate
    double r; // Circle radius
    CircleObstacle(double x, double y, double r) : x(x), y(y), r(r) {}

    // Check if a point is inside the circle
    bool contains(double px, double py) const {
        return (px - x) * (px - x) + (py - y) * (py - y) < r * r;
    }
};

#endif // __MAIN_H__