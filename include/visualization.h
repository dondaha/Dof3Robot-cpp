#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <vector>
#include <SFML/Graphics.hpp>
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define Window_Name "Visualization"
#define Window_Width 1200
#define Window_Height 800
#define circle_thickness 5.f // 圆的边框厚度
#define line_thickness 5.f   // 线的宽度
#define World_Center_X 300
#define World_Center_Y 400
#define Line1_Color sf::Color(240, 178, 122)
#define Line2_Color sf::Color(69, 179, 157)
#define Line3_Color sf::Color(236, 112, 99)
#define Circle_Color sf::Color(170, 183, 184)
#define Max_FrameRate 30
#define PlayLoop false // 是否循环播放

class Visualization
{
private:
    double L1, L2, L3;                         // Lengths of the arms
    double circle_x, circle_y;                 // Center of the circle
    double circle_r;                           // Radius of the circle
    std::vector<double> q;                     // Array to store current joint angles
    int num_points;                            // Number of points in the trajectory
    sf::RenderWindow window;                   // SFML window for visualization
    sf::CircleShape *circle;                   // Circle shape for drawing
    sf::RectangleShape *line1, *line2, *line3; // Line shapes for drawing the arms
    std::vector<CircleObstacle> obstacles; // 存储障碍物的容器
public:
    Visualization(double L1, double L2, double L3, double x, double y, double r, const std::vector<CircleObstacle> obstacles);
    ~Visualization();
    void drawCircle(double x, double y, double r);
    void drawArm(double q1, double q2, double q3);
    void drawInfo(double total_q_length, double q1, double q2, double q3);
    void Visualization::drawCollisionCircles(double x0, double y0, double angle, double length, sf::Color color);
    void visualize(const std::vector<std::vector<double>> &q);
    void testSinusoidalMotion();
    void Visualization::drawObs(); // Draw obstacles in the visualization
};

#endif