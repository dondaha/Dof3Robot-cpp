#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__
// TODO: 写一套可视化的代码，需要接收一个圆的坐标和半径，以及机械臂L1,L2,L3
// TODO: 可视化函数要求接收一个二维数组q，来绘制出机械臂的运动轨迹
// TODO: 还需要一个测试函数，让q1,q2,q3直接正弦变化，测试可视化效果
#include <vector>
#include <SFML/Graphics.hpp>

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
#define Max_FrameRate 60
#define PlayLoop true  // 是否循环播放

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
public:
    Visualization(double L1, double L2, double L3, double x, double y, double r);
    ~Visualization();
    void drawCircle(double x, double y, double r);
    void drawArm(double q1, double q2, double q3);
    void visualize(std::vector<std::vector<double>> q);
    void testSinusoidalMotion();
};

#endif