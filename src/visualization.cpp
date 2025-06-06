#include "visualization.h"
#include <SFML/Graphics.hpp>
#include <math.h>

using namespace sf;

Visualization::Visualization(double L1, double L2, double L3, double x, double y, double r)
{
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->circle_x = World_Center_X + x;
    this->circle_y = World_Center_Y + y;
    this->circle_r = r;
    this->num_points = 0;
    this->q = std::vector<double>{M_PI / 6, M_PI / 6, -M_PI / 4}; // Initialize q with 3 joint angles
    // 创建一个 SFML 窗口
    this->window = RenderWindow(VideoMode(Vector2u(Window_Width, Window_Height)), Window_Name);
    this->window.setFramerateLimit(Max_FrameRate); // Set the frame rate limit to 60 FPS
    // 初始化形状三根线与圆
    circle = new CircleShape(r, 360);  // Create a new circle shape
    circle->setOrigin(Vector2f(r, r)); // Set the origin of the circle to its center
    circle->setFillColor(Color(255, 255, 255));
    circle->setOutlineThickness(-circle_thickness); // 轮廓从边缘向内凹进
    circle->setOutlineColor(Circle_Color);
    this->line1 = new RectangleShape(Vector2f(L1, line_thickness)); // Create a new rectangle shape for the first arm
    this->line2 = new RectangleShape(Vector2f(L2, line_thickness)); // Create a new rectangle shape for the second arm
    this->line3 = new RectangleShape(Vector2f(L3, line_thickness)); // Create a new rectangle shape for the third arm
    this->line1->setOrigin(Vector2f(0, line_thickness / 2));        // Set the origin of the first arm to its left edge
    this->line2->setOrigin(Vector2f(0, line_thickness / 2));        // Set the origin of the second arm to its left edge
    this->line3->setOrigin(Vector2f(0, line_thickness / 2));        // Set the origin of the third arm to its left edge
    this->line1->setFillColor(Line1_Color);                         // Set the color of the first arm to black
    this->line2->setFillColor(Line2_Color);                         // Set the color of the second arm to black
    this->line3->setFillColor(Line3_Color);                         // Set the color of the third arm to black
}

Visualization::~Visualization()
{
    // Destructor
}

void Visualization::drawCircle(double x, double y, double r)
{
    circle->setPosition(Vector2f(x, y)); // Set the position of the circle
    circle->setRadius(r);                // Set the radius of the circle
    this->window.draw(*circle);          // Draw the circle
}

void Visualization::drawArm(double q1, double q2, double q3)
{
    // 先计算每个关节的坐标
    double x0 = World_Center_X; // 第一根机械臂的起点
    double y0 = World_Center_Y;
    double x1 = x0 + L1 * cos(q1);
    double y1 = y0 + L1 * sin(q1);
    double x2 = x1 + L2 * cos(q1 + q2);
    double y2 = y1 + L2 * sin(q1 + q2);
    // 设置每根机械臂的角度
    this->line1->setRotation(radians(q1));           // Convert radians to degrees
    this->line2->setRotation(radians(q1 + q2));      // Convert radians to degrees
    this->line3->setRotation(radians(q1 + q2 + q3)); // Convert radians to degrees
    // 设置每根机械臂的坐标
    this->line1->setPosition(Vector2f(x0, y0)); // Set the position of the first arm to (0, 0)
    this->line2->setPosition(Vector2f(x1, y1)); // Set the position of the second arm to (0, 0)
    this->line3->setPosition(Vector2f(x2, y2)); // Set the position of the third arm to (0, 0)
    // 画出每根机械臂
    this->window.draw(*line1); // Draw the first arm
    this->window.draw(*line2); // Draw the second arm
    this->window.draw(*line3); // Draw the third arm
}

void Visualization::drawInfo(double total_q_length, double q1, double q2, double q3)
{
    // Create font (using default for simplicity)
    static sf::Font font;
    if (!font.openFromFile("D:/Projects/Dof3Robot-cpp/HarmonyOS_Sans_Black.ttf"))
    {
        printf("[Visualization] Error loading font\n");
    }

    // Convert radians to degrees for display
    auto radToDeg = [](double rad)
    { return rad * 180.0 / M_PI; };

    // Create text objects
    sf::Text q1Text(font);
    sf::Text q2Text(font);
    sf::Text q3Text(font);
    sf::Text totalText(font);
    q1Text.setCharacterSize(16);
    q2Text.setCharacterSize(16);
    q3Text.setCharacterSize(16);
    totalText.setCharacterSize(16);

    // Position texts in top-left corner
    q1Text.setPosition(Vector2f(10, 10));
    q2Text.setPosition(Vector2f(10, 40));
    q3Text.setPosition(Vector2f(10, 70));
    totalText.setPosition(Vector2f(10, 100));

    // Set text colors
    q1Text.setFillColor(sf::Color::Black);
    q2Text.setFillColor(sf::Color::Black);
    q3Text.setFillColor(sf::Color::Black);
    totalText.setFillColor(sf::Color::Black);

    // Update text content
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "q1: %.2f rads, %.2f degrees", q1, radToDeg(q1));
    q1Text.setString(buffer);

    snprintf(buffer, sizeof(buffer), "q2: %.2f rads, %.2f degrees", q2, radToDeg(q2));
    q2Text.setString(buffer);

    snprintf(buffer, sizeof(buffer), "q3: %.2f rads, %.2f degrees", q3, radToDeg(q3));
    q3Text.setString(buffer);

    snprintf(buffer, sizeof(buffer), "Total q length: %.2f", total_q_length);
    totalText.setString(buffer);

    // Draw the texts
    window.draw(q1Text);
    window.draw(q2Text);
    window.draw(q3Text);
    window.draw(totalText);
}

void Visualization::visualize(const std::vector<std::vector<double>> &q)
{
    uint64_t total_frames = q.size(); // 帧数
    uint64_t frames = 0;              // 帧数
    double total_q_length = 0.0;
    while (this->window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        while (const std::optional event = window.pollEvent())
        {
            // "close requested" event: we close the window
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // clear the window with black color
        window.clear(sf::Color::White);

        // draw the circle
        drawCircle(circle_x, circle_y, circle_r);
        // draw the arms
        drawArm(q[frames][0], q[frames][1], q[frames][2]);
        // draw the info text
        drawInfo(total_q_length, q[frames][0], q[frames][1], q[frames][2]);
        if (frames < total_frames - 1)
        {
            // 计算当前q的总长度
            total_q_length += sqrt(pow(q[frames+1][0] - q[frames][0], 2) +
                                   pow(q[frames+1][1] - q[frames][1], 2) +
                                   pow(q[frames+1][2] - q[frames][2], 2));
            frames++;
        }
        else if (PlayLoop)
        {
            frames = 0; // Reset frames to loop the animation
            total_q_length = 0.0;
        }
        // end the current frame
        window.display();
    }
}

void Visualization::testSinusoidalMotion()
{
    uint64_t frames = 0; // 帧数
    std::vector<std::vector<double>> q;
    while (frames < Max_FrameRate * 10)
    {
        // draw the arms
        this->q[0] = M_PI / 6 * sin(2.0 * frames / Max_FrameRate); // q1
        this->q[1] = M_PI / 6 * sin(3.0 * frames / Max_FrameRate); // q2
        this->q[2] = M_PI / 4 * sin(4.0 * frames / Max_FrameRate); // q3
        q.push_back(this->q);
        frames++;
    }
    // 调用函数进行可视化
    visualize(q);
}
