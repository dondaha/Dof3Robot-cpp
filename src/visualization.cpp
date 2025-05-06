#include "visualization.h"
#include <SFML/Graphics.hpp>

using namespace sf;

Visualization::Visualization(double L1, double L2, double L3, double x, double y, double r) {
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->circle_x = x;
    this->circle_y = y;
    this->circle_r = r;
    this->num_points = 0;
    this->q = std::vector<double>(3, 0.0); // Initialize q with 3 joint angles
    // 创建一个 SFML 窗口
    this->window = RenderWindow(VideoMode(Vector2u(Window_Width,Window_Height)), Window_Name);
    this->window.setFramerateLimit(60); // Set the frame rate limit to 60 FPS
    // 初始化形状三根线与圆
    circle = new CircleShape(r, 60); // Create a new circle shape
    circle->setPosition(Vector2f(x,y)); // Set the position of the circle
    circle->setFillColor(Color(255, 255, 255));
    circle->setOutlineThickness(-5.f); // 轮廓从边缘向内凹进
    circle->setOutlineColor(sf::Color(0, 0, 0));
    this->line1 = new RectangleShape(Vector2f(L1, 5)); // Create a new rectangle shape for the first arm
    this->line2 = new RectangleShape(Vector2f(L2, 5)); // Create a new rectangle shape for the second arm
    this->line3 = new RectangleShape(Vector2f(L3, 5)); // Create a new rectangle shape for the third arm
}

Visualization::~Visualization() {
    // Destructor
    
}

void Visualization::drawCircle(double x, double y, double r) {
    this->window.draw(*circle); // Draw the circle
}

void Visualization::visualize(){
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

        // draw everything here...
        // draw the circle
        drawCircle(circle_x, circle_y, circle_r);

        // end the current frame
        window.display();
    }
}