#include "visualization.h"
#include <SFML/Graphics.hpp>

Visualization::Visualization(double L1, double L2, double L3, double x, double y, double r) {
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->x = x;
    this->y = y;
    this->r = r;
    this->num_points = 0;
    this->q = std::vector<double>(3, 0.0); // Initialize q with 3 joint angles
}

Visualization::~Visualization() {
    // Destructor
    
}

void Visualization::drawCircle(double x, double y, double r) {
    // 创建一个 SFML 窗口
    sf::RenderWindow window(sf::VideoMode(800, 600), "Visualization - Circle");

    // 创建一个圆形对象
    sf::CircleShape circle(r);
    circle.setFillColor(sf::Color::Transparent); // 设置为透明填充
    circle.setOutlineColor(sf::Color::Red);      // 设置边框颜色
    circle.setOutlineThickness(2);               // 设置边框厚度

    // 设置圆心位置
    circle.setPosition(x - r, y - r); // SFML 的圆心是左上角，需要调整

    // 主绘图循环
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // 清空窗口
        window.clear(sf::Color::Black);

        // 绘制圆形
        window.draw(circle);

        // 显示内容
        window.display();
    }
}