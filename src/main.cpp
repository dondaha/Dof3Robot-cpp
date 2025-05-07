#include "stdio.h"
#include "visualization.h"

int main(){
    printf("Hello World\n");
    // 创建一个可视化模型
    Visualization *vis = new Visualization(150, 150, 150, 300, 0, 80);
    // 可视化
    vis->testSinusoidalMotion();
    printf("Program finished\n");
    return 0;
}