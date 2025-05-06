#include "stdio.h"
#include "visualization.h"

int main(){
    printf("Hello World\n");
    // 创建一个可视化模型
    Visualization *vis = new Visualization(100, 100, 100, 200, 200, 40);
    // 可视化
    vis->visualize();
    printf("Program finished\n");
    return 0;
}