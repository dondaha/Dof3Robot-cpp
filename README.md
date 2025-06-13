# 平面RRR机械臂扫圆形路径规划

## 效果展示

![alt text](<效果演示.gif>)

## Requirement

vcpkg安装依赖：

```shell
vcpkg install sfml:x64-windows
vcpkg install ompl:x64-windows
vcpkg install eigen3:x64-windows
```

## 配置

在`main.h`中定义了宏USE_CIRCLE_OBSTACLE，开启即可打开障碍物的生成和避障规划。
