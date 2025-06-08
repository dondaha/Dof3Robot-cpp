#include "planner.h"

Planner::Planner(double L1, double L2, double L3, double x, double y, double r, const std::vector<CircleObstacle> &obs, double q1, double q2, double q3)
    : L1(L1), L2(L2), L3(L3), circle_x(x), circle_y(y), circle_r(r), obstacles(obs) // 初始化障碍物列表
{
    this->q = {q1, q2, q3}; // 初始化关节角
}

Planner::~Planner()
{
    // Destructor
}

std::vector<std::vector<double>> Planner::pointsSampler(double step)
{
    std::vector<std::vector<double>> points;
    // 如果机械臂碰不到圆心直接返回
    if (L1 - L2 - L3 < 0)
    { // 先考虑工作空间为圆
        if (sqrt(pow(circle_x, 2) + pow(circle_y, 2)) > circle_r + L1 + L2 + L3)
        { // 圆心距离大于半径之和，无法到达
            return points;
        }
    }
    else
    { // 工作空间为圆环
      // TODO 暂时不考虑圆在工作空间圆环内部
    }
    // 计算最远可达点
    double alpha = atan2(circle_y, circle_x);             // 计算圆心的角度
    double r = sqrt(pow(circle_x, 2) + pow(circle_y, 2)); // 计算圆心的距离
    if (r + circle_r > L1 + L2 + L3)
    {                                                                                                              // 工作空间与圆的重叠区域为圆弧
        double beta = acos(-(r * r + circle_r * circle_r - (L1 + L2 + L3) * (L1 + L2 + L3)) / (2 * r * circle_r)); // 一个中间变量，代表 圆心->可达最远处 向量与 原点->圆心 向量的夹角
        for (double theta = alpha + beta; theta <= alpha + beta + (2 * M_PI - 2 * beta); theta += step)
        {
            double x = circle_x + circle_r * cos(theta);
            double y = circle_y + circle_r * sin(theta);
            points.push_back({x, y});
        }
    }
    else
    {
        // 工作空间与圆的重叠区域为整个圆
        for (double theta = alpha; theta < alpha + 2 * M_PI; theta += step)
        {
            double x = circle_x + circle_r * cos(theta);
            double y = circle_y + circle_r * sin(theta);
            points.push_back({x, y});
        }
    }
    return points;
}

std::vector<std::vector<double>> Planner::planTrajectoryNewton(std::vector<std::vector<double>> points)
{
    std::vector<std::vector<double>> trajectory;

    for (const auto &point : points)
    {
        Eigen::Vector2d p_d(point[0], point[1]);     // 目标点
        auto ik = ikinematics(p_d, q);               // 使用逆运动学求解当前点的关节角
        q = ik;                                      // 更新当前关节角
        trajectory.push_back({ik[0], ik[1], ik[2]}); // 将当前点添加到轨迹中
    }
    return trajectory;
}

std::vector<std::vector<double>> Planner::planTrajectoryOpitmization(std::vector<std::vector<double>> points)
{
    // 使用优化方法，min{1/2*dQ^T*H*dQ + g^T*dQ}，Q的大小为3(N-1)x1，由q2,q3,...,q_N组成。H为hessian矩阵大小3(n-1)x3(n-1)，g为梯度向量大小3(n-1)x1
    // 约束 J*dQ = -C ，其中C_i = F(q_i) - p_i 大小为2x1，代表x,y方向的误差，C的大小为2(N-1)x1
    // KKT方程组：
    // [H  J^T]   [  dQ  ]   [ -g ]
    // [J   0 ] X [lambda] = [ -C ]
    // H的主对角线为4I_3的块，主对角线最后一个块为2I_3，次对角线全为-2I_3的块，为稀疏矩阵
    // 每一次更新中，J为2(N-1)x3(N-1)的矩阵，由对角上的2x3雅可比矩阵J_i组成。
    // 每一次更新中，g为3(N-1)x1的梯度向量，由2(2q_2-q_1-q_3),2(q_3-q_2-q_4),...,2(2q_{N-1}-q_{N-2}-q_N),2(q_N-q_{N-1})组成。
    // 使用eigen的SimplicialLDLT求解KKT方程组
    printf("[Planner] Use planner: Opimization\n");
    int64_t num_points = static_cast<int64_t>(points.size());
    if (num_points <= 1)
        return {std::vector<double>{q[0], q[1], q[2]}};

    // 初始化优化变量 Q = [q2, q3, ..., qN]
    std::vector<Eigen::Vector3d> Q_initial;
    Eigen::Vector3d q_current = q;
    Q_initial.reserve(num_points - 1);
    for (int i = 0; i < num_points; ++i)
    {
        // 用牛顿法获取初始猜测
        Eigen::Vector2d p_target(points[i][0], points[i][1]);
        for (int iter = 0; iter < Max_Iter; ++iter)
        {
            Eigen::Vector2d p = kinematics(q_current);
            Eigen::Vector2d error = p_target - p;
            if (error.norm() < Tolerance)
                break;
            Eigen::Matrix<double, 2, 3> J = J_matrix(q_current);
            q_current += J.completeOrthogonalDecomposition().solve(error);
        }
        if (i == 0)
        {
            q[0] = q_current[0];
            q[1] = q_current[1];
            q[2] = q_current[2];
        }
        else
        {
            Q_initial.push_back(q_current);
        }
    }

    // 转换为Eigen向量
    Eigen::VectorXd Q(3 * (num_points - 1));
    for (size_t i = 0; i < Q_initial.size(); ++i)
        Q.segment<3>(3 * i) = Q_initial[i];

    // 优化迭代
    for (int opt_iter = 0; opt_iter < Max_Iter; ++opt_iter)
    {
        // 1. 构建Hessian矩阵 (稀疏)
        Eigen::SparseMatrix<double> H(3 * (num_points - 1), 3 * (num_points - 1));
        std::vector<Eigen::Triplet<double>> triplets_H;
        for (int i = 0; i < num_points - 1; ++i)
        {
            // 主对角块
            double diag_val = (i == num_points - 2) ? 2.0 : 4.0;
            for (int j = 0; j < 3; ++j)
                triplets_H.emplace_back(3 * i + j, 3 * i + j, diag_val);

            // 非对角块
            if (i < num_points - 2)
            {
                for (int j = 0; j < 3; ++j)
                {
                    triplets_H.emplace_back(3 * i + j, 3 * (i + 1) + j, -2.0);
                    triplets_H.emplace_back(3 * (i + 1) + j, 3 * i + j, -2.0);
                }
            }
        }
        H.setFromTriplets(triplets_H.begin(), triplets_H.end());

        // 2. 计算梯度g
        Eigen::VectorXd g = Eigen::VectorXd::Zero(3 * (num_points - 1));
        for (int i = 0; i < num_points - 1; ++i)
        {
            Eigen::Vector3d q_i = Q.segment<3>(3 * i);
            Eigen::Vector3d q_prev = (i == 0) ? q : Q.segment<3>(3 * (i - 1));
            Eigen::Vector3d q_next = (i == num_points - 2) ? Eigen::Vector3d(Eigen::Vector3d::Zero()) : Q.segment<3>(3 * (i + 1));
            if (i == num_points - 2)
            {
                g.segment<3>(3 * i) = 2.0 * (q_i - q_prev);
            }
            else
            {
                g.segment<3>(3 * i) = 2.0 * ((2.0 * q_i) - q_prev - q_next);
            }
        }
#ifdef USE_CIRCLE_OBSTACLE
        // 2.1 计算碰撞代价梯度并添加到 g
        Eigen::VectorXd g_collision = Eigen::VectorXd::Zero(g.size());
        for (int i = 0; i < num_points - 1; ++i)
        {
            Eigen::Vector3d q_i = Q.segment<3>(3 * i);
            double k_list[3] = {1.0 / 6.0, 1.0 / 2.0, 5.0 / 6.0}; // 定义连杆上三个碰撞圆的相对位置（比例）

            // 检查第一个连杆
            for (int j = 0; j < 3; ++j)
            {
                double d = k_list[j] * L1; // 距离关节1的长度
                Eigen::Vector2d center(d * cos(q_i[0]), d * sin(q_i[0]));
                double radius = L1 / 6.0; // 碰撞圆半径

                for (const auto &obstacle : obstacles)
                {
                    double dx = center.x() - obstacle.x;
                    double dy = center.y() - obstacle.y;
                    double dist = sqrt(dx * dx + dy * dy);
                    double min_dist = dist - (radius + obstacle.r);

                    // 限制min_dist，避免过大的负值
                    min_dist = std::max(min_dist, -Collision_Epsilon);

                    if (min_dist < Collision_Epsilon)
                    {
                        Eigen::Vector2d dir(dx / dist, dy / dist); // 单位方向向量（从障碍物指向碰撞圆中心）
                        Eigen::Matrix<double, 2, 3> Jc = Eigen::Matrix<double, 2, 3>::Zero();
                        Jc(0, 0) = -d * sin(q_i[0]);
                        Jc(1, 0) = d * cos(q_i[0]);

                        // 计算梯度: ∇c = k * min_dist * (Jcᵀ * dir)
                        Eigen::Vector3d grad = Collision_K * min_dist * (Jc.transpose() * dir);
                        g_collision.segment<3>(3 * i) += grad;
                    }
                }
            }
        }
        g += g_collision; // 将碰撞梯度添加到目标函数的梯度
#endif

        // 3. 构建雅可比矩阵J和残差c
        Eigen::SparseMatrix<double> J(2 * (num_points - 1), 3 * (num_points - 1));
        Eigen::VectorXd c(2 * (num_points - 1));
        std::vector<Eigen::Triplet<double>> triplets_J;
        for (int i = 0; i < num_points - 1; ++i)
        {
            Eigen::Vector3d q_i = Q.segment<3>(3 * i);
            Eigen::Matrix<double, 2, 3> Ji = J_matrix(q_i);
            for (int row = 0; row < 2; ++row)
                for (int col = 0; col < 3; ++col)
                    triplets_J.emplace_back(2 * i + row, 3 * i + col, Ji(row, col));

            Eigen::Vector2d p_i = kinematics(q_i);
            c.segment<2>(2 * i) = p_i - Eigen::Map<Eigen::Vector2d>(points[i + 1].data());
        }
        J.setFromTriplets(triplets_J.begin(), triplets_J.end());

        // 4. 构建KKT系统
        Eigen::SparseMatrix<double> KKT(3 * (num_points - 1) + 2 * (num_points - 1), 3 * (num_points - 1) + 2 * (num_points - 1));
        std::vector<Eigen::Triplet<double>> triplets_KKT;

        // 填入H
        for (int i = 0; i < H.outerSize(); ++i)
            for (Eigen::SparseMatrix<double>::InnerIterator it(H, i); it; ++it)
                triplets_KKT.emplace_back(it.row(), it.col(), it.value());

        // 填入J和J^T
        for (int i = 0; i < J.outerSize(); ++i)
            for (Eigen::SparseMatrix<double>::InnerIterator it(J, i); it; ++it)
            {
                triplets_KKT.emplace_back(3 * (num_points - 1) + it.row(), it.col(), it.value());
                triplets_KKT.emplace_back(it.col(), 3 * (num_points - 1) + it.row(), it.value());
            }

        KKT.setFromTriplets(triplets_KKT.begin(), triplets_KKT.end());

        // 5. 构建右侧向量
        Eigen::VectorXd rhs(3 * (num_points - 1) + 2 * (num_points - 1));
        rhs.head(3 * (num_points - 1)) = -g;
        rhs.tail(2 * (num_points - 1)) = -c;

        // 6. 求解
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(KKT);
        if (solver.info() != Eigen::Success)
        {
            printf("[Planner] KKT matrix decomposition failed!\n");
            break;
        }
        Eigen::VectorXd delta = solver.solve(rhs);

        // 7. 搜索方向上的步长

        // 7.1 固定步长
        // double alpha = 0.01; // 固定步长

        // 7.2 Diminishing step size
        // double alpha = 2.0/(opt_iter+1.0); // 步长

        // 7.3 使用Backtracking/Armijo 线搜索
        // 算法说明
        // 1. 计算当前点的函数值f(x^k)
        // 2. 计算将要迭代到的点的函数值f(x^k+alpha*delta)
        // 3. 计算Armijo条件：f(x^k+alpha*delta) - f(x^k) > c * (alpha*delta)^T * gradiant 即目标点需要在一根线下面
        // 4. 如果不满足条件，收缩步长alpha *= beta
        // 5. 直到满足条件
        // 在这里gradiant就是delta归一化之后的值，与前面的delta^T相乘即为1，因此为f(x^k+alpha*delta) - f(x^k) > c * alpha
        double alpha = 1.0;        // 初始步长
        double beta = 0.5;         // 收缩因子
        double armijo_error = 0.0; // 初始的f(x^k+alpha*delta) - f(x^k)

        double f_current = 0;
        double f_new = 0.5 * alpha * delta.head(3 * (num_points - 1)).dot(H * alpha * delta.head(3 * (num_points - 1))) + g.dot(alpha * delta.head(3 * (num_points - 1)));
        armijo_error = f_new - f_current;
        while (armijo_error > 0.5 * alpha)
        {
            alpha *= beta; // 收缩步长
            f_new = 0.5 * alpha * delta.head(3 * (num_points - 1)).dot(H * alpha * delta.head(3 * (num_points - 1))) + g.dot(alpha * delta.head(3 * (num_points - 1)));
            armijo_error = f_new - f_current;
        }

        // 8. 更新Q
        Q += alpha * delta.head(3 * (num_points - 1));

        printf("Iteration %d: Delta Q Norm: %.6f\n", opt_iter, delta.head(3 * (num_points - 1)).norm());

        // 检查收敛
        if (delta.head(3 * (num_points - 1)).norm() < Tolerance)
            break;
    }

    // 转换为轨迹
    std::vector<std::vector<double>> trajectory;
    trajectory.push_back({q[0], q[1], q[2]}); // q1
    for (int i = 0; i < num_points - 1; ++i)
    {
        Eigen::Vector3d qi = Q.segment<3>(3 * i);
        trajectory.push_back({qi[0], qi[1], qi[2]});
    }
    return trajectory;
}

std::vector<std::vector<double>> Planner::planTrajectoryBruteForce(std::vector<std::vector<double>> points)
{
    // 参数设置
    const int M = 360;                                           // theta1离散数量
    const int delta_j = 1;                                       // 相邻点theta1索引最大变化量
    const double max_angle_step_per_joint = 10.0 * M_PI / 180.0; // 最大关节角度变化（弧度）

    int N = points.size(); // 轨迹点数量
    if (N == 0)
        return {};

    // 存储所有节点
    std::vector<Node> nodes;
    // 存储每个轨迹点的节点索引范围 [start_index, end_index)
    std::vector<std::pair<int, int>> node_ranges(N);

    // 1. 生成网格节点（考虑碰撞检测）
    for (int i = 0; i < N; i++)
    {
        const double x_d = points[i][0];
        const double y_d = points[i][1];
        int start_index = nodes.size();

        for (int j = 0; j < M; j++)
        {
            double theta1 = -M_PI + j * (2 * M_PI / M);

            // 计算第一个关节位置
            double x1 = L1 * cos(theta1);
            double y1 = L1 * sin(theta1);

            // 计算从第一个关节到目标点的向量
            double dx = x_d - x1;
            double dy = y_d - y1;
            double d = sqrt(dx * dx + dy * dy);

            // 检查是否可达
            if (d > L2 + L3 || d < fabs(L2 - L3))
            {
                continue;
            }
            // 计算半径为L2和L3的圆的两个交点
            Circle c1 = {x1, y1, L2};
            Circle c2 = {x_d, y_d, L3};
            double theta2_1, theta2_2, theta3_1, theta3_2;
            try
            {
                auto intersections = computeCircleIntersection(c1, c2); // 实际为第三个关节的位置
                theta2_1 = atan2(intersections[0].y - y1, intersections[0].x - x1) - theta1;
                theta2_2 = atan2(intersections[1].y - y1, intersections[1].x - x1) - theta1;
                theta3_1 = atan2(y_d - intersections[0].y, x_d - intersections[0].x) - atan2(intersections[0].y - y1, intersections[0].x - x1);
                theta3_2 = atan2(y_d - intersections[1].y, x_d - intersections[1].x) - atan2(intersections[1].y - y1, intersections[1].x - x1);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error: " << e.what() << std::endl;
                continue;
            }

            // 创建两种可能的配置
            Eigen::Vector3d q1(theta1, theta2_1, theta3_1);
            Eigen::Vector3d q2(theta1, theta2_2, theta3_2);

            // 检查碰撞并添加节点
            if (!checkCollision(q1))
            {
                nodes.emplace_back(i, j, std::numeric_limits<double>::max(), -1, q1);
            }
            if (!checkCollision(q2))
            {
                nodes.emplace_back(i, j, std::numeric_limits<double>::max(), -1, q2);
            }
        }

        node_ranges[i] = {start_index, static_cast<int>(nodes.size())};

        // 如果没有可行节点，返回空路径
        if (node_ranges[i].first == node_ranges[i].second)
        {
            return {};
        }
    }

    // 2. 初始化优先队列（最小堆）
    auto cmp = [](const Node *a, const Node *b)
    { return a->cost > b->cost; };
    std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> pq(cmp);

    // 3. 设置起点（第一个轨迹点的所有节点）
    for (int idx = node_ranges[0].first; idx < node_ranges[0].second; idx++)
    {
        nodes[idx].cost = 0.0;
        pq.push(&nodes[idx]);
    }

    // 4. Dijkstra算法
    Node *end_node = nullptr;
    double min_end_cost = std::numeric_limits<double>::max();

    while (!pq.empty())
    {
        Node *u = pq.top();
        pq.pop();

        // 如果当前节点是最后一个轨迹点
        if (u->i == N - 1)
        {
            if (u->cost < min_end_cost)
            {
                min_end_cost = u->cost;
                end_node = u;
            }
            continue;
        }

        // 处理下一个轨迹点
        int next_i = u->i + 1;
        for (int v_idx = node_ranges[next_i].first; v_idx < node_ranges[next_i].second; v_idx++)
        {
            Node *v = &nodes[v_idx];

            // 检查theta1索引差（考虑循环）
            int dj = abs(u->j - v->j);
            if (dj > M / 2)
                dj = M - dj;
            if (dj > delta_j)
                continue;

            // 计算关节角度变化
            double dq0 = normalizeAngle(u->q[0] - v->q[0]);
            double dq1 = normalizeAngle(u->q[1] - v->q[1]);
            double dq2 = normalizeAngle(u->q[2] - v->q[2]);

            // 检查关节角度变化是否在允许范围内
            if (fabs(dq0) > max_angle_step_per_joint ||
                fabs(dq1) > max_angle_step_per_joint ||
                fabs(dq2) > max_angle_step_per_joint)
            {
                continue;
            }

            // 计算边权（欧氏距离）
            double dist = sqrt(dq0 * dq0 + dq1 * dq1 + dq2 * dq2);
            double new_cost = u->cost + dist;

            // 更新节点代价
            if (new_cost < v->cost)
            {
                v->cost = new_cost;
                v->prev = u - &nodes[0]; // 存储索引
                pq.push(v);
            }
        }
    }

    // 5. 如果没有找到路径
    if (end_node == nullptr)
    {
        return {};
    }

    // 6. 回溯路径
    std::vector<std::vector<double>> result;
    Node *current = end_node;
    while (current != nullptr)
    {
        result.push_back({current->q[0], current->q[1], current->q[2]});
        if (current->prev == -1)
            break;
        current = &nodes[current->prev];
    }

    // 反转路径（从起点到终点）
    std::reverse(result.begin(), result.end());
    return result;
}

// std::vector<std::vector<double>> Planner::planTrajectoryRRTStar(std::vector<std::vector<double>> points) {
//     std::vector<std::vector<double>> trajectory;
//     if (points.empty()) return trajectory;

//     // ==================== 1. 初始化空间 ====================
//     auto space(std::make_shared<ob::RealVectorStateSpace>(3));
//     ob::RealVectorBounds bounds(3);
//     for (int i = 0; i < 3; ++i) {
//         bounds.setLow(i, -M_PI);
//         bounds.setHigh(i, M_PI);
//     }
//     space->setBounds(bounds);

//     // ================= 2. 空间信息配置 =================
//     auto si(std::make_shared<ob::SpaceInformation>(space));
//     auto validity_checker = std::make_shared<StateValidityChecker>(si, this);
//     si->setStateValidityChecker(validity_checker);
//     si->setup();

//     // =============== 3. 约束空间定义 =============== 
//     class ArmConstraint : public ob::Constraint {
//         Planner* planner;
//     public:
//         ArmConstraint(Planner* p) : ob::Constraint(3, 2), planner(p) {}
        
//         void function(const Eigen::Ref<const Eigen::VectorXd>& q,
//                       Eigen::Ref<Eigen::VectorXd> out) const override 
//         {
//             Eigen::Vector2d p = planner->kinematics(q);
//             out[0] = p[0]; // 末端x约束
//             out[1] = p[1]; // 末端y约束
//         }
//     };
//     auto constraint = std::make_shared<ArmConstraint>(this);
//     auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
//     auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
//     csi->setStateValidityChecker(validity_checker);
//     csi->setup();

//     // =============== 4. 自定义采样器改进 =============== 
//     class ProjectedSampler : public ob::StateSampler {
//         Planner* planner;
//         const ob::ProjectedStateSpace* css;
//         std::vector<Eigen::Vector2d> trajectoryPoints;
        
//     public:
//         ProjectedSampler(const ob::StateSpace* space, Planner* p, 
//                         const std::vector<Eigen::Vector2d>& points)
//             : ob::StateSampler(space), planner(p),
//               css(space->as<ob::ProjectedStateSpace>()) 
//         {
//             for (const auto& pt : points) {
//                 trajectoryPoints.push_back(pt);
//             }
//         }

//         void sampleUniform(ob::State* state) override {
//             auto* rstate = state->as<ob::RealVectorStateSpace::StateType>();
            
//             // 混合采样策略：60%目标偏向 + 30%随机 + 10%边界采样
//             double strategy = (double)rand() / RAND_MAX;
//             Eigen::Vector2d target_point;
            
//             if (strategy < 0.6) { // 目标偏向采样
//                 target_point = trajectoryPoints[rand() % trajectoryPoints.size()];
//             } 
//             else if (strategy < 0.9) { // 完全随机采样
//                 for (int i = 0; i < 3; ++i) {
//                     rstate->values[i] = css->getBounds().low[i] + 
//                                       (css->getBounds().high[i] - css->getBounds().low[i]) * 
//                                       ((double)rand() / RAND_MAX);
//                 }
//                 css->project(rstate); // 投影到流形
//                 return;
//             }
//             else { // 边界增强采样
//                 for (int i = 0; i < 3; ++i) {
//                     if (rand() % 2 == 0) 
//                         rstate->values[i] = css->getBounds().low[i];
//                     else
//                         rstate->values[i] = css->getBounds().high[i];
//                 }
//                 css->project(rstate);
//                 return;
//             }
            
//             // 牛顿法投影（带收敛检查）
//             Eigen::Vector3d q_current(
//                 rstate->values[0],
//                 rstate->values[1],
//                 rstate->values[2]
//             );
//             Eigen::Vector3d q_projected = planner->ikinematics(target_point, q_current);
            
//             // 检查投影有效性
//             Eigen::Vector2d p_projected = planner->kinematics(q_projected);
//             if ((p_projected - target_point).norm() < 2.0 * Tolerance) {
//                 rstate->values[0] = q_projected[0];
//                 rstate->values[1] = q_projected[1];
//                 rstate->values[2] = q_projected[2];
//             } else {
//                 // 投影失败时回退到随机采样
//                 for (int i = 0; i < 3; ++i) {
//                     rstate->values[i] = css->getBounds().low[i] + 
//                                       (css->getBounds().high[i] - css->getBounds().low[i]) * 
//                                       ((double)rand() / RAND_MAX);
//                 }
//                 css->project(rstate);
//             }
//         }
//     };

//     // =============== 5. 轨迹点分段规划 ===============
//     Eigen::Vector3d current_start = ikinematics(
//         Eigen::Vector2d(points[0][0], points[0][1]), 
//         q
//     );
//     trajectory.push_back({current_start[0], current_start[1], current_start[2]});
    
//     for (size_t i = 1; i < points.size(); ++i) {
//         // 5.1 设置当前目标点
//         Eigen::Vector2d current_goal(points[i][0], points[i][1]);
//         validity_checker->currentGoal = current_goal;  // 仅用于可视化
        
//         // 5.2 问题定义
//         auto pdef(std::make_shared<ob::ProblemDefinition>(csi));
        
//         // 起点设置（验证有效性）
//         ob::ScopedState<> start_state(css);
//         start_state[0] = current_start[0];
//         start_state[1] = current_start[1];
//         start_state[2] = current_start[2];
        
//         if (!csi->isValid(start_state.get())) {
//             // 起点无效时的恢复策略 
//             ob::StateSamplerPtr sampler = css->allocStateSampler();
//             for (int attempt = 0; attempt < 100; ++attempt) {
//                 sampler->sampleUniform(start_state.get());
//                 if (csi->isValid(start_state.get())) break;
//             }
//         }
//         pdef->addStartState(start_state);
        
//         // 目标设置（约束感知）
//         auto goal_condition = std::make_shared<ob::GoalSampleableRegion>(
//             csi, 
//             ikinematics(current_goal, current_start).data(), 
//             Tolerance
//         );
//         pdef->setGoal(goal_condition);
        
//         // 5.3 优化目标
//         pdef->setOptimizationObjective(
//             std::make_shared<ob::PathLengthOptimizationObjective>(csi)
//         );
        
//         // 5.4 采样器配置
//         std::vector<Eigen::Vector2d> eigen_points;
//         for (const auto& pt : points) {
//             eigen_points.emplace_back(pt[0], pt[1]);
//         }
//         css->setStateSamplerAllocator([=](const ob::StateSpace* ss) {
//             return std::make_shared<ProjectedSampler>(ss, this, eigen_points);
//         });
        
//         // 5.5 规划器设置
//         auto planner(std::make_shared<og::RRTstar>(csi));
//         planner->setProblemDefinition(pdef);
//         planner->setup();
//         planner->setRange(0.5); // 根据机械臂尺寸调整
//         planner->setGoalBias(0.2);
        
//         // 5.6 执行规划
//         ob::PlannerStatus solved = planner->solve(2.0 + i * 0.5); // 动态增加时间
        
//         // 5.7 提取路径
//         if (solved) {
//             og::PathGeometric path = 
//                 static_cast<og::PathGeometric&>(*pdef->getSolutionPath());
                
//             // 处理角度连续性
//             Eigen::Vector3d prev_q = current_start;
//             for (size_t j = 1; j < path.getStateCount(); ++j) {
//                 auto* state = path.getState(j)->as<ob::RealVectorStateSpace::StateType>();
//                 Eigen::Vector3d new_q(state->values[0], state->values[1], state->values[2]);
                
//                 // 角度规范化
//                 for (int k = 0; k < 3; k++) {
//                     double diff = new_q[k] - prev_q[k];
//                     if (diff > M_PI) new_q[k] -= 2 * M_PI;
//                     if (diff < -M_PI) new_q[k] += 2 * M_PI;
//                 }
//                 trajectory.push_back({new_q[0], new_q[1], new_q[2]});
//                 prev_q = new_q;
//             }
//             current_start = prev_q;
//         } else {
//             // 规划失败回退
//             Eigen::Vector3d fallback_q = ikinematics(current_goal, current_start);
//             trajectory.push_back({fallback_q[0], fallback_q[1], fallback_q[2]});
//             current_start = fallback_q;
//         }
//     }
    
//     return trajectory; // 返回路径已包含平滑处理
// }

Eigen::Matrix<double, 2, 3> Planner::J_matrix(const Eigen::Vector3d &q_val)
{
    Eigen::Matrix<double, 2, 3> J;
    J(0, 0) = -L1 * sin(q_val[0]) - L2 * sin(q_val[0] + q_val[1]) - L3 * sin(q_val[0] + q_val[1] + q_val[2]);
    J(0, 1) = -L2 * sin(q_val[0] + q_val[1]) - L3 * sin(q_val[0] + q_val[1] + q_val[2]);
    J(0, 2) = -L3 * sin(q_val[0] + q_val[1] + q_val[2]);
    J(1, 0) = L1 * cos(q_val[0]) + L2 * cos(q_val[0] + q_val[1]) + L3 * cos(q_val[0] + q_val[1] + q_val[2]);
    J(1, 1) = L2 * cos(q_val[0] + q_val[1]) + L3 * cos(q_val[0] + q_val[1] + q_val[2]);
    J(1, 2) = L3 * cos(q_val[0] + q_val[1] + q_val[2]);
    return J;
}

Eigen::Vector2d Planner::kinematics(const Eigen::Vector3d &q_val)
{
    Eigen::Vector2d p;
    p(0) = L1 * cos(q_val[0]) + L2 * cos(q_val[0] + q_val[1]) + L3 * cos(q_val[0] + q_val[1] + q_val[2]);
    p(1) = L1 * sin(q_val[0]) + L2 * sin(q_val[0] + q_val[1]) + L3 * sin(q_val[0] + q_val[1] + q_val[2]);
    return p;
}

Eigen::Vector3d Planner::ikinematics(const Eigen::Vector2d &p_d, const Eigen::Vector3d &q_initial)
{
    Eigen::Vector2d p = kinematics(q_initial);            // 当前点
    Eigen::Vector2d p_error = p_d - p;                    // 计算当前点与目标点的误差
    double distance_error = sqrt((p_error).dot(p_error)); // 计算当前点与目标点的距离误差
    if (distance_error < 1e-10)
    { // 如果误差小于容忍度，直接返回
        return {q_initial[0], q_initial[1], q_initial[2]};
    }
    uint64_t iter = 0; // 迭代次数
    // 使用牛顿-拉夫森法求解逆运动学
    // 需要求解方程：J delta_q = p_error 求出 delta_q 那么 q = q + delta_q
    // J 是 3x2 的雅可比矩阵
    Eigen::Vector3d q_out = q_initial; // 初始化角度
    while (distance_error > Tolerance && iter < Max_Iter)
    {
        Eigen::Matrix<double, 2, 3> J = J_matrix(q_out); // 雅可比矩阵
        p_error = p_d - p;                               // 计算当前点与目标点的误差
        // std::cout << "J: " << J << std::endl;
        Eigen::Matrix<double, 3, 1> delta_q = J.completeOrthogonalDecomposition().solve(p_error);
        q_out += delta_q;                        // 更新角度
        p = kinematics(q_out);                   // 更新当前点
        distance_error = (p_error).dot(p_error); // 更新距离误差
        iter++;
    }
    return {q_out[0], q_out[1], q_out[2]}; // 返回逆运动学解
}

// 碰撞检测函数
bool Planner::checkCollision(const Eigen::Vector3d &q)
{
    // 计算所有碰撞圆的位置
    std::vector<std::pair<double, double>> centers;
    std::vector<double> radii;

    // 第一个连杆的碰撞圆
    double q0 = q[0];
    centers.push_back({(L1 / 6) * cos(q0), (L1 / 6) * sin(q0)});
    centers.push_back({(L1 / 2) * cos(q0), (L1 / 2) * sin(q0)});
    centers.push_back({(5 * L1 / 6) * cos(q0), (5 * L1 / 6) * sin(q0)});
    radii.insert(radii.end(), 3, L1 / 6);

    // 第二个连杆的碰撞圆
    double x1 = L1 * cos(q0);
    double y1 = L1 * sin(q0);
    double q01 = q0 + q[1];
    centers.push_back({x1 + (L2 / 6) * cos(q01), y1 + (L2 / 6) * sin(q01)});
    centers.push_back({x1 + (L2 / 2) * cos(q01), y1 + (L2 / 2) * sin(q01)});
    centers.push_back({x1 + (5 * L2 / 6) * cos(q01), y1 + (5 * L2 / 6) * sin(q01)});
    radii.insert(radii.end(), 3, L2 / 6);

    // 第三个连杆的碰撞圆
    double x2 = x1 + L2 * cos(q01);
    double y2 = y1 + L2 * sin(q01);
    double q012 = q01 + q[2];
    centers.push_back({x2 + (L3 / 6) * cos(q012), y2 + (L3 / 6) * sin(q012)});
    centers.push_back({x2 + (L3 / 2) * cos(q012), y2 + (L3 / 2) * sin(q012)});
    centers.push_back({x2 + (5 * L3 / 6) * cos(q012), y2 + (5 * L3 / 6) * sin(q012)});
    radii.insert(radii.end(), 3, L3 / 6);

    // 检查每个碰撞圆与所有障碍物
    for (int i = 0; i < 9; i++)
    {
        double cx = centers[i].first;
        double cy = centers[i].second;
        double r = radii[i];

        for (const auto &obs : obstacles)
        {
            double dx = cx - obs.x;
            double dy = cy - obs.y;
            double dist_sq = dx * dx + dy * dy;
            double min_dist = r + obs.r + Collision_Epsilon;

            if (dist_sq < min_dist * min_dist)
            {
                return true; // 碰撞
            }
        }
    }

    return false; // 无碰撞
}

double calculate_total_q_distance(const std::vector<std::vector<double>> &q)
{
    double total_q_length = 0; // q在状态空间中的轨迹长度
    for (size_t i = 0; i < q.size() - 1; i++)
    {
        double q_length = sqrt(pow(q[i + 1][0] - q[i][0], 2) + pow(q[i + 1][1] - q[i][1], 2) + pow(q[i + 1][2] - q[i][2], 2));
        total_q_length += q_length;
    }
    return total_q_length;
}

// 辅助函数：规范化角度到[-π, π]
double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

std::vector<Point> computeCircleIntersection(const Circle &c1, const Circle &c2)
{
    // 计算圆心之间的向量
    double dx = c2.x - c1.x;
    double dy = c2.y - c1.y;

    // 计算圆心距离
    double d = std::hypot(dx, dy);
    const double EPS = 1e-8;

    // 处理圆心重合的情况（题目确保有交点，但需避免除以0）
    if (d < EPS)
    {
        throw std::invalid_argument("Circles are concentric, infinite solutions");
    }

    // 计算距离相关的中间量
    double d_sq = d * d;
    double r1_sq = c1.r * c1.r;
    double r2_sq = c2.r * c2.r;

    // 计算投影长度
    double h = (r1_sq + d_sq - r2_sq) / (2 * d);

    // 计算垂直方向的距离（勾股定理）
    double k_sq = r1_sq - h * h;
    // 处理浮点精度导致负值的情况
    if (k_sq < -EPS)
    {
        throw std::runtime_error("No intersection due to floating point precision");
    }
    double k = std::sqrt(std::max(k_sq, 0.0));

    // 计算基座点（圆心连线上的投影点）
    double base_x = c1.x + (h * dx) / d;
    double base_y = c1.y + (h * dy) / d;

    // 计算垂直方向的偏移向量（归一化并缩放）
    double offset_x = (k * (-dy)) / d;
    double offset_y = (k * dx) / d;

    // 计算两个交点
    Point p1 = {base_x + offset_x, base_y + offset_y};
    Point p2 = {base_x - offset_x, base_y - offset_y};

    return {p1, p2};
}