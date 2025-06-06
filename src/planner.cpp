#include "planner.h"
#include <iostream>
#include <Eigen/Sparse> // Include the Eigen Sparse module
#include <ompl/config.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>

Planner::Planner(double L1, double L2, double L3, double x, double y, double r, double q1, double q2, double q3)
{
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->circle_x = x;
    this->circle_y = y;
    this->circle_r = r;
    this->q = {q1, q2, q3}; // Initialize q with 3 joint angles
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
        Eigen::Vector2d p_d(point[0], point[1]);              // 目标点
        Eigen::Vector2d p = kinematics(q);                    // 当前点
        Eigen::Vector2d p_error = p_d - p;                    // 计算当前点与目标点的误差
        double distance_error = sqrt((p_error).dot(p_error)); // 计算当前点与目标点的距离误差
        if (distance_error < 1e-10)
        { // 如果误差小于容忍度，直接返回（不优化第一个点的位置，因为这是机械臂的初始位置）
            trajectory.push_back({q[0], q[1], q[2]});
            continue;
        }
        uint64_t iter = 0; // 迭代次数
        // 使用牛顿-拉夫森法求解逆运动学
        // 需要求解方程：J delta_q = p_error 求出 delta_q 那么 q = q + delta_q
        // J 是 3x2 的雅可比矩阵
        while (distance_error > Tolerance && iter < Max_Iter)
        {
            Eigen::Matrix<double, 2, 3> J = J_matrix(q); // 雅可比矩阵
            p_error = p_d - p;                           // 计算当前点与目标点的误差
            // std::cout << "J: " << J << std::endl;
            Eigen::Matrix<double, 3, 1> delta_q = J.completeOrthogonalDecomposition().solve(p_error);
            q += delta_q;                            // 更新角度
            p = kinematics(q);                       // 更新当前点
            distance_error = (p_error).dot(p_error); // 更新距离误差
            iter++;
        }
        trajectory.push_back({q[0], q[1], q[2]}); // 将当前点添加到轨迹中
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
        // double alpha = 0.1; // 固定步长

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

std::vector<std::vector<double>> Planner::planTrajectoryRRTStar(std::vector<std::vector<double>> points)
{
    // 除去第一个点，因为它是机械臂的初始位置
    if (points.size() <= 1)
        return {std::vector<double>{q[0], q[1], q[2]}};

    // 使用OMPL库进行RRT*规划
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    // 1. 定义状态空间（三连杆关节角度空间）
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // 2. 设置关节角度边界（示例为全周角度，可根据实际调整）
    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; ++i)
    {
        bounds.setLow(i, -M_PI);
        bounds.setHigh(i, M_PI);
    }
    space->setBounds(bounds);

    // 3. 创建空间信息实例
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // 4. 定义状态有效性检查器
    class ValidityChecker : public ob::StateValidityChecker
    {
    public:
        ValidityChecker(const ob::SpaceInformationPtr &si, Planner *planner)
            : ob::StateValidityChecker(si), planner_(planner) {}

        bool isValid(const ob::State *state) const override
        {
            const auto *angles = state->as<ob::RealVectorStateSpace::StateType>();
            Eigen::Vector3d q(angles->values[0], angles->values[1], angles->values[2]);
            Eigen::Vector2d p = planner_->kinematics(q);

            // 约束末端必须在目标圆上
            double dx = p[0] - planner_->circle_x;
            double dy = p[1] - planner_->circle_y;
            double error = fabs(dx * dx + dy * dy - pow(planner_->circle_r, 2));
            return error < Tolerance; // 使用与优化相同的容差
        }

    private:
        Planner *planner_;
    };
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si, this));
    si->setup();

    // 5. 定义优化目标（路径长度）
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    // 6. 设置起点和终点
    ob::ScopedState<> start(space);
    start[0] = q[0];
    start[1] = q[1];
    start[2] = q[2];

    ob::ScopedState<> goal(space);
    // 假设目标点对应机械臂末端坐标，需转换为关节空间
    Eigen::Vector2d target_point(circle_x, circle_y); // 目标点为圆心坐标
    Eigen::Vector3d q_goal = q;                       // 初始猜测为当前关节角度

    // 使用牛顿法求解逆运动学
    for (int iter = 0; iter < Max_Iter; ++iter)
    {
        Eigen::Vector2d p = kinematics(q_goal);
        Eigen::Vector2d error = target_point - p;
        if (error.norm() < Tolerance)
            break;
        Eigen::Matrix<double, 2, 3> J = J_matrix(q_goal);
        q_goal += J.completeOrthogonalDecomposition().solve(error);
    }

    goal[0] = q_goal[0];
    goal[1] = q_goal[1];
    goal[2] = q_goal[2];

    pdef->setStartAndGoalStates(start, goal);

    // 7. 配置RRT*规划器
    auto planner = std::make_shared<og::RRTstar>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // 8. 执行规划
    ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(2.0)); // 规划时间2秒

    // 9. 提取路径
    std::vector<std::vector<double>> trajectory;
    if (solved)
    {
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        for (size_t i = 0; i < path->getStateCount(); ++i)
        {
            const auto *state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            trajectory.push_back({state->values[0], state->values[1], state->values[2]});
        }
    }

    return trajectory;
}

// planner.cpp 实现修改
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

// Eigen::Vector3d Planner::ikinematics(const Eigen::Vector2d &p_d, const Eigen::Vector3d &q_initial){

// }

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