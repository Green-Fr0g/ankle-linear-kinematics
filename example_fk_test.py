"""
踝关节FK求解器测试示例
演示如何使用FK求解器进行正向运动学计算
"""
import math
from ankle_fk_solver import AnkleFKSolver
from param import get_geometry


def test_basic_fk():
    """基本FK求解测试"""
    print("=" * 60)
    print("测试 1: 基本FK求解")
    print("=" * 60)

    # 创建求解器
    geometry = get_geometry("standard")
    solver = AnkleFKSolver(geometry)

    # 测试用例：给定直线模组长度
    l1 = 0.35  # 第一根直线模组长度（米）
    l2 = 0.36  # 第二根直线模组长度（米）

    print(f"\n输入:")
    print(f"  l1 = {l1:.4f} m")
    print(f"  l2 = {l2:.4f} m")

    # 求解FK
    try:
        state = solver.solve_fk(l1, l2)

        alpha_deg, theta_deg = state.to_degrees()

        print(f"\n输出:")
        print(f"  alpha (roll)  = {state.alpha:.6f} rad = {alpha_deg:.3f}°")
        print(f"  theta (pitch) = {state.theta:.6f} rad = {theta_deg:.3f}°")

        # 验证：将结果代入正向运动学，检查是否能恢复原始长度
        l1_check, l2_check = solver._forward_kinematics(state.alpha, state.theta)
        error1 = abs(l1 - l1_check)
        error2 = abs(l2 - l2_check)

        print(f"\n验证:")
        print(f"  l1 重新计算 = {l1_check:.6f} m, 误差 = {error1:.2e} m")
        print(f"  l2 重新计算 = {l2_check:.6f} m, 误差 = {error2:.2e} m")

        if error1 < 1e-6 and error2 < 1e-6:
            print("  ✓ 验证通过")
        else:
            print("  ✗ 验证失败")

    except Exception as e:
        print(f"\n✗ 求解失败: {e}")


def test_multiple_poses():
    """测试多个姿态"""
    print("\n" + "=" * 60)
    print("测试 2: 多个姿态测试")
    print("=" * 60)

    geometry = get_geometry("standard")
    solver = AnkleFKSolver(geometry)

    # 测试多组长度
    test_cases = [
        (0.32, 0.32, "中立位置"),
        (0.30, 0.34, "向左倾斜"),
        (0.34, 0.30, "向右倾斜"),
        (0.35, 0.35, "前倾"),
    ]

    print(f"\n{'长度 (m)':>20} | {'角度 (deg)':>25} | {'描述':>12}")
    print("-" * 60)
    print(f"{'l1':>9} {'l2':>9} | {'alpha':>11} {'theta':>11} |")
    print("-" * 60)

    for l1, l2, description in test_cases:
        try:
            state = solver.solve_fk(l1, l2)
            alpha_deg, theta_deg = state.to_degrees()

            print(f"{l1:>9.4f} {l2:>9.4f} | "
                  f"{alpha_deg:>11.3f} {theta_deg:>11.3f} | {description}")
        except Exception as e:
            print(f"{l1:>9.4f} {l2:>9.4f} | {'失败':>25} | {description}")
            print(f"  错误: {e}")


def test_jacobian_and_singularity():
    """测试雅可比矩阵和奇异性检查"""
    print("\n" + "=" * 60)
    print("测试 3: 雅可比矩阵和奇异性分析")
    print("=" * 60)

    geometry = get_geometry("standard")
    solver = AnkleFKSolver(geometry)

    # 求解一个姿态
    l1, l2 = 0.33, 0.34
    state = solver.solve_fk(l1, l2)

    print(f"\n当前姿态:")
    alpha_deg, theta_deg = state.to_degrees()
    print(f"  alpha = {alpha_deg:.3f}°")
    print(f"  theta = {theta_deg:.3f}°")

    # 计算雅可比
    J = solver.compute_jacobian(state.alpha, state.theta)
    print(f"\n雅可比矩阵 J_l = ∂(l1,l2)/∂(alpha,theta):")
    print(f"  [{J[0][0]:>10.6f}  {J[0][1]:>10.6f}]")
    print(f"  [{J[1][0]:>10.6f}  {J[1][1]:>10.6f}]")

    # 奇异性检查
    is_singular, min_sv = solver.check_singularity(state.alpha, state.theta)
    print(f"\n奇异性分析:")
    print(f"  最小奇异值 = {min_sv:.6f}")
    print(f"  是否接近奇异: {'是 ⚠' if is_singular else '否 ✓'}")


def test_iterative_tracking():
    """测试实时控制场景：连续轨迹跟踪"""
    print("\n" + "=" * 60)
    print("测试 4: 连续轨迹跟踪（模拟实时控制）")
    print("=" * 60)

    geometry = get_geometry("standard")
    solver = AnkleFKSolver(geometry)

    # 模拟一个正弦轨迹
    print("\n模拟机构沿正弦轨迹运动...")

    import time
    total_time = 0.0
    num_steps = 10

    for i in range(num_steps):
        t = i / (num_steps - 1)  # 归一化时间 [0, 1]

        # 生成轨迹：l1 和 l2 做正弦变化
        l1 = 0.33 + 0.02 * math.sin(2 * math.pi * t)
        l2 = 0.33 + 0.02 * math.cos(2 * math.pi * t)

        # 求解（使用上一次的 theta 作为初值，加快收敛）
        start = time.perf_counter()
        state = solver.solve_fk(l1, l2)
        elapsed = time.perf_counter() - start
        total_time += elapsed

        if i % 3 == 0:  # 每3步打印一次
            alpha_deg, theta_deg = state.to_degrees()
            print(f"  步骤 {i+1:2d}: "
                  f"l1={l1:.4f}, l2={l2:.4f} → "
                  f"α={alpha_deg:>6.2f}°, θ={theta_deg:>6.2f}° "
                  f"({elapsed*1000:.2f}ms)")

    avg_time = total_time / num_steps
    print(f"\n平均求解时间: {avg_time*1000:.3f} ms")
    print(f"理论控制频率: {1/avg_time:.1f} Hz")


def test_force_mapping():
    """测试力映射：从直线电机推力到关节力矩"""
    print("\n" + "=" * 60)
    print("测试 5: 力映射 (τ = J_l^T F)")
    print("=" * 60)

    geometry = get_geometry("standard")
    solver = AnkleFKSolver(geometry)

    # 求解一个姿态
    l1, l2 = 0.33, 0.33
    state = solver.solve_fk(l1, l2)

    alpha_deg, theta_deg = state.to_degrees()
    print(f"\n当前姿态: alpha={alpha_deg:.2f}°, theta={theta_deg:.2f}°")

    # 获取雅可比
    J = solver.compute_jacobian(state.alpha, state.theta)

    # 假设直线电机推力
    F1 = 100.0  # N
    F2 = 100.0  # N

    print(f"\n直线电机推力:")
    print(f"  F1 = {F1:.1f} N")
    print(f"  F2 = {F2:.1f} N")

    # 计算关节力矩: τ = J_l^T F
    tau_roll = J[0][0] * F1 + J[1][0] * F2
    tau_pitch = J[0][1] * F1 + J[1][1] * F2

    print(f"\n关节力矩:")
    print(f"  τ_roll  = {tau_roll:.3f} N·m")
    print(f"  τ_pitch = {tau_pitch:.3f} N·m")


def main():
    """运行所有测试"""
    print("\n踝关节直线模组机构 FK 求解器测试\n")

    test_basic_fk()
    test_multiple_poses()
    test_jacobian_and_singularity()
    test_iterative_tracking()
    test_force_mapping()

    print("\n" + "=" * 60)
    print("所有测试完成")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
