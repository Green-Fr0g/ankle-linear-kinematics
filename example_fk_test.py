"""
踝关节FK求解器测试示例
几何参数使用 model/xml/calf_actuator.xml 对应的 "calf_actuator" 配置
"""
import math
import time
from pathlib import Path

from ankle_fk_solver import AnkleFKSolver
from param import (
    get_geometry,
    CALF_ACTUATOR_NOMINAL_LENGTH,
    CALF_ACTUATOR_ROLL_RANGE,
    CALF_ACTUATOR_PITCH_RANGE,
)

GEOMETRY_NAME = "calf_actuator"
XML_PATH = Path(__file__).parent / "model" / "xml" / "calf_actuator.xml"


def make_solver() -> AnkleFKSolver:
    """带 XML 关节限位的求解器（限位用于剔除长度方程的伪解）"""
    return AnkleFKSolver(
        get_geometry(GEOMETRY_NAME),
        alpha_range=CALF_ACTUATOR_ROLL_RANGE,
        theta_range=CALF_ACTUATOR_PITCH_RANGE,
    )


def _header(title: str):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def test_zero_pose():
    """零位：两根推杆等长且等于名义长度，反解回 (0, 0)"""
    _header("测试 1: 零位")
    solver = make_solver()

    l1, l2 = solver._forward_kinematics(0.0, 0.0)
    state = solver.solve_fk(l1, l2)
    print(f"  l1 = {l1:.6f} m, l2 = {l2:.6f} m, 名义长度 = {CALF_ACTUATOR_NOMINAL_LENGTH:.6f} m")
    print(f"  反解: alpha = {state.alpha:.2e} rad, theta = {state.theta:.2e} rad")

    ok = (abs(l1 - CALF_ACTUATOR_NOMINAL_LENGTH) < 1e-12
          and abs(l2 - CALF_ACTUATOR_NOMINAL_LENGTH) < 1e-12
          and abs(state.alpha) < 1e-8 and abs(state.theta) < 1e-8)
    print(f"  {'✓ 通过' if ok else '✗ 失败'}")
    return ok


def test_round_trip():
    """多姿态往返：角度 -> 长度 -> 角度"""
    _header("测试 2: 多姿态往返")
    solver = make_solver()

    cases = [
        (0.3, 0.0, "+roll（+y 侧推杆 l1 缩短）"),
        (-0.3, 0.0, "-roll"),
        (0.0, 0.3, "+pitch"),
        (0.0, -0.3, "-pitch"),
        (0.5, 0.5, "roll + pitch 耦合"),
        (0.87, -0.87, "关节限位角"),
    ]

    print(f"  {'alpha':>7} {'theta':>7} | {'l1 (m)':>9} {'l2 (m)':>9} | {'角度误差 (deg)':>14} | 描述")
    all_ok = True
    for alpha, theta, desc in cases:
        l1, l2 = solver._forward_kinematics(alpha, theta)
        # 冷启动，检验从 0 初值出发的收敛性
        state = make_solver().solve_fk(l1, l2)
        err = math.degrees(max(abs(state.alpha - alpha), abs(state.theta - theta)))
        all_ok &= err < 1e-4
        print(f"  {alpha:>7.3f} {theta:>7.3f} | {l1:>9.5f} {l2:>9.5f} | {err:>14.2e} | {desc}")

    print(f"  {'✓ 通过' if all_ok else '✗ 失败'}")
    return all_ok


def test_mujoco_consistency(num_samples: int = 500):
    """与 MuJoCo 模型对比：同一组关节角下的 site 距离应等于求解器长度"""
    _header("测试 3: 与 MuJoCo (calf_actuator.xml) 一致性")
    try:
        import random
        import numpy as np
        import mujoco
    except ImportError:
        print("  未安装 mujoco / numpy，跳过")
        return True

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    def qadr(name):
        return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]

    def sid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)

    q_roll, q_pitch = qadr("ankle_roll_joint"), qadr("ankle_pitch_joint")
    sites = [sid(n) for n in ("a1", "b1", "a2", "b2")]
    solver = make_solver()

    random.seed(0)
    max_len_err = max_ang_err = 0.0
    for _ in range(num_samples):
        alpha = random.uniform(*CALF_ACTUATOR_ROLL_RANGE)
        theta = random.uniform(*CALF_ACTUATOR_PITCH_RANGE)
        data.qpos[:] = model.qpos0
        data.qpos[q_roll], data.qpos[q_pitch] = alpha, theta
        mujoco.mj_kinematics(model, data)

        a1, b1, a2, b2 = (data.site_xpos[i] for i in sites)
        l1_mj, l2_mj = np.linalg.norm(a1 - b1), np.linalg.norm(a2 - b2)
        l1, l2 = solver._forward_kinematics(alpha, theta)
        max_len_err = max(max_len_err, abs(l1 - l1_mj), abs(l2 - l2_mj))

        state = solver.solve_fk(l1_mj, l2_mj)
        max_ang_err = max(max_ang_err, abs(state.alpha - alpha), abs(state.theta - theta))

    ok = max_len_err < 1e-9 and math.degrees(max_ang_err) < 1e-4
    print(f"  随机采样 {num_samples} 个限位内姿态（复用同一求解器，热启动初值互不相关）")
    print(f"  最大长度误差 = {max_len_err:.2e} m")
    print(f"  最大反解角度误差 = {math.degrees(max_ang_err):.2e} deg")
    print(f"  {'✓ 通过' if ok else '✗ 失败'}")
    return ok


def test_jacobian_and_singularity():
    """解析雅可比 vs 数值差分，并做奇异性检查"""
    _header("测试 4: 雅可比矩阵和奇异性分析")
    solver = make_solver()
    alpha, theta = 0.2, -0.3

    J = solver.compute_jacobian(alpha, theta)
    eps = 1e-6
    fk = solver._forward_kinematics
    Jn = (
        ((fk(alpha + eps, theta)[0] - fk(alpha - eps, theta)[0]) / (2 * eps),
         (fk(alpha, theta + eps)[0] - fk(alpha, theta - eps)[0]) / (2 * eps)),
        ((fk(alpha + eps, theta)[1] - fk(alpha - eps, theta)[1]) / (2 * eps),
         (fk(alpha, theta + eps)[1] - fk(alpha, theta - eps)[1]) / (2 * eps)),
    )
    err = max(abs(J[i][k] - Jn[i][k]) for i in range(2) for k in range(2))

    print(f"  姿态: alpha = {alpha}, theta = {theta} rad")
    print(f"  J_l = ∂(l1,l2)/∂(alpha,theta):")
    print(f"    [{J[0][0]:>10.6f}  {J[0][1]:>10.6f}]")
    print(f"    [{J[1][0]:>10.6f}  {J[1][1]:>10.6f}]")
    print(f"  与数值差分最大误差 = {err:.2e}")

    is_singular, min_sv = solver.check_singularity(alpha, theta)
    print(f"  最小奇异值 = {min_sv:.6f} m/rad, 是否接近奇异: {'是 ⚠' if is_singular else '否'}")

    ok = err < 1e-8
    print(f"  {'✓ 通过' if ok else '✗ 失败'}")
    return ok


def test_iterative_tracking(num_steps: int = 200):
    """连续轨迹跟踪（模拟实时控制，使用上一周期结果作初值）"""
    _header("测试 5: 连续轨迹跟踪")
    solver = make_solver()

    total_time = 0.0
    max_err = 0.0
    for i in range(num_steps):
        t = i / (num_steps - 1)
        alpha = 0.5 * math.sin(2 * math.pi * t)
        theta = 0.6 * math.sin(4 * math.pi * t)
        l1, l2 = solver._forward_kinematics(alpha, theta)

        start = time.perf_counter()
        state = solver.solve_fk(l1, l2)
        total_time += time.perf_counter() - start
        max_err = max(max_err, abs(state.alpha - alpha), abs(state.theta - theta))

    avg_time = total_time / num_steps
    ok = math.degrees(max_err) < 1e-4
    print(f"  {num_steps} 步, 最大角度误差 = {math.degrees(max_err):.2e} deg")
    print(f"  平均求解时间 = {avg_time * 1000:.3f} ms（约 {1 / avg_time:.0f} Hz）")
    print(f"  {'✓ 通过' if ok else '✗ 失败'}")
    return ok


def test_force_mapping():
    """力映射：tau = J_l^T F"""
    _header("测试 6: 力映射 (τ = J_l^T F)")
    solver = make_solver()

    alpha, theta = 0.0, 0.0
    J = solver.compute_jacobian(alpha, theta)
    F1, F2 = 100.0, 100.0  # N，沿推杆伸长方向为正

    tau_roll = J[0][0] * F1 + J[1][0] * F2
    tau_pitch = J[0][1] * F1 + J[1][1] * F2

    print(f"  姿态: alpha = {alpha}, theta = {theta} rad")
    print(f"  F1 = {F1:.1f} N, F2 = {F2:.1f} N")
    print(f"  τ_roll  = {tau_roll:.3f} N·m（两推杆同力时应为 0）")
    print(f"  τ_pitch = {tau_pitch:.3f} N·m")

    ok = abs(tau_roll) < 1e-9
    print(f"  {'✓ 通过' if ok else '✗ 失败'}")
    return ok


def main():
    """运行所有测试，任一失败返回非零退出码"""
    print("\n踝关节直线模组机构 FK 求解器测试\n")

    results = [
        test_zero_pose(),
        test_round_trip(),
        test_mujoco_consistency(),
        test_jacobian_and_singularity(),
        test_iterative_tracking(),
        test_force_mapping(),
    ]

    passed = sum(results)
    print("\n" + "=" * 60)
    print(f"测试完成: {passed}/{len(results)} 通过")
    print("=" * 60 + "\n")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
