"""
用 MuJoCo 模型 (model/ankle_mechanism.xml) 验证踝关节 FK 算法

验证项:
  1. 几何一致性  : 模型站点距离 (l1,l2) 与 solver._forward_kinematics(roll,pitch) 逐点对比
  2. 正运动学求解: 由模型测得的 (l1,l2) 输入 solver.solve_fk, 恢复的 (roll,pitch) 与模型姿态一致
  3. 雅可比一致性: 模型中心差分雅可比 与 solver.compute_jacobian 对比
  4. 伺服仿真    : 位置伺服跟踪模组长度指令, 仿真收敛后的姿态 = 解析解

用法:
    python model/verify_fk_mujoco.py [config]     # config 默认 standard, 需与 XML 一致
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
from ankle_fk_solver import AnkleFKSolver  # noqa: E402
from param import get_geometry  # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_XML = HERE / "ankle_mechanism.xml"

# 判定阈值
TOL_GEOM = 1e-9     # m, 几何一致性
TOL_ANGLE = 1e-6    # rad, FK 求解角度误差
TOL_RESIDUAL = 1e-9  # m, FK 求解长度残差
TOL_JAC = 1e-6      # 雅可比绝对误差
TOL_SERVO_ANGLE = 1e-3   # rad, 伺服收敛姿态误差
TOL_SERVO_LEN = 1e-4     # m, 伺服收敛模组长度误差


def wrap_pi(angle: float) -> float:
    """把角度归一到 (-pi, pi]"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MjAnkle:
    """MuJoCo 踝关节模型访问封装"""

    def __init__(self, xml_path: Path):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        m = self.model

        def obj_id(obj_type, name: str) -> int:
            oid = mujoco.mj_name2id(m, obj_type, name)
            if oid < 0:
                raise ValueError(f"模型中没有名为 {name!r} 的对象")
            return oid

        self.site = {n: obj_id(mujoco.mjtObj.mjOBJ_SITE, n)
                     for n in ("a1", "a2", "b1", "b2")}

        joint_roll = obj_id(mujoco.mjtObj.mjOBJ_JOINT, "ankle_roll")
        joint_pitch = obj_id(mujoco.mjtObj.mjOBJ_JOINT, "ankle_pitch")
        self.qpos_roll = m.jnt_qposadr[joint_roll]
        self.qpos_pitch = m.jnt_qposadr[joint_pitch]

        self.act = [obj_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                    for n in ("motor_l1", "motor_l2")]
        self.ten = [obj_id(mujoco.mjtObj.mjOBJ_TENDON, n)
                    for n in ("module_l1", "module_l2")]

    def set_pose(self, roll: float, pitch: float) -> None:
        """设置关节角并前向计算"""
        self.data.qpos[self.qpos_roll] = roll
        self.data.qpos[self.qpos_pitch] = pitch
        mujoco.mj_forward(self.model, self.data)

    def lengths(self) -> tuple[float, float]:
        """由站点位置计算两根模组长度"""
        d, s = self.data, self.site
        a1, a2 = d.site_xpos[s["a1"]], d.site_xpos[s["a2"]]
        b1, b2 = d.site_xpos[s["b1"]], d.site_xpos[s["b2"]]
        return float(np.linalg.norm(a1 - b1)), float(np.linalg.norm(a2 - b2))

    def tendon_lengths(self) -> tuple[float, float]:
        return float(self.data.ten_length[self.ten[0]]), float(self.data.ten_length[self.ten[1]])


def test_geometry(mj: MjAnkle, solver: AnkleFKSolver,
                  rolls: np.ndarray, pitches: np.ndarray) -> float:
    """测试 1: 模型几何 vs 解析正运动学"""
    worst = 0.0
    for roll in rolls:
        for pitch in pitches:
            mj.set_pose(float(roll), float(pitch))
            l1_mj, l2_mj = mj.lengths()
            l1_py, l2_py = solver._forward_kinematics(float(roll), float(pitch))
            worst = max(worst, abs(l1_mj - l1_py), abs(l2_mj - l2_py))
    return worst


def test_fk_solve(mj: MjAnkle, solver: AnkleFKSolver,
                  poses: list[tuple[float, float]]) -> tuple[float, float]:
    """测试 2: 由模型长度反解姿态"""
    worst_angle = 0.0
    worst_res = 0.0
    for roll, pitch in poses:
        mj.set_pose(roll, pitch)
        l1, l2 = mj.lengths()
        state = solver.solve_fk(l1, l2, alpha_init=0.0, theta_init=0.0)
        worst_angle = max(
            worst_angle,
            abs(wrap_pi(state.alpha - roll)),
            abs(wrap_pi(state.theta - pitch)),
        )
        l1_py, l2_py = solver._forward_kinematics(state.alpha, state.theta)
        worst_res = max(worst_res, abs(l1_py - l1), abs(l2_py - l2))
    return worst_angle, worst_res


def test_jacobian(mj: MjAnkle, solver: AnkleFKSolver,
                  poses: list[tuple[float, float]], eps: float = 1e-6) -> float:
    """测试 3: 模型有限差分雅可比 vs 解析雅可比"""
    worst = 0.0
    for roll, pitch in poses:
        mj.set_pose(roll + eps, pitch)
        lp = mj.lengths()
        mj.set_pose(roll - eps, pitch)
        lm = mj.lengths()
        mj.set_pose(roll, pitch + eps)
        tp = mj.lengths()
        mj.set_pose(roll, pitch - eps)
        tm = mj.lengths()

        j_mj = (
            ((lp[0] - lm[0]) / (2 * eps), (tp[0] - tm[0]) / (2 * eps)),
            ((lp[1] - lm[1]) / (2 * eps), (tp[1] - tm[1]) / (2 * eps)),
        )
        j_py = solver.compute_jacobian(roll, pitch)
        for i in range(2):
            for j in range(2):
                worst = max(worst, abs(j_mj[i][j] - j_py[i][j]))
    return worst


def test_servo(mj: MjAnkle, solver: AnkleFKSolver,
               targets: list[tuple[float, float]]) -> list[dict]:
    """测试 4: 位置伺服跟踪模组长度指令（闭环仿真）"""
    model, data = mj.model, mj.data
    mujoco.mj_resetData(model, data)

    # 从中立位姿出发
    data.qpos[mj.qpos_roll] = 0.0
    data.qpos[mj.qpos_pitch] = 0.0
    l1_0, l2_0 = solver._forward_kinematics(0.0, 0.0)
    data.ctrl[mj.act[0]] = l1_0
    data.ctrl[mj.act[1]] = l2_0
    mujoco.mj_forward(model, data)

    results = []
    for roll_t, pitch_t in targets:
        l1_t, l2_t = solver._forward_kinematics(roll_t, pitch_t)
        data.ctrl[mj.act[0]] = l1_t
        data.ctrl[mj.act[1]] = l2_t
        for _ in range(4000):  # 4 s @ 1 kHz
            mujoco.mj_step(model, data)

        roll = float(data.qpos[mj.qpos_roll])
        pitch = float(data.qpos[mj.qpos_pitch])
        ten = mj.tendon_lengths()
        results.append({
            "target": (roll_t, pitch_t),
            "final": (roll, pitch),
            "angle_err": max(abs(wrap_pi(roll - roll_t)), abs(wrap_pi(pitch - pitch_t))),
            "ctrl_len": (l1_t, l2_t),
            "ten_len": ten,
            "len_err": max(abs(ten[0] - l1_t), abs(ten[1] - l2_t)),
        })
    return results


def main() -> int:
    config = sys.argv[1] if len(sys.argv) > 1 else "standard"

    print("=" * 72)
    print("MuJoCo 模型 vs 解析 FK 算法 一致性验证")
    print("=" * 72)
    print(f"MuJoCo 版本 : {mujoco.__version__}")
    print(f"模型文件    : {DEFAULT_XML.name}")
    print(f"参数配置    : {config}")

    geometry = get_geometry(config)
    print(f"几何参数    : m={geometry.m}, n={geometry.n}, h={geometry.h}, "
          f"c={geometry.c}, d={geometry.d}, h2={geometry.h2}")

    mj = MjAnkle(DEFAULT_XML)
    solver = AnkleFKSolver(geometry)

    # 中立位姿长度对比
    mj.set_pose(0.0, 0.0)
    l_mj = mj.lengths()
    l_py = solver._forward_kinematics(0.0, 0.0)
    print(f"中立长度    : MuJoCo l1={l_mj[0]:.6f}, l2={l_mj[1]:.6f} m | "
          f"解析 l1={l_py[0]:.6f}, l2={l_py[1]:.6f} m")
    print("-" * 72)

    ok = True

    # 测试 1: 几何一致性
    rolls = np.linspace(-0.35, 0.35, 8)
    pitches = np.linspace(-0.60, 0.60, 9)
    err = test_geometry(mj, solver, rolls, pitches)
    passed = err < TOL_GEOM
    ok &= passed
    print(f"[1] 几何一致性   : 最大误差 {err:.3e} m  (阈值 {TOL_GEOM:.0e})  "
          f"{'✓ 通过' if passed else '✗ 失败'}")

    # 测试 2: FK 求解
    poses = [(0.0, 0.0), (0.2, 0.3), (-0.25, 0.2), (0.3, -0.4),
             (-0.3, -0.5), (0.1, 0.55), (0.35, 0.1), (-0.15, -0.3)]
    err_angle, err_res = test_fk_solve(mj, solver, poses)
    passed = err_angle < TOL_ANGLE and err_res < TOL_RESIDUAL
    ok &= passed
    print(f"[2] FK 求解      : 最大角度误差 {err_angle:.3e} rad, "
          f"最大残差 {err_res:.3e} m  {'✓ 通过' if passed else '✗ 失败'}")

    # 测试 3: 雅可比
    err_jac = test_jacobian(mj, solver, poses)
    passed = err_jac < TOL_JAC
    ok &= passed
    print(f"[3] 雅可比一致性 : 最大误差 {err_jac:.3e}  (阈值 {TOL_JAC:.0e})  "
          f"{'✓ 通过' if passed else '✗ 失败'}")

    # 测试 4: 伺服仿真
    targets = [(0.20, -0.35), (-0.15, 0.30), (0.0, 0.0)]
    servo = test_servo(mj, solver, targets)
    print("[4] 伺服仿真     : 指令模组长度 -> 收敛姿态")
    print(f"    {'目标 (deg)':>18} | {'收敛 (deg)':>18} | {'角度误差 (rad)':>14} | {'长度误差 (m)':>12}")
    servo_ok = True
    for r in servo:
        t_deg = (math.degrees(r["target"][0]), math.degrees(r["target"][1]))
        f_deg = (math.degrees(r["final"][0]), math.degrees(r["final"][1]))
        row_ok = r["angle_err"] < TOL_SERVO_ANGLE and r["len_err"] < TOL_SERVO_LEN \
            and math.isfinite(r["angle_err"]) and math.isfinite(r["len_err"])
        servo_ok &= row_ok
        print(f"    {t_deg[0]:>7.2f}, {t_deg[1]:>7.2f} | {f_deg[0]:>7.2f}, {f_deg[1]:>7.2f} | "
              f"{r['angle_err']:>14.3e} | {r['len_err']:>12.3e}  {'✓' if row_ok else '✗'}")
    ok &= servo_ok

    print("-" * 72)
    print("结果: " + ("全部通过 ✓" if ok else "存在失败项 ✗"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
