"""
根据 param.py 中的几何参数生成 MuJoCo 模型 XML（model/ankle_mechanism.xml）

模型与 ankle_fk_solver.py 采用完全一致的几何约定:
    小腿固定点 (lower_leg 坐标系): a1=(-m,-n,h),  a2=(-m,+n,h)
    脚踝连接点 (foot 坐标系):      b1=(-c,-d,-h2), b2=(-c,+d,-h2)
    脚踝相对小腿的姿态:            R = Rx(roll) * Ry(pitch)
    模组长度:                      l1=|a1-R*b1|, l2=|a2-R*b2|
    （MuJoCo 中同一 body 的多个 hinge 按定义顺序合成旋转, 因此关节顺序为 roll -> pitch）

用法:
    python model/build_mjcf.py                     # 使用 STANDARD_GEOMETRY, 输出默认文件
    python model/build_mjcf.py compact             # 使用 COMPACT_GEOMETRY
    python model/build_mjcf.py compact --out model/ankle_mechanism_compact.xml
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ankle_fk_solver import AnkleFKSolver  # noqa: E402
from param import get_geometry  # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "ankle_mechanism.xml"

# 关节工作范围（弧度）, 同时用于推导模组行程范围
ROLL_LIMIT = 0.40   # ±22.9°
PITCH_LIMIT = 0.70  # ±40.1°

# 位置伺服增益（单位: N/m, N·s/m）
SERVO_KP = 5000.0
SERVO_KV = 100.0
SERVO_FORCE_LIMIT = 300.0  # N


def fmt(value: float, sig: int = 6) -> str:
    """格式化浮点数（有效数字）"""
    if value == 0.0:
        value = 0.0  # 归一化 -0.0
    return f"{value:.{sig}g}"


def stroke_range(geometry) -> tuple[float, float]:
    """在工作范围内采样, 得到两根模组长度的极值（向外取整到 5mm）"""
    solver = AnkleFKSolver(geometry)
    l_min, l_max = math.inf, -math.inf
    steps = 60
    for i in range(steps + 1):
        alpha = -ROLL_LIMIT + 2.0 * ROLL_LIMIT * i / steps
        for j in range(steps + 1):
            theta = -PITCH_LIMIT + 2.0 * PITCH_LIMIT * j / steps
            l1, l2 = solver._forward_kinematics(alpha, theta)
            l_min = min(l_min, l1, l2)
            l_max = max(l_max, l1, l2)
    return math.floor(l_min / 0.005) * 0.005, math.ceil(l_max / 0.005) * 0.005


def build(config_name: str, out_path: Path) -> Path:
    """生成 XML 模型文件"""
    g = get_geometry(config_name)
    solver = AnkleFKSolver(g)

    l1_0, l2_0 = solver._forward_kinematics(0.0, 0.0)  # 中立位姿模组长度
    l_min, l_max = stroke_range(g)

    # 纯视觉尺寸（不影响运动学）
    shin_top = g.h + 0.06
    calf_bracket_ny = g.n + 0.010
    foot_bracket_ny = g.d + 0.008
    arm_sx = (g.c + 0.006) / 2.0

    xml = f"""<mujoco model="ankle_linear_module_{config_name}">
  <!--
    踝关节直线模组机构 (ankle linear-module parallel mechanism)
    由 model/build_mjcf.py 自动生成, 请勿直接修改几何数值;
    需要改参数时请编辑 param.py 后重新运行生成脚本.

    几何参数 (param.py / {config_name}):
      m  = {fmt(g.m)}    小腿固定端 x 偏置
      n  = {fmt(g.n)}    小腿固定端 y 偏置
      h  = {fmt(g.h)}    小腿固定端 z 偏置
      c  = {fmt(g.c)}    脚踝连接点 x 偏置
      d  = {fmt(g.d)}    脚踝连接点 y 偏置
      h2 = {fmt(g.h2)}   脚踝连接点 z 偏置

    运动学约定 (与 ankle_fk_solver.py 一致):
      a1=(-m,-n,h), a2=(-m,+n,h)        在小腿坐标系 lower_leg 下
      b1=(-c,-d,-h2), b2=(-c,+d,-h2)    在脚踝坐标系 foot 下
      R = Rx(roll) * Ry(pitch)          同 body 多 hinge 按定义顺序合成
      l1 = |a1 - R*b1|, l2 = |a2 - R*b2|   由两根 spatial tendon 实时给出

    说明:
      * lower_leg 焊接在世界系上, 本模型只验证机构运动学;
        接入整机时把 lower_leg 改为相应安装关节即可.
      * gravity = 0 便于算法验证; 做重力动力学测试时改为 "0 0 -9.81".
      * 执行器为位置伺服, 控制量单位 = 米, 即直线模组长度.
  -->
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 0" integrator="implicitfast"/>
  <statistic center="0 0 0.16" extent="0.45"/>

  <visual>
    <!-- 离屏渲染默认分辨率（供 mujoco.Renderer 出图使用） -->
    <global offwidth="1280" offheight="960"/>
  </visual>

  <default>
    <!-- 仅用于算法验证, 关闭所有碰撞 -->
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <light pos="0.6 -0.6 0.9" dir="-0.4 0.4 -0.6" directional="false"/>
    <light pos="-0.6 0.6 0.7" dir="0.3 -0.3 -0.5" directional="false"/>

    <!-- 小腿（固定机架） -->
    <body name="lower_leg" pos="0 0 0">
      <geom name="shin" type="capsule" fromto="0 0 -0.04 0 0 {fmt(shin_top)}"
            size="0.032" rgba="0.60 0.62 0.66 1"/>
      <!-- 模组安装座（a1-a2 连线处） -->
      <geom name="calf_bracket" type="box" pos="{fmt(-g.m)} 0 {fmt(g.h)}"
            size="0.018 {fmt(calf_bracket_ny)} 0.014" rgba="0.36 0.38 0.42 1"/>
      <site name="a1" pos="{fmt(-g.m)} {fmt(-g.n)} {fmt(g.h)}"
            size="0.0065" rgba="0.95 0.25 0.2 1"/>
      <site name="a2" pos="{fmt(-g.m)} {fmt(g.n)} {fmt(g.h)}"
            size="0.0065" rgba="0.25 0.55 0.95 1"/>

      <!-- 脚踝 2 自由度: 先绕 x 轴 roll, 再绕 y 轴 pitch（与 R=Rx*Ry 对应） -->
      <body name="foot" pos="0 0 0">
        <joint name="ankle_roll" type="hinge" axis="1 0 0"
               range="{fmt(-ROLL_LIMIT)} {fmt(ROLL_LIMIT)}" damping="0.05"/>
        <joint name="ankle_pitch" type="hinge" axis="0 1 0"
               range="{fmt(-PITCH_LIMIT)} {fmt(PITCH_LIMIT)}" damping="0.05"/>

        <geom name="ankle" type="sphere" size="0.016" rgba="0.30 0.31 0.34 1"/>
        <geom name="ankle_arm" type="box" pos="{fmt(-arm_sx)} 0 {fmt(-g.h2 / 2.0)}"
              size="{fmt(arm_sx)} 0.012 0.008" rgba="0.42 0.44 0.48 1"/>
        <geom name="foot_plate" type="box" pos="0.055 0 -0.046"
              size="0.095 0.035 0.012" rgba="0.30 0.32 0.36 1"/>
        <!-- 脚踝连接座（b1-b2 连线处） -->
        <geom name="foot_bracket" type="box" pos="{fmt(-g.c)} 0 {fmt(-g.h2)}"
              size="0.012 {fmt(foot_bracket_ny)} 0.010" rgba="0.42 0.44 0.48 1"/>
        <site name="b1" pos="{fmt(-g.c)} {fmt(-g.d)} {fmt(-g.h2)}"
              size="0.0065" rgba="0.95 0.25 0.2 1"/>
        <site name="b2" pos="{fmt(-g.c)} {fmt(g.d)} {fmt(-g.h2)}"
              size="0.0065" rgba="0.25 0.55 0.95 1"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <!-- 两根直线模组: 长度 = 对应两个连接点之间的距离 -->
    <spatial name="module_l1" width="0.006" rgba="0.95 0.25 0.2 1">
      <site site="a1"/>
      <site site="b1"/>
    </spatial>
    <spatial name="module_l2" width="0.006" rgba="0.25 0.55 0.95 1">
      <site site="a2"/>
      <site site="b2"/>
    </spatial>
  </tendon>

  <actuator>
    <!-- 直线模组位置伺服: ctrl = 目标模组长度 (m) -->
    <position name="motor_l1" tendon="module_l1" kp="{fmt(SERVO_KP)}" kv="{fmt(SERVO_KV)}"
              ctrlrange="{fmt(l_min)} {fmt(l_max)}" forcerange="-{fmt(SERVO_FORCE_LIMIT)} {fmt(SERVO_FORCE_LIMIT)}"/>
    <position name="motor_l2" tendon="module_l2" kp="{fmt(SERVO_KP)}" kv="{fmt(SERVO_KV)}"
              ctrlrange="{fmt(l_min)} {fmt(l_max)}" forcerange="-{fmt(SERVO_FORCE_LIMIT)} {fmt(SERVO_FORCE_LIMIT)}"/>
  </actuator>

  <sensor>
    <jointpos name="roll_pos" joint="ankle_roll"/>
    <jointpos name="pitch_pos" joint="ankle_pitch"/>
    <tendonpos name="l1_pos" tendon="module_l1"/>
    <tendonpos name="l2_pos" tendon="module_l2"/>
  </sensor>

  <keyframe>
    <!-- 中立位姿 roll = pitch = 0 -->
    <key name="neutral" qpos="0 0" ctrl="{fmt(l1_0)} {fmt(l2_0)}"/>
  </keyframe>
</mujoco>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(xml, encoding="utf-8")

    print(f"已生成: {out_path}")
    print(f"  配置      : {config_name}")
    print(f"  中立长度  : l1 = l2 = {l1_0:.6f} m")
    print(f"  模组行程  : [{l_min:.3f}, {l_max:.3f}] m")
    print(f"  关节范围  : roll ±{ROLL_LIMIT} rad, pitch ±{PITCH_LIMIT} rad")
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", nargs="?", default="standard",
                        help="param.get_geometry 的配置名（standard / compact）")
    parser.add_argument("--out", type=Path, default=None,
                        help=f"输出文件路径（默认 {DEFAULT_OUT}）")
    args = parser.parse_args()

    out_path = args.out if args.out is not None else DEFAULT_OUT
    build(args.config, out_path)


if __name__ == "__main__":
    main()
