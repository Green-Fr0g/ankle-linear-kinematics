"""
位置正解方法对比：精度与耗时

真值来自解析逆解：先取关节角 q，算出 l = IK(q)，再比较 FK(l) 与 q。
    随机姿态  限位内随机采样，每次求解前清除热启动（相当于冷启动 / 跳变输入）
    连续轨迹  1 kHz 采样的圆形轨迹，保留热启动（相当于实际状态估计）

用法：
    .venv/bin/python bench_fk.py
    .venv/bin/python bench_fk.py --step 0.5 --samples 20000   # 不读文件，按指定步长现场建表
"""
import argparse
import math
import random
import time

from ankle_lut import DEFAULT_LUT_PATH, FK_METHODS, calf_actuator_config, make_kinematics
from param import (
    CALF_ACTUATOR_LENGTH_RANGE as LENGTH_RANGE,
    CALF_ACTUATOR_ROLL_RANGE as ROLL_RANGE,
    CALF_ACTUATOR_PITCH_RANGE as PITCH_RANGE,
)


def random_poses(kin, n, seed=0):
    """限位内、且推杆长度在行程内的随机姿态 → [(alpha, theta, l1, l2)]"""
    rng = random.Random(seed)
    out = []
    while len(out) < n:
        a = rng.uniform(*ROLL_RANGE)
        t = rng.uniform(*PITCH_RANGE)
        l1, l2 = kin.inverse_kinematics(a, t)
        if all(LENGTH_RANGE[0] <= x <= LENGTH_RANGE[1] for x in (l1, l2)):
            out.append((a, t, l1, l2))
    return out


def circle_poses(kin, duration=4.0, rate=1000.0, roll=0.35, pitch=0.44, freq=0.5):
    w = 2 * math.pi * freq
    out = []
    for k in range(int(duration * rate)):
        t = k / rate
        a, p = roll * math.sin(w * t), pitch * math.cos(w * t)
        out.append((a, p) + kin.inverse_kinematics(a, p))
    return out


def evaluate(kin, poses, cold):
    """→ (失败次数, roll 最大误差 deg, pitch 最大误差 deg, RMS deg, 平均耗时 us)"""
    kin.reset()
    fails, ea, et, sq, n = 0, 0.0, 0.0, 0.0, 0
    t0 = time.perf_counter()
    for a, t, l1, l2 in poses:
        if cold:
            kin.reset()
        try:
            st = kin.forward_kinematics(l1, l2)
        except ValueError:
            fails += 1
            kin.reset()
            continue
        da, dt = abs(st.alpha - a), abs(st.theta - t)
        ea, et = max(ea, da), max(et, dt)
        sq += da * da + dt * dt
        n += 1
    us = (time.perf_counter() - t0) / len(poses) * 1e6
    rms = math.sqrt(sq / (2 * n)) if n else math.nan
    return fails, math.degrees(ea), math.degrees(et), math.degrees(rms), us


def main():
    p = argparse.ArgumentParser(description="位置正解方法对比")
    p.add_argument("--step", type=float, default=None,
                   help="查表网格步长 mm；指定时现场建表，否则读取查表文件")
    p.add_argument("--samples", type=int, default=5000, help="随机姿态数（默认 5000）")
    args = p.parse_args()

    cfg = calf_actuator_config()
    if args.step is None:
        lut_kw = dict(lut_path=DEFAULT_LUT_PATH)
        source = f"读取 {DEFAULT_LUT_PATH.name}"
    else:
        lut_kw = dict(lut_path=None, step=args.step * 1e-3)
        source = "现场建表"
    kins = {}
    for m in FK_METHODS:
        t0 = time.perf_counter()
        kins[m] = make_kinematics(m, **lut_kw, **cfg)
        lut = getattr(kins[m], "lut", None)
        if lut is not None:
            print(f"[{m}] {source}：{lut.n}x{lut.n}，步长 {lut.step*1e3:.3f} mm，"
                  f"可达节点 {lut.valid_ratio:.1%}，耗时 {time.perf_counter()-t0:.2f} s")

    ref = kins["newton"]
    cases = (("随机姿态（冷启动）", random_poses(ref, args.samples), True),
             ("连续轨迹（热启动）", circle_poses(ref), False))
    for name, poses, cold in cases:
        print(f"\n{name}，{len(poses)} 个样本")
        print(f"  {'方法':<12}{'失败':>6}{'roll 最大':>12}{'pitch 最大':>12}{'RMS':>12}{'耗时':>12}")
        for m, kin in kins.items():
            f, ea, et, rms, us = evaluate(kin, poses, cold)
            print(f"  {m:<12}{f:>6}{ea:>11.2e}°{et:>11.2e}°{rms:>11.2e}°{us:>9.1f} us")


if __name__ == "__main__":
    main()
