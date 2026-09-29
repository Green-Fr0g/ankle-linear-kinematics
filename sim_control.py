"""
踝关节直线模组仿真控制与记录

两种控制模式，对应实际控制中的两种情况：
    joystick    手柄直接控制两根直线模组（驱动空间控制）
                左摇杆上下 → 模组 1（+y 侧）伸缩速度，右摇杆上下 → 模组 2
    trajectory  给定踝关节角度曲线（关节空间控制），经 inverse_kinematics 换算成模组目标行程

运行过程中后台记录：
    编码器量  推杆长度 l、伸长速度 l̇（仅由 actuatorpos / actuatorvel 传感器得到）
    估计量    forward_kinematics / forward_velocity 算出的 α、θ、α̇、θ̇
    对照量    仿真关节真值、指令行程，以及 trajectory 模式下的参考角度
程序结束后在 logs/<时间>/ 下保存 data.csv，并按类别分别绘制曲线图（actuator/angle/velocity/error.png）。

用法：
    .venv/bin/python sim_control.py joystick
    .venv/bin/python sim_control.py trajectory --shape circle --duration 10
    .venv/bin/python sim_control.py trajectory --shape sine --no-viewer
    .venv/bin/python sim_control.py trajectory --fk newton        # 切换位置正解方法
    .venv/bin/python sim_control.py --help / joystick --help / trajectory --help
"""
import argparse
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path

import mujoco

from ankle_kinematics import AnkleKinematics
from ankle_lut import DEFAULT_LUT_PATH, FK_METHODS, calf_actuator_config, make_kinematics
from param import (
    CALF_ACTUATOR_NOMINAL_LENGTH as L0,
    CALF_ACTUATOR_STROKE_RANGE as STROKE_RANGE,
    CALF_ACTUATOR_ROLL_RANGE as ROLL_RANGE,
    CALF_ACTUATOR_PITCH_RANGE as PITCH_RANGE,
)

ROOT = Path(__file__).parent
XML_PATH = ROOT / "model" / "xml" / "calf_actuator.xml"
LOG_DIR = ROOT / "logs"
ACTUATORS = ("linear_actuator_b1", "linear_actuator_b2")

NAN = float("nan")


# ======================================================================
# 仿真与状态估计
# ======================================================================
class AnkleSim:
    """MuJoCo 模型 + 只依赖编码器传感器的状态估计"""

    def __init__(self, fk: str = "newton", lut_path: Path = DEFAULT_LUT_PATH,
                 xml_path: Path = XML_PATH):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.fk = fk
        # 估计器限位比关节限位宽 CALF_ACTUATOR_ESTIMATOR_MARGIN（见 param.py）
        self.kin: AnkleKinematics = make_kinematics(fk, lut_path=lut_path,
                                                    **calf_actuator_config())
        self.estimate_failures = 0

    def _s(self, name: str) -> float:
        return float(self.data.sensor(name).data[0])

    def set_stroke(self, s1: float, s2: float):
        """设置两根模组的目标行程（m，相对零位长度）"""
        self.data.actuator(ACTUATORS[0]).ctrl[0] = s1
        self.data.actuator(ACTUATORS[1]).ctrl[0] = s2

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def measure_and_estimate(self):
        """读取编码器 → (l1, l2, l1_dot, l2_dot, alpha, theta, alpha_dot, theta_dot)

        只使用 stroke / stroke_vel 传感器，不读取任何关节角度。
        """
        l1 = L0 + self._s("stroke_b1")
        l2 = L0 + self._s("stroke_b2")
        l1_dot = self._s("stroke_vel_b1")
        l2_dot = self._s("stroke_vel_b2")
        try:
            st = self.kin.forward_kinematics(l1, l2)
            q_dot = self.kin.forward_velocity(st.alpha, st.theta, l1_dot, l2_dot)
            est = (st.alpha, st.theta, q_dot[0], q_dot[1])
        except ValueError:
            # 奇异 / 不收敛 / 超出限位：本帧记为 NaN，下一帧从零位初值重新求解
            self.kin.reset()
            self.estimate_failures += 1
            est = (NAN, NAN, NAN, NAN)
        return (l1, l2, l1_dot, l2_dot) + est

    def truth(self):
        """仿真真值 (alpha, theta, alpha_dot, theta_dot)，只用于对照，不参与估计"""
        return (
            self._s("ankle_roll_pos"), self._s("ankle_pitch_pos"),
            self._s("ankle_roll_vel"), self._s("ankle_pitch_vel"),
        )


# ======================================================================
# 控制器：统一接口 poll()（每帧读输入一次）/ command(t, dt)（每个仿真步）
# command 返回 (s1_cmd, s2_cmd, alpha_ref, theta_ref)，无参考角度时为 NaN
# ======================================================================
def _clip_stroke(s: float) -> float:
    return min(max(s, STROKE_RANGE[0]), STROKE_RANGE[1])


class JoystickController:
    """手柄速度控制：摇杆偏转量 → 模组伸缩速度，积分得到目标行程

    松开摇杆时保持当前行程；按住回零键时两根模组以最大速度回到零位。
    """

    def __init__(self, axis1: int, axis2: int, invert: bool, deadzone: float,
                 max_speed: float, home_button: int, index: int = 0):
        # 查看器窗口占据焦点时仍接收手柄输入
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= index:
            pygame.quit()
            raise RuntimeError("未检测到手柄，请确认已连接（ls /dev/input/js*）")

        self.js = pygame.joystick.Joystick(index)
        self.js.init()
        n_axes = self.js.get_numaxes()
        if max(axis1, axis2) >= n_axes:
            raise RuntimeError(f"手柄只有 {n_axes} 个轴，--axis1/--axis2 超出范围")
        print(f"[手柄] {self.js.get_name()}，{n_axes} 个轴，{self.js.get_numbuttons()} 个按键")
        print(f"[手柄] 模组1 ← 轴 {axis1}，模组2 ← 轴 {axis2}，回零键 = 按键 {home_button}")

        self.axis = (axis1, axis2)
        # 大多数手柄摇杆向上推为负值，默认取反使"上推 = 伸长"
        self.sign = -1.0 if invert else 1.0
        self.deadzone = deadzone
        self.max_speed = max_speed
        self.home_button = home_button
        self.rate = [0.0, 0.0]
        self.homing = False
        self.cmd = [0.0, 0.0]

    def _shape(self, v: float) -> float:
        """死区 + 死区外线性重新映射到 [-1, 1]"""
        if abs(v) < self.deadzone:
            return 0.0
        return math.copysign((abs(v) - self.deadzone) / (1.0 - self.deadzone), v)

    def poll(self):
        self.pg.event.pump()
        self.rate = [self.sign * self._shape(self.js.get_axis(a)) * self.max_speed
                     for a in self.axis]
        self.homing = (self.home_button < self.js.get_numbuttons()
                       and self.js.get_button(self.home_button))

    def command(self, t: float, dt: float):
        for i in range(2):
            if self.homing:
                step = self.max_speed * dt
                self.cmd[i] -= max(-step, min(step, self.cmd[i]))
            else:
                self.cmd[i] = _clip_stroke(self.cmd[i] + self.rate[i] * dt)
        return self.cmd[0], self.cmd[1], NAN, NAN

    def status(self) -> str:
        return (f"速度 [{self.rate[0]*1e3:+5.1f} {self.rate[1]*1e3:+5.1f}] mm/s  "
                f"指令行程 [{self.cmd[0]*1e3:+5.1f} {self.cmd[1]*1e3:+5.1f}] mm"
                + ("  回零中" if self.homing else ""))

    def close(self):
        self.pg.quit()


class TrajectoryController:
    """给定踝关节角度曲线 q_ref(t)，逆解得到模组目标行程 s_i = l_i(q_ref) - L0

    shape:
        sine    roll 与 pitch 各自独立正弦（pitch 频率为 roll 的一半）
        circle  roll / pitch 相位差 90°，脚尖画椭圆（两轴耦合）
        pitch   只做 pitch 正弦（勾脚 / 绷脚）
        roll    只做 roll 正弦（内翻 / 外翻）
    开头 ramp 秒内幅值由 0 平滑增加，避免起步阶跃。
    """

    SHAPES = ("sine", "circle", "pitch", "roll")

    def __init__(self, kin: AnkleKinematics, shape: str, roll_amp: float,
                 pitch_amp: float, freq: float, ramp: float = 1.0):
        if shape not in self.SHAPES:
            raise ValueError(f"未知轨迹 {shape}，可选 {self.SHAPES}")
        self.kin = kin
        self.shape = shape
        self.roll_amp = roll_amp
        self.pitch_amp = pitch_amp
        self.freq = freq
        self.ramp = ramp
        self.last = (0.0, 0.0)

    def reference(self, t: float):
        """参考角度 (alpha_ref, theta_ref)，rad"""
        env = 1.0 if t >= self.ramp else 0.5 - 0.5 * math.cos(math.pi * t / self.ramp)
        w = 2.0 * math.pi * self.freq
        a, p = self.roll_amp * env, self.pitch_amp * env
        if self.shape == "sine":
            return a * math.sin(w * t), p * math.sin(0.5 * w * t)
        if self.shape == "circle":
            return a * math.sin(w * t), p * math.cos(w * t)
        if self.shape == "pitch":
            return 0.0, p * math.sin(w * t)
        return a * math.sin(w * t), 0.0

    def check(self, duration: float, samples: int = 2000):
        """预检：参考角度是否在关节限位内、所需行程是否在模组行程内"""
        worst_q = worst_s = 0.0
        for k in range(samples + 1):
            q = self.reference(duration * k / samples)
            worst_q = max(worst_q, abs(q[0]) / ROLL_RANGE[1], abs(q[1]) / PITCH_RANGE[1])
            l1, l2 = self.kin.inverse_kinematics(*q)
            worst_s = max(worst_s, abs(l1 - L0), abs(l2 - L0))
        print(f"[轨迹] 最大角度 = 限位的 {worst_q*100:.0f}%，"
              f"最大所需行程 = {worst_s*1e3:.1f} mm（模组行程 ±{STROKE_RANGE[1]*1e3:.0f} mm）")
        if worst_q > 1.0 or worst_s > STROKE_RANGE[1]:
            print("[轨迹] ⚠ 超出限位 / 行程，超出部分会被截断，跟踪误差会变大")

    def poll(self):
        pass

    def command(self, t: float, dt: float):
        a_ref, t_ref = self.reference(t)
        l1, l2 = self.kin.inverse_kinematics(a_ref, t_ref)
        self.last = (a_ref, t_ref)
        return _clip_stroke(l1 - L0), _clip_stroke(l2 - L0), a_ref, t_ref

    def status(self) -> str:
        return (f"参考角度 roll {math.degrees(self.last[0]):+6.1f}°  "
                f"pitch {math.degrees(self.last[1]):+6.1f}°")

    def close(self):
        pass


# ======================================================================
# 运行循环与记录
# ======================================================================
FIELDS = (
    "t",
    "s1_cmd", "s2_cmd",                            # 指令行程 (m)
    "l1", "l2", "l1_dot", "l2_dot",                # 编码器：推杆长度 (m)、伸长速度 (m/s)
    "alpha_est", "theta_est", "alpha_dot_est", "theta_dot_est",  # 正解估计 (rad, rad/s)
    "alpha_true", "theta_true", "alpha_dot_true", "theta_dot_true",  # 仿真真值
    "alpha_ref", "theta_ref",                      # 参考角度（trajectory 模式）
)


def run(sim: AnkleSim, ctrl, duration, viewer: bool, realtime: bool,
        log_rate: float, fps: float = 60.0):
    """运行仿真，返回记录的数据行（列顺序见 FIELDS）

    duration=None 时一直运行，直到关闭查看器窗口或 Ctrl+C。
    """
    rows = []
    log_every = max(1, round(1.0 / (log_rate * sim.dt)))
    frame_steps = max(1, round(1.0 / (fps * sim.dt)))
    status_every = max(1, round(0.2 / (frame_steps * sim.dt)))  # 终端状态约 5 Hz

    handle = None
    if viewer:
        import mujoco.viewer
        handle = mujoco.viewer.launch_passive(sim.model, sim.data)

    step = 0
    frame = 0
    wall0 = time.perf_counter()
    try:
        while duration is None or step * sim.dt < duration:
            if handle is not None and not handle.is_running():
                break
            ctrl.poll()

            for _ in range(frame_steps):
                if duration is not None and step * sim.dt >= duration - 1e-9:
                    break
                t = step * sim.dt
                s1, s2, a_ref, t_ref = ctrl.command(t, sim.dt)
                sim.set_stroke(s1, s2)
                sim.step()
                step += 1
                if step % log_every == 0:
                    rows.append((step * sim.dt, s1, s2)
                                + sim.measure_and_estimate() + sim.truth()
                                + (a_ref, t_ref))

            frame += 1
            if handle is not None:
                handle.sync()
            if frame % status_every == 0 and rows:
                r = dict(zip(FIELDS, rows[-1]))
                print(f"\r t={r['t']:6.2f}s  {ctrl.status()}  |  估计 roll "
                      f"{math.degrees(r['alpha_est']):+6.1f}°  pitch "
                      f"{math.degrees(r['theta_est']):+6.1f}°   ", end="", flush=True)

            if realtime:
                lag = step * sim.dt - (time.perf_counter() - wall0)
                if lag > 0:
                    time.sleep(lag)
                elif lag < -4 * frame_steps * sim.dt:
                    # 严重落后（例如拖动窗口）：重设时间基准，不做突发追赶
                    wall0 = time.perf_counter() - step * sim.dt
    except KeyboardInterrupt:
        print("\n[结束] Ctrl+C")
    finally:
        print()
        if handle is not None:
            handle.close()
        ctrl.close()
    return rows


def save_csv(rows, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        writer.writerows(rows)


def _columns(rows):
    return {name: [r[i] for r in rows] for i, name in enumerate(FIELDS)}


def _err_stats(est, ref):
    """(最大绝对误差, RMS)，忽略 NaN"""
    e = [a - b for a, b in zip(est, ref) if not (math.isnan(a) or math.isnan(b))]
    if not e:
        return NAN, NAN
    return max(abs(x) for x in e), math.sqrt(sum(x * x for x in e) / len(e))


def summarize(rows, failures: int):
    c = _columns(rows)
    print(f"[统计] 记录 {len(rows)} 帧，时长 {c['t'][-1]:.2f} s，估计失败 {failures} 次")
    for key, unit, scale in (("alpha", "deg", 180 / math.pi), ("theta", "deg", 180 / math.pi),
                             ("alpha_dot", "deg/s", 180 / math.pi),
                             ("theta_dot", "deg/s", 180 / math.pi)):
        mx, rms = _err_stats(c[f"{key}_est"], c[f"{key}_true"])
        print(f"  估计误差 {key:<9}: 最大 {mx*scale:8.4f} {unit}, RMS {rms*scale:8.4f} {unit}")
    if not all(math.isnan(v) for v in c["alpha_ref"]):
        for key in ("alpha", "theta"):
            mx, rms = _err_stats(c[f"{key}_est"], c[f"{key}_ref"])
            print(f"  跟踪误差 {key:<9}: 最大 {math.degrees(mx):8.4f} deg, "
                  f"RMS {math.degrees(rms):8.4f} deg")


def _setup_matplotlib(show: bool):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    # 选一个系统里已有的中文字体，避免标签显示成方框。
    # Noto CJK 的 .ttc 在 matplotlib 中常被登记为 "Noto Sans CJK JP"，字形同样覆盖中文，
    # 所以按关键字匹配而不是精确名称。
    available = sorted({f.name for f in font_manager.fontManager.ttflist})
    for key in ("Noto Sans CJK", "WenQuanYi", "Microsoft YaHei", "SimHei", "PingFang",
                "Droid Sans Fallback", "Noto Serif CJK", "AR PL"):
        match = next((n for n in available if key in n), None)
        if match:
            plt.rcParams["font.sans-serif"] = [match] + plt.rcParams["font.sans-serif"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    # 字号与线宽：默认值在大图上显得太小
    plt.rcParams.update({
        "font.size": 15,
        "axes.labelsize": 16,
        "xtick.labelsize": 14,
        "ytick.labelsize": 14,
        "legend.fontsize": 13,
        "figure.titlesize": 19,
        "lines.linewidth": 2.0,
    })
    return plt


def _finish(plt, fig, axes, title: str, path: Path):
    """统一收尾：网格、图例、标题、横轴标签，并保存"""
    fig.suptitle(title)
    for a in axes:
        a.grid(True, alpha=0.3)
        a.legend(loc="best", framealpha=0.85)
    axes[-1].set_xlabel("时间 (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"[输出] 曲线图 {path}")


def plot(rows, title: str, out_dir: Path, show: bool):
    """每类数据单独一张图，保存到 out_dir：
    actuator.png  推杆长度 / 伸长速度
    angle.png     roll / pitch 角度（真值、估计、参考）
    velocity.png  roll / pitch 角速度（真值、估计）
    error.png     角度估计误差 / 角速度估计误差 [/ 跟踪误差]
    """
    plt = _setup_matplotlib(show)
    c = _columns(rows)
    t = c["t"]
    deg = lambda xs: [math.degrees(x) for x in xs]
    mm = lambda xs: [x * 1e3 for x in xs]
    diff = lambda a, b: [x - y for x, y in zip(a, b)]
    has_ref = not all(math.isnan(v) for v in c["alpha_ref"])
    joints = (("alpha", "roll α"), ("theta", "pitch θ"))

    # 1. 驱动空间：推杆长度与伸长速度
    fig, ax = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
    for i, color in ((1, "tab:blue"), (2, "tab:orange")):
        ax[0].plot(t, mm(c[f"l{i}"]), color=color, label=f"l{i} 编码器")
        ax[0].plot(t, mm([L0 + s for s in c[f"s{i}_cmd"]]), "--", color=color,
                   lw=1.5, label=f"l{i} 指令")
        ax[1].plot(t, mm(c[f"l{i}_dot"]), color=color, label=f"模组{i}")
    ax[0].set_ylabel("推杆长度 (mm)")
    ax[1].set_ylabel("伸长速度 (mm/s)")
    _finish(plt, fig, ax, f"{title}\n直线模组", out_dir / "actuator.png")

    # 2. 关节角度 / 3. 关节角速度
    for suffix, unit, name, fname in (("", "deg", "关节角度", "angle.png"),
                                      ("_dot", "deg/s", "关节角速度", "velocity.png")):
        fig, ax = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
        for a, (key, label) in zip(ax, joints):
            a.plot(t, deg(c[f"{key}{suffix}_true"]), color="0.6", lw=6, alpha=0.6,
                   label="仿真真值")
            a.plot(t, deg(c[f"{key}{suffix}_est"]), color="tab:green", lw=2,
                   label="正解估计")
            if has_ref and suffix == "":
                a.plot(t, deg(c[f"{key}_ref"]), "k--", lw=1.6, label="参考轨迹")
            a.set_ylabel(f"{label} ({unit})")
        _finish(plt, fig, ax, f"{title}\n{name}", out_dir / fname)

    # 4. 误差：估计误差（估计 - 真值），轨迹模式另加跟踪误差（估计 - 参考）
    panels = [("", "角度估计误差 (deg)"), ("_dot", "角速度估计误差 (deg/s)")]
    fig, ax = plt.subplots(len(panels) + has_ref, 1,
                           figsize=(16, 5 * (len(panels) + has_ref)), sharex=True)
    for a, (suffix, ylabel) in zip(ax, panels):
        for key, label in joints:
            a.plot(t, deg(diff(c[f"{key}{suffix}_est"], c[f"{key}{suffix}_true"])),
                   label=label)
        a.set_ylabel(ylabel)
    if has_ref:
        for key, label in joints:
            ax[-1].plot(t, deg(diff(c[f"{key}_est"], c[f"{key}_ref"])), label=label)
        ax[-1].set_ylabel("跟踪误差 (deg)")
    _finish(plt, fig, ax, f"{title}\n误差", out_dir / "error.png")

    if show:
        plt.show()


# ======================================================================
# 命令行入口
# ======================================================================
def parse_args():
    p = argparse.ArgumentParser(description="踝关节直线模组仿真控制与记录")
    sub = p.add_subparsers(dest="mode", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 查看器")
    common.add_argument("--no-show", action="store_true", help="只保存曲线图，不弹出窗口")
    common.add_argument("--log-rate", type=float, default=200.0, help="记录频率 Hz（默认 200）")
    common.add_argument("--out", type=Path, default=None,
                        help="输出目录（默认 logs/<时间>）")
    common.add_argument("--fk", choices=FK_METHODS, default="lut-newton",
                        help="位置正解方法：newton 迭代 / lut 查表插值 / "
                             "lut-newton 查表初值 + Newton 精修（默认 lut-newton）")
    common.add_argument("--lut", type=Path, default=DEFAULT_LUT_PATH,
                        help="查表文件（默认 model/lut/calf_actuator_fk_lut.npz）")

    js = sub.add_parser("joystick", parents=[common], help="手柄控制两根直线模组")
    js.add_argument("--duration", type=float, default=None,
                    help="运行时长 s（默认一直运行，关闭查看器或 Ctrl+C 结束）")
    js.add_argument("--axis1", type=int, default=1, help="模组 1 对应的轴（默认 1：左摇杆上下）")
    js.add_argument("--axis2", type=int, default=4, help="模组 2 对应的轴（默认 4：右摇杆上下）")
    js.add_argument("--no-invert", action="store_true", help="不取反（默认上推 = 伸长）")
    js.add_argument("--deadzone", type=float, default=0.08, help="摇杆死区（默认 0.08）")
    js.add_argument("--max-speed", type=float, default=0.05,
                    help="摇杆推满时的伸缩速度 m/s（默认 0.05）")
    js.add_argument("--home-button", type=int, default=0, help="回零键编号（默认 0，Xbox 的 A）")
    js.add_argument("--index", type=int, default=0, help="手柄编号（默认 0）")

    tr = sub.add_parser("trajectory", parents=[common], help="给定踝关节角度曲线")
    tr.add_argument("--shape", choices=TrajectoryController.SHAPES, default="sine")
    tr.add_argument("--duration", type=float, default=10.0, help="运行时长 s（默认 10）")
    tr.add_argument("--roll-amp", type=float, default=20.0, help="roll 幅值 deg（默认 20）")
    tr.add_argument("--pitch-amp", type=float, default=25.0, help="pitch 幅值 deg（默认 25）")
    tr.add_argument("--freq", type=float, default=0.5, help="频率 Hz（默认 0.5）")
    tr.add_argument("--ramp", type=float, default=1.0, help="起步渐入时间 s（默认 1）")
    tr.add_argument("--fast", action="store_true",
                    help="不按实时速度运行（无查看器时默认就不按实时）")
    return p.parse_args()


def main():
    args = parse_args()
    sim = AnkleSim(args.fk, args.lut)
    print(f"[正解] {args.fk}" + (f"，查表文件 {args.lut}" if args.fk != "newton" else ""))
    viewer = not args.no_viewer

    if args.mode == "joystick":
        ctrl = JoystickController(args.axis1, args.axis2, not args.no_invert, args.deadzone,
                                  args.max_speed, args.home_button, args.index)
        realtime = True  # 人在回路，始终按实时速度
        title = "手柄控制（驱动空间）"
        if not viewer and args.duration is None:
            print("[提示] 无查看器且未指定 --duration，按 Ctrl+C 结束")
    else:
        ctrl = TrajectoryController(sim.kin, args.shape, math.radians(args.roll_amp),
                                    math.radians(args.pitch_amp), args.freq, args.ramp)
        ctrl.check(args.duration)
        realtime = viewer and not args.fast
        title = (f"关节轨迹控制：{args.shape}，roll ±{args.roll_amp:g}°，"
                 f"pitch ±{args.pitch_amp:g}°，{args.freq:g} Hz")

    title += f"，正解 {args.fk}"
    rows = run(sim, ctrl, args.duration, viewer, realtime, args.log_rate)
    if not rows:
        print("[结束] 没有记录到数据")
        return

    out_dir = args.out or LOG_DIR / f"{datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "data.csv"
    save_csv(rows, csv_path)
    print(f"[输出] 数据 {csv_path}")
    summarize(rows, sim.estimate_failures)
    plot(rows, title, out_dir, show=not args.no_show)


if __name__ == "__main__":
    main()
