"""控制器：统一接口 poll()（每帧读输入一次）/ command(t, dt)（每个仿真步）/ status() / close()

command 返回 (s1_cmd, s2_cmd, alpha_ref, theta_ref)，s 为相对零位的目标行程 (m)，
无参考角度时 alpha_ref / theta_ref 为 NaN。
    JoystickController    手柄直接控制两根模组（驱动空间）
    TrajectoryController  给定关节角度曲线，逆解得到模组行程（关节空间）
"""
import math
import os

from ankle import AnkleKinematics
from ankle.params import (
    CALF_ACTUATOR_NOMINAL_LENGTH as L0,
    CALF_ACTUATOR_STROKE_RANGE as STROKE_RANGE,
    CALF_ACTUATOR_ROLL_RANGE as ROLL_RANGE,
    CALF_ACTUATOR_PITCH_RANGE as PITCH_RANGE,
)

NAN = float("nan")


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
