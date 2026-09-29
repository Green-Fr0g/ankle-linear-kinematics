"""MuJoCo 模型封装 + 只依赖编码器传感器的状态估计"""
from pathlib import Path

import mujoco

from ankle import AnkleKinematics, calf_actuator_config, make_kinematics
from ankle.params import CALF_ACTUATOR_NOMINAL_LENGTH as L0
from ankle.paths import LUT_PATH, XML_PATH

ACTUATORS = ("linear_actuator_b1", "linear_actuator_b2")

NAN = float("nan")


class AnkleSim:
    """MuJoCo 模型 + 只依赖编码器传感器的状态估计"""

    def __init__(self, fk: str = "newton", lut_path: Path = LUT_PATH,
                 xml_path: Path = XML_PATH):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.fk = fk
        # 估计器限位比关节限位宽 CALF_ACTUATOR_ESTIMATOR_MARGIN（见 ankle/params.py）
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
