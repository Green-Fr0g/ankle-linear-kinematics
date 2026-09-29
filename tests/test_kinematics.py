"""运动学库单元测试（标准库 unittest，无额外依赖）

运行：.venv/bin/python -m unittest discover -s tests -v
"""
import math
import random
import unittest

from ankle import FK_METHODS, AnkleKinematics, calf_actuator_config, make_kinematics
from ankle.params import (
    CALF_ACTUATOR_LENGTH_RANGE as LENGTH_RANGE,
    CALF_ACTUATOR_NOMINAL_LENGTH as L0,
    CALF_ACTUATOR_PITCH_RANGE as PITCH_RANGE,
    CALF_ACTUATOR_ROLL_RANGE as ROLL_RANGE,
)
from ankle.paths import XML_PATH

CFG = calf_actuator_config()


def sample_poses(kin, n=300, seed=1):
    """限位内且推杆长度在行程内的随机姿态"""
    rng = random.Random(seed)
    out = []
    while len(out) < n:
        a, t = rng.uniform(*ROLL_RANGE), rng.uniform(*PITCH_RANGE)
        l1, l2 = kin.inverse_kinematics(a, t)
        if all(LENGTH_RANGE[0] <= x <= LENGTH_RANGE[1] for x in (l1, l2)):
            out.append((a, t))
    return out


class TestAnalytic(unittest.TestCase):
    def setUp(self):
        self.kin = AnkleKinematics(CFG["geometry"], CFG["alpha_range"], CFG["theta_range"])
        self.poses = sample_poses(self.kin)

    def test_zero_pose_length(self):
        l1, l2 = self.kin.inverse_kinematics(0.0, 0.0)
        self.assertAlmostEqual(l1, L0, places=9)
        self.assertAlmostEqual(l2, L0, places=9)

    def test_jacobian_matches_finite_difference(self):
        h = 1e-6
        for a, t in self.poses[:50]:
            J = self.kin.jacobian(a, t)
            for col, (da, dt) in enumerate(((h, 0.0), (0.0, h))):
                lp = self.kin.inverse_kinematics(a + da, t + dt)
                lm = self.kin.inverse_kinematics(a - da, t - dt)
                for row in range(2):
                    self.assertAlmostEqual(J[row][col], (lp[row] - lm[row]) / (2 * h), places=6)

    def test_velocity_roundtrip(self):
        for a, t in self.poses[:50]:
            l_dot = self.kin.inverse_velocity(a, t, 0.3, -0.7)
            q_dot = self.kin.forward_velocity(a, t, *l_dot)
            self.assertAlmostEqual(q_dot[0], 0.3, places=9)
            self.assertAlmostEqual(q_dot[1], -0.7, places=9)

    def test_force_torque_roundtrip_and_virtual_work(self):
        for a, t in self.poses[:50]:
            f = self.kin.torque_to_force(a, t, 5.0, -3.0)
            tau = self.kin.force_to_torque(a, t, *f)
            self.assertAlmostEqual(tau[0], 5.0, places=8)
            self.assertAlmostEqual(tau[1], -3.0, places=8)
            # 虚功：Fᵀ l̇ = τᵀ q̇
            l_dot = self.kin.inverse_velocity(a, t, 0.2, 0.4)
            self.assertAlmostEqual(f[0] * l_dot[0] + f[1] * l_dot[1],
                                   5.0 * 0.2 - 3.0 * 0.4, places=8)


class TestForwardKinematics(unittest.TestCase):
    """三种位置正解都应还原 IK 的输入"""

    # Newton 收敛判据为长度残差 1e-9 m，除以雅可比（约 0.03 m/rad）后角度误差约 3e-8 rad
    TOL = {"newton": 1e-7, "lut-newton": 1e-7, "lut": math.radians(0.02)}

    @classmethod
    def setUpClass(cls):
        cls.kins = {m: make_kinematics(m, **CFG) for m in FK_METHODS}
        cls.poses = sample_poses(cls.kins["newton"])

    def test_roundtrip_cold_start(self):
        for m, kin in self.kins.items():
            with self.subTest(method=m):
                for a, t in self.poses:
                    kin.reset()
                    st = kin.forward_kinematics(*kin.inverse_kinematics(a, t))
                    self.assertLess(abs(st.alpha - a), self.TOL[m])
                    self.assertLess(abs(st.theta - t), self.TOL[m])

    def test_out_of_table_raises(self):
        kin = self.kins["lut"]
        with self.assertRaises(ValueError):
            kin.forward_kinematics(LENGTH_RANGE[1] + 0.01, L0)

    def test_unknown_method(self):
        with self.assertRaises(ValueError):
            make_kinematics("foo", **CFG)


class TestLutFile(unittest.TestCase):
    def test_mismatched_geometry_rejected(self):
        from ankle import AnkleGeometry, load_lut
        from ankle.paths import LUT_PATH
        g = CFG["geometry"]
        other = AnkleGeometry(g.ax, g.ay, g.az + 0.001, g.bx, g.by, g.bz)
        with self.assertRaises(ValueError):
            load_lut(LUT_PATH, other, CFG["alpha_range"], CFG["theta_range"],
                     CFG["length_range"])

    def test_missing_file(self):
        from ankle import AnkleLUT
        with self.assertRaises(FileNotFoundError):
            AnkleLUT.load(XML_PATH.parent / "no_such_lut.npz")


class TestMatchesXml(unittest.TestCase):
    """IK 算出的推杆长度应与 MuJoCo 模型中 |a_i - b_i| 一致（验证 params.py 与 XML 对齐）"""

    def test_site_distance(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("未安装 mujoco")
        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        data = mujoco.MjData(model)
        kin = AnkleKinematics(CFG["geometry"])
        qa = model.joint("ankle_roll_joint").qposadr[0]
        qt = model.joint("ankle_pitch_joint").qposadr[0]
        for a, t in sample_poses(kin, n=30):
            data.qpos[:] = model.qpos0
            data.qpos[qa], data.qpos[qt] = a, t
            mujoco.mj_kinematics(model, data)
            expect = kin.inverse_kinematics(a, t)
            for i, l_ik in zip((1, 2), expect):
                d = data.site(f"a{i}").xpos - data.site(f"b{i}").xpos
                self.assertAlmostEqual(float((d @ d) ** 0.5), l_ik, places=9)


if __name__ == "__main__":
    unittest.main()
