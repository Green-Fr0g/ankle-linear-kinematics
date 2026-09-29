"""
踝关节直线模组机构几何参数定义

坐标约定（与 model/xml/calf_actuator.xml 一致）：
    原点：踝关节旋转中心；坐标轴方向与 XML 相同（z 轴向上）
    所有参数都是带符号的真实坐标值（单位：米），公式中不再额外加负号。

    a1 = (ax,  ay, az)    a2 = (ax, -ay, az)    # 小腿上的固定安装点（踝中心系）
    b1 = (bx,  by, bz)    b2 = (bx, -by, bz)    # 脚踝上的安装点（脚踝局部系）

    下标 1 位于 +y 侧（XML 中 actuator_b1），下标 2 位于 -y 侧（actuator_b2）。
"""
import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class AnkleGeometry:
    """踝关节机构几何参数（带符号坐标）

    Attributes:
        ax, ay, az: a1 在踝中心系下的坐标（a2 为其关于 xz 平面的镜像）
        bx, by, bz: b1 在脚踝局部系下的坐标（b2 为其关于 xz 平面的镜像）
    """
    ax: float
    ay: float
    az: float
    bx: float
    by: float
    bz: float

    def __post_init__(self):
        """验证参数有效性"""
        values = (self.ax, self.ay, self.az, self.bx, self.by, self.bz)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("几何参数必须为有限值")
        if self.ay <= 0 or self.by <= 0:
            raise ValueError("ay, by 必须为正值（下标 1 的安装点位于 +y 侧）")

    @property
    def a1(self) -> Vec3:
        return (self.ax, self.ay, self.az)

    @property
    def a2(self) -> Vec3:
        return (self.ax, -self.ay, self.az)

    @property
    def b1(self) -> Vec3:
        return (self.bx, self.by, self.bz)

    @property
    def b2(self) -> Vec3:
        return (self.bx, -self.by, self.bz)


@dataclass
class AnkleState:
    """踝关节状态

    Attributes:
        alpha: roll 角 (弧度)，ankle_roll_joint，绕 x 轴
        theta: pitch 角 (弧度)，ankle_pitch_joint，绕 y 轴
    """
    alpha: float
    theta: float

    def to_degrees(self) -> tuple[float, float]:
        """转换为角度制

        Returns:
            (alpha_deg, theta_deg): 角度制的 roll 和 pitch 角
        """
        return (math.degrees(self.alpha), math.degrees(self.theta))
