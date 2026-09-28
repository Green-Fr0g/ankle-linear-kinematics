"""
踝关节直线模组机构几何参数定义
"""
from dataclasses import dataclass


@dataclass
class AnkleGeometry:
    """踝关节机构几何参数

    Attributes:
        m: 小腿固定端连接点 x 方向偏置
        n: 小腿固定端连接点 y 方向偏置
        h: 小腿固定端连接点 z 方向偏置
        c: 脚踝局部坐标系连接点 x 方向偏置
        d: 脚踝局部坐标系连接点 y 方向偏置
        h2: 脚踝连接点相对于脚踝旋转中心的局部 z 方向偏置
    """
    m: float
    n: float
    h: float
    c: float
    d: float
    h2: float

    def __post_init__(self):
        """验证参数有效性"""
        if self.m <= 0 or self.n <= 0 or self.h <= 0:
            raise ValueError("m, n, h 必须为正值")
        if self.c <= 0 or self.d <= 0:
            raise ValueError("c, d 必须为正值")


@dataclass
class AnkleState:
    """踝关节状态

    Attributes:
        alpha: roll 角 (弧度)
        theta: pitch 角 (弧度)
    """
    alpha: float
    theta: float

    def to_degrees(self) -> tuple[float, float]:
        """转换为角度制

        Returns:
            (alpha_deg, theta_deg): 角度制的 roll 和 pitch 角
        """
        import math
        return (math.degrees(self.alpha), math.degrees(self.theta))
