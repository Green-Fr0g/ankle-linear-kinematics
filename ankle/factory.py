"""按名称选择位置正解方法（仿真、对比脚本共用）

    newton      Newton 迭代（热启动 + 零位回退）
    lut         查表 + 双线性插值，不迭代
    lut-newton  查表初值 + Newton 精修（推荐）
"""
from pathlib import Path
from typing import Optional, Tuple

from .geometry import AnkleGeometry
from .kinematics import AnkleKinematics
from .lut import AnkleLUT, LutKinematics, LutNewtonKinematics
from .paths import LUT_PATH

Vec2 = Tuple[float, float]

FK_METHODS = ("newton", "lut", "lut-newton")


def load_lut(path: Path, geometry: AnkleGeometry, alpha_range: Vec2, theta_range: Vec2,
             length_range: Vec2) -> AnkleLUT:
    """从文件读取查表并校验与当前参数一致"""
    lut = AnkleLUT.load(path)
    lut.check_matches(geometry, alpha_range, theta_range, length_range)
    return lut


def make_kinematics(method: str, geometry: AnkleGeometry, alpha_range: Vec2,
                    theta_range: Vec2, length_range: Vec2,
                    lut_path: Optional[Path] = LUT_PATH,
                    step: float = 1e-3) -> AnkleKinematics:
    """按方法名构造运动学对象

    Args:
        method: FK_METHODS 之一
        lut_path: 查表文件；None 时按 step 现场建表（仅用于对比不同步长）
        step: 现场建表的网格步长（米）
    """
    if method not in FK_METHODS:
        raise ValueError(f"未知的正解方法: {method}，可选 {', '.join(FK_METHODS)}")
    if method == "newton":
        return AnkleKinematics(geometry, alpha_range, theta_range)
    lut = (load_lut(lut_path, geometry, alpha_range, theta_range, length_range)
           if lut_path is not None else None)
    cls = LutKinematics if method == "lut" else LutNewtonKinematics
    return cls(geometry, alpha_range, theta_range, length_range, step, lut)
