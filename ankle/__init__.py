"""踝关节直线模组并联机构运动学库（纯 Python，不依赖 MuJoCo）

常用入口：
    from ankle import make_kinematics, calf_actuator_config

    kin = make_kinematics("lut-newton", **calf_actuator_config())
    st = kin.forward_kinematics(l1, l2)            # 推杆长度 → 关节角
    l1, l2 = kin.inverse_kinematics(alpha, theta)   # 关节角 → 推杆长度
"""
from .factory import FK_METHODS, load_lut, make_kinematics
from .geometry import AnkleGeometry, AnkleState
from .kinematics import AnkleKinematics, SingularityError
from .lut import AnkleLUT, LutKinematics, LutNewtonKinematics
from .params import calf_actuator_config, get_geometry
from .paths import LUT_PATH, ROOT, XML_PATH

__all__ = [
    "FK_METHODS", "load_lut", "make_kinematics",
    "AnkleGeometry", "AnkleState",
    "AnkleKinematics", "SingularityError",
    "AnkleLUT", "LutKinematics", "LutNewtonKinematics",
    "calf_actuator_config", "get_geometry",
    "LUT_PATH", "ROOT", "XML_PATH",
]
