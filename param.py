"""
踝关节直线模组机构参数配置

坐标约定见 ankle_geometry.py：以踝关节旋转中心为原点，参数为带符号的真实坐标（米）。
    a1 = (ax,  ay, az)    a2 = (ax, -ay, az)
    b1 = (bx,  by, bz)    b2 = (bx, -by, bz)
"""
from ankle_geometry import AnkleGeometry


# 标准机构几何参数（单位：米）
STANDARD_GEOMETRY = AnkleGeometry(
    ax=-0.05, ay=0.045, az=0.30,
    bx=-0.06, by=0.0375, bz=0.0,
)


# 另一组示例参数
COMPACT_GEOMETRY = AnkleGeometry(
    ax=-0.04, ay=0.05, az=0.25,
    bx=-0.03, by=0.04, bz=-0.015,
)


# ==================== model/xml/calf_actuator.xml 对应参数 ====================
# 踝关节旋转中心：ankle_pitch_Link 在 calf_link 下的 pos
CALF_ACTUATOR_ANKLE_CENTER = (0.049, 0.0, -0.378)

# a1：XML calf_link 下 site a1 = (-0.012, 0.0405, -0.087)，减去踝中心
#     = (-0.012-0.049, 0.0405, -0.087+0.378) = (-0.061, 0.0405, 0.291)
# b1：XML ankle_roll_Link 下 site b1 = (-0.075, 0.0405, 0.001)，直接使用
#     （bz=+0.001 表示 b 点比旋转中心高 1 mm）
CALF_ACTUATOR_GEOMETRY = AnkleGeometry(
    ax=-0.061, ay=0.0405, az=0.291,
    bx=-0.075, by=0.0405, bz=0.001,
)

# 零位推杆长度 |a1 - b1|（与 XML 中 actuator_link 相对 actuator 的 pos (-0.014, 0, -0.29) 一致）
CALF_ACTUATOR_NOMINAL_LENGTH = (0.014 ** 2 + 0.290 ** 2) ** 0.5  # ≈ 0.290338 m

# 直线模组行程（actuator_slide 关节 range / ctrlrange），相对零位长度，单位 m
# 注意：slide 轴方向为 a→b，正行程 = 推杆伸长
CALF_ACTUATOR_STROKE_RANGE = (-0.04, 0.04)

# 推杆长度范围 [l_min, l_max]
CALF_ACTUATOR_LENGTH_RANGE = (
    CALF_ACTUATOR_NOMINAL_LENGTH + CALF_ACTUATOR_STROKE_RANGE[0],
    CALF_ACTUATOR_NOMINAL_LENGTH + CALF_ACTUATOR_STROKE_RANGE[1],
)

# 踝关节角度限位（rad）
CALF_ACTUATOR_ROLL_RANGE = (-0.87, 0.87)   # ankle_roll_joint
CALF_ACTUATOR_PITCH_RANGE = (-0.87, 0.87)  # ankle_pitch_joint

# 状态估计时的限位余量（rad）：MuJoCo 关节限位是软约束，撞限位时会略微越界
CALF_ACTUATOR_ESTIMATOR_MARGIN = 0.05


def get_geometry(name: str = "standard") -> AnkleGeometry:
    """获取指定名称的几何参数配置

    Args:
        name: 配置名称 ("standard"、"compact" 或 "calf_actuator")

    Returns:
        AnkleGeometry: 几何参数对象

    Raises:
        ValueError: 如果配置名称不存在
    """
    configs = {
        "standard": STANDARD_GEOMETRY,
        "compact": COMPACT_GEOMETRY,
        "calf_actuator": CALF_ACTUATOR_GEOMETRY,
    }

    if name not in configs:
        raise ValueError(
            f"未知的配置名称: {name}. "
            f"可用配置: {', '.join(configs.keys())}"
        )

    return configs[name]
