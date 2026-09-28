"""
踝关节直线模组机构标准参数配置
"""
from ankle_geometry import AnkleGeometry


# 标准机构几何参数（单位：米）
STANDARD_GEOMETRY = AnkleGeometry(
    m=0.05,   # 小腿固定端 x 偏置
    n=0.045,   # 小腿固定端 y 偏置
    h=0.30,   # 小腿固定端 z 偏置
    c=0.06,   # 脚踝连接点 x 偏置
    d=0.0375,   # 脚踝连接点 y 偏置
    h2=0.0   # 脚踝 z 方向偏置
)


# 另一组示例参数
COMPACT_GEOMETRY = AnkleGeometry(
    m=0.04,
    n=0.05,
    h=0.25,
    c=0.03,
    d=0.04,
    h2=0.015
)


def get_geometry(name: str = "standard") -> AnkleGeometry:
    """获取指定名称的几何参数配置

    Args:
        name: 配置名称 ("standard" 或 "compact")

    Returns:
        AnkleGeometry: 几何参数对象

    Raises:
        ValueError: 如果配置名称不存在
    """
    configs = {
        "standard": STANDARD_GEOMETRY,
        "compact": COMPACT_GEOMETRY
    }

    if name not in configs:
        raise ValueError(
            f"未知的配置名称: {name}. "
            f"可用配置: {', '.join(configs.keys())}"
        )

    return configs[name]
