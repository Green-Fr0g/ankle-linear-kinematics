"""
踝关节位置正解：查表 + 双线性插值

思路参考 agibot_x1_infer 的 AnkleParallelTransmission：离线把驱动空间 (l1, l2)
划成规则网格，逐点求出关节角 (alpha, theta) 存表；在线只做一次查表插值，
不迭代，耗时固定、不会收敛到伪解。

与参考实现的差异（修正其问题）：
    - 四角点双线性插值（参考实现只用对角两点，误差约大 200 倍且跨格线跳变）
    - 表存为平铺数组，按下标直接取值（参考实现用字符串键哈希表）
    - 超出表格或落入不可达区域时抛出 ValueError（参考实现静默输出 0）

网格节点用 AnkleKinematics.forward_kinematics（Newton）离线求解，建表时限位
比使用时放宽 BUILD_MARGIN，保证限位边界附近的格子四个角点都有值。

速度、力映射与逆解仍使用解析公式（继承自 AnkleKinematics），只替换位置正解。
"""
import math
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

from ankle_geometry import AnkleGeometry, AnkleState
from ankle_kinematics import AnkleKinematics

Vec2 = Tuple[float, float]

# 建表时关节限位额外放宽的量（rad），让限位附近的格子角点可用
BUILD_MARGIN = 0.1


def _widen(rng: Vec2, margin: float) -> Vec2:
    return (rng[0] - margin, rng[1] + margin)


class AnkleLUT:
    """驱动空间 (l1, l2) 上的规则网格表：节点值为 (alpha, theta)，不可达节点为 NaN"""

    def __init__(self, l_min: float, step: float, alpha: np.ndarray, theta: np.ndarray,
                 meta: Optional[dict] = None):
        """
        Args:
            l_min: 网格起点长度（l1、l2 相同），米
            step: 网格步长，米
            alpha, theta: 形状 (n, n) 的节点表，[i, j] 对应 l1 = l_min + i·step, l2 = l_min + j·step
            meta: 建表参数（几何、限位等），仅用于记录与校验
        """
        if alpha.shape != theta.shape or alpha.ndim != 2 or alpha.shape[0] != alpha.shape[1]:
            raise ValueError("alpha / theta 必须是相同形状的方阵")
        self.l_min = float(l_min)
        self.step = float(step)
        self.n = alpha.shape[0]
        self.l_max = self.l_min + (self.n - 1) * self.step
        self.meta = meta or {}
        # 平铺成 Python 列表：逐点取值比 numpy 标量索引快一个数量级
        self._a = alpha.ravel().tolist()
        self._t = theta.ravel().tolist()
        self.alpha = alpha
        self.theta = theta

    # ------------------------------------------------------------------
    # 建表 / 存取
    # ------------------------------------------------------------------
    @classmethod
    def build(cls, geometry: AnkleGeometry, length_range: Vec2, step: float,
              alpha_range: Vec2, theta_range: Vec2) -> "AnkleLUT":
        """离线建表：蛇形扫描网格，用相邻节点的解作 Newton 初值

        Args:
            geometry: 机构几何参数
            length_range: 推杆长度范围 (l_min, l_max)，l1、l2 共用
            step: 网格步长（米），会微调为能整除长度范围
            alpha_range, theta_range: 关节限位（使用时的限位，内部再放宽 BUILD_MARGIN）
        """
        l_min, l_max = length_range
        n = int(round((l_max - l_min) / step)) + 1
        step = (l_max - l_min) / (n - 1)
        solver = AnkleKinematics(geometry, _widen(alpha_range, BUILD_MARGIN),
                                 _widen(theta_range, BUILD_MARGIN))
        alpha = np.full((n, n), np.nan)
        theta = np.full((n, n), np.nan)
        for i in range(n):
            # 蛇形顺序：相邻两次求解的节点在网格上相邻，热启动初值最接近
            cols = range(n) if i % 2 == 0 else range(n - 1, -1, -1)
            for j in cols:
                try:
                    st = solver.forward_kinematics(l_min + i * step, l_min + j * step)
                except ValueError:
                    solver.reset()  # 不可达：记 NaN，下一点从零位重解
                    continue
                alpha[i, j], theta[i, j] = st.alpha, st.theta
        meta = {
            "geometry": [geometry.ax, geometry.ay, geometry.az,
                         geometry.bx, geometry.by, geometry.bz],
            "alpha_range": list(alpha_range),
            "theta_range": list(theta_range),
            "build_margin": BUILD_MARGIN,
        }
        return cls(l_min, step, alpha, theta, meta)

    def save(self, path: Path):
        """保存为 .npz（便于移植到嵌入式时导出成数组）"""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, l_min=self.l_min, step=self.step, alpha=self.alpha,
                            theta=self.theta, **{f"meta_{k}": v for k, v in self.meta.items()})

    @classmethod
    def load(cls, path: Path) -> "AnkleLUT":
        with np.load(Path(path)) as f:
            meta = {k[5:]: f[k].tolist() for k in f.files if k.startswith("meta_")}
            return cls(float(f["l_min"]), float(f["step"]), f["alpha"], f["theta"], meta)

    # ------------------------------------------------------------------
    # 查表
    # ------------------------------------------------------------------
    def lookup(self, l1: float, l2: float) -> Vec2:
        """双线性插值 (l1, l2) → (alpha, theta)

        Raises:
            ValueError: 超出表格范围，或所在格子有不可达（NaN）角点
        """
        eps = 1e-12
        if not (self.l_min - eps <= l1 <= self.l_max + eps
                and self.l_min - eps <= l2 <= self.l_max + eps):
            raise ValueError(
                f"推杆长度超出查表范围 [{self.l_min:.4f}, {self.l_max:.4f}]: l1={l1}, l2={l2}")
        u = (l1 - self.l_min) / self.step
        v = (l2 - self.l_min) / self.step
        # 落在最后一条格线上时归到最后一个格子
        i = min(int(u), self.n - 2)
        j = min(int(v), self.n - 2)
        fu, fv = u - i, v - j

        k00 = i * self.n + j
        k10 = k00 + self.n
        a, t = self._a, self._t
        w00 = (1 - fu) * (1 - fv)
        w10 = fu * (1 - fv)
        w01 = (1 - fu) * fv
        w11 = fu * fv
        alpha = w00 * a[k00] + w10 * a[k10] + w01 * a[k00 + 1] + w11 * a[k10 + 1]
        theta = w00 * t[k00] + w10 * t[k10] + w01 * t[k00 + 1] + w11 * t[k10 + 1]
        if math.isnan(alpha) or math.isnan(theta):
            raise ValueError(f"推杆长度落入不可达区域: l1={l1}, l2={l2}")
        return alpha, theta

    @property
    def valid_ratio(self) -> float:
        """可达节点占比"""
        return float(np.count_nonzero(~np.isnan(self.alpha))) / self.alpha.size


class LutKinematics(AnkleKinematics):
    """位置正解改为查表插值，其余（逆解、速度、力、奇异检测）沿用解析公式"""

    def __init__(self, geometry: AnkleGeometry, alpha_range: Vec2, theta_range: Vec2,
                 length_range: Vec2, step: float = 1e-3, lut: Optional[AnkleLUT] = None):
        """
        Args:
            geometry: 机构几何参数
            alpha_range, theta_range: 关节限位，查表结果超出时抛出 ValueError
            length_range: 建表的推杆长度范围 (l_min, l_max)
            step: 网格步长（米），默认 1 mm
            lut: 已建好的表；None 时现场建表
        """
        super().__init__(geometry, alpha_range, theta_range)
        self.lut = lut or AnkleLUT.build(geometry, length_range, step, alpha_range, theta_range)

    def forward_kinematics(self, l1: float, l2: float, alpha_init: Optional[float] = None,
                           theta_init: Optional[float] = None, max_iter: int = 50,
                           tolerance: float = 1e-9) -> AnkleState:
        """位置正解：查表插值，不迭代（初值、迭代参数保留只为接口兼容，均被忽略）"""
        if not (math.isfinite(l1) and math.isfinite(l2)):
            raise ValueError("l1 和 l2 必须为有限数")
        alpha, theta = self.lut.lookup(l1, l2)
        if not self._in_range(alpha, theta):
            raise ValueError(f"解超出关节限位: alpha={alpha}, theta={theta}")
        self.alpha_last, self.theta_last = alpha, theta
        return AnkleState(alpha=alpha, theta=theta)


# ======================================================================
# 正解方法选择（sim_control / bench_fk 共用）
# ======================================================================
FK_METHODS = ("newton", "lut")


def make_kinematics(method: str, geometry: AnkleGeometry, alpha_range: Vec2,
                    theta_range: Vec2, length_range: Vec2,
                    step: float = 1e-3) -> AnkleKinematics:
    """按方法名构造运动学对象

    newton: Newton 迭代（热启动 + 零位回退）
    lut:    查表 + 双线性插值
    """
    if method == "newton":
        return AnkleKinematics(geometry, alpha_range, theta_range)
    if method == "lut":
        return LutKinematics(geometry, alpha_range, theta_range, length_range, step)
    raise ValueError(f"未知的正解方法: {method}，可选 {', '.join(FK_METHODS)}")
