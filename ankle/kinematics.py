"""
踝关节直线模组并联机构运动学（位置 / 速度 / 力）

命名按并联机构惯例：
    关节空间 q = (alpha, theta)   alpha: roll（绕 x）, theta: pitch（绕 y）
    驱动空间 l = (l1, l2)         l1: +y 侧推杆（actuator_b1）, l2: -y 侧推杆

    |        | 逆运动学 IK：q → l（解析）     | 正运动学 FK：l → q（Newton 迭代） |
    |--------|-------------------------------|----------------------------------|
    | 位置   | inverse_kinematics            | forward_kinematics               |
    | 速度   | inverse_velocity  l̇ = J q̇     | forward_velocity  q̇ = J⁻¹ l̇     |
    | 力     | torque_to_force  F = J⁻ᵀ τ    | force_to_torque  τ = Jᵀ F        |

    其中 J = ∂l/∂q 为 2x2 雅可比（jacobian）。

典型用法：
    状态估计：编码器 (l, l̇, F)   → forward_kinematics / forward_velocity / force_to_torque
    控制下发：规划器 (q*, q̇*, τ*) → inverse_kinematics / inverse_velocity / torque_to_force

几何与旋转约定（与 model/xml/calf_actuator.xml 一致，见 geometry.py）：
    R(alpha, theta) = Ry(theta) · Rx(alpha)     # pitch 为父关节，roll 为子关节
    e_i = a_i - R · b_i,  l_i = |e_i|

符号约定：
    F_i > 0 表示推杆沿伸长方向出力；τ 与 q 同向为正。
    由虚功原理 Fᵀ l̇ = τᵀ q̇ 得 τ = Jᵀ F。
"""
import math
from typing import Optional, Tuple
from .geometry import AnkleGeometry, AnkleState, Vec3

Vec2 = Tuple[float, float]
Mat2 = Tuple[Vec2, Vec2]
Mat3 = Tuple[Vec3, Vec3, Vec3]


class SingularityError(ValueError):
    """雅可比接近奇异，无法求逆"""


# ======================================================================
# 基础数学工具
# ======================================================================
def rotation_matrix(alpha: float, theta: float) -> Mat3:
    """R = Ry(theta) · Rx(alpha)

    Args:
        alpha: roll 角（绕 x 轴）
        theta: pitch 角（绕 y 轴）
    """
    sa, ca = math.sin(alpha), math.cos(alpha)
    st, ct = math.sin(theta), math.cos(theta)
    return (
        (ct, st * sa, st * ca),
        (0.0, ca, -sa),
        (-st, ct * sa, ct * ca),
    )


def _rotation_derivatives(alpha: float, theta: float) -> Tuple[Mat3, Mat3]:
    """返回 (dR/dalpha, dR/dtheta)"""
    sa, ca = math.sin(alpha), math.cos(alpha)
    st, ct = math.sin(theta), math.cos(theta)
    dR_da = (
        (0.0, st * ca, -st * sa),
        (0.0, -sa, -ca),
        (0.0, ct * ca, -ct * sa),
    )
    dR_dt = (
        (-st, ct * sa, ct * ca),
        (0.0, 0.0, 0.0),
        (-ct, -st * sa, -st * ca),
    )
    return dR_da, dR_dt


def _matvec(M: Mat3, v: Vec3) -> Vec3:
    return (
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    )


def _dot(u: Vec3, v: Vec3) -> float:
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def _solve2(M: Mat2, b: Vec2, det_eps: float = 1e-12) -> Vec2:
    """解 2x2 线性方程 M x = b（Cramer 法则）"""
    (m11, m12), (m21, m22) = M
    det = m11 * m22 - m12 * m21
    if abs(det) < det_eps:
        raise SingularityError(f"矩阵接近奇异: det={det}")
    return (
        (m22 * b[0] - m12 * b[1]) / det,
        (-m21 * b[0] + m11 * b[1]) / det,
    )


def _transpose(M: Mat2) -> Mat2:
    return ((M[0][0], M[1][0]), (M[0][1], M[1][1]))


def _mul2(M: Mat2, v: Vec2) -> Vec2:
    return (M[0][0] * v[0] + M[0][1] * v[1], M[1][0] * v[0] + M[1][1] * v[1])


# ======================================================================
# 运动学求解器
# ======================================================================
class AnkleKinematics:
    """踝关节直线模组机构运动学：位置 / 速度 / 力的正、逆求解"""

    def __init__(
        self,
        geometry: AnkleGeometry,
        alpha_range: Optional[Vec2] = None,
        theta_range: Optional[Vec2] = None,
    ):
        """
        Args:
            geometry: 机构几何参数
            alpha_range: roll 角限位 (min, max)，用于剔除长度方程的伪解；None 表示不限制
            theta_range: pitch 角限位 (min, max)，同上
        """
        self.geom = geometry
        self.alpha_range = alpha_range
        self.theta_range = theta_range

        # 上一次正解的姿态，作为连续轨迹的迭代初值（热启动）
        self.alpha_last: Optional[float] = None
        self.theta_last: Optional[float] = None

        # 最近一次 _solve_pose 的 Newton 迭代次数（性能分析用）
        self.last_iterations = 0

    def reset(self):
        """清除热启动记录（轨迹不连续时调用）"""
        self.alpha_last = None
        self.theta_last = None

    # ------------------------------------------------------------------
    # 位置
    # ------------------------------------------------------------------
    def _link_vectors(self, alpha: float, theta: float) -> Tuple[Vec3, Vec3]:
        """两根推杆向量 e_i = a_i - R·b_i"""
        R = rotation_matrix(alpha, theta)
        g = self.geom
        Rb1 = _matvec(R, g.b1)
        Rb2 = _matvec(R, g.b2)
        e1 = (g.a1[0] - Rb1[0], g.a1[1] - Rb1[1], g.a1[2] - Rb1[2])
        e2 = (g.a2[0] - Rb2[0], g.a2[1] - Rb2[1], g.a2[2] - Rb2[2])
        return e1, e2

    def inverse_kinematics(self, alpha: float, theta: float) -> Vec2:
        """位置逆解：关节角 (alpha, theta) → 推杆长度 (l1, l2)，解析计算

        控制下发时用于把期望关节角换算成电机目标长度。
        """
        e1, e2 = self._link_vectors(alpha, theta)
        return math.sqrt(_dot(e1, e1)), math.sqrt(_dot(e2, e2))

    def forward_kinematics(
        self,
        l1: float,
        l2: float,
        alpha_init: Optional[float] = None,
        theta_init: Optional[float] = None,
        max_iter: int = 50,
        tolerance: float = 1e-9,
    ) -> AnkleState:
        """位置正解：推杆长度 (l1, l2) → 关节角 (alpha, theta)，Newton 迭代

        状态估计时用于把编码器长度换算成踝关节角度。

        Args:
            l1, l2: 推杆长度（米）
            alpha_init, theta_init: 迭代初值；None 时使用上次结果，没有则为 0
            max_iter: 最大迭代次数
            tolerance: 收敛容差（长度残差，米）

        Raises:
            ValueError: 输入无效、不收敛或解超出关节限位
        """
        if not (math.isfinite(l1) and math.isfinite(l2)) or l1 <= 0 or l2 <= 0:
            raise ValueError("l1 和 l2 必须为有限的正数")

        if alpha_init is None:
            alpha_init = self.alpha_last if self.alpha_last is not None else 0.0
        if theta_init is None:
            theta_init = self.theta_last if self.theta_last is not None else 0.0

        # 长度方程在限位外存在伪解（如 alpha≈2.6 rad）。若从热启动初值出发失败或
        # 落到限位外，退回零位初值重解；零位附近的解就是物理可行分支。
        try:
            alpha, theta = self._solve_pose(
                l1, l2, alpha_init, theta_init, max_iter, tolerance
            )
            warm_ok = self._in_range(alpha, theta)
        except ValueError:
            warm_ok = False

        if not warm_ok:
            alpha, theta = self._solve_pose(l1, l2, 0.0, 0.0, max_iter, tolerance)
            if not self._in_range(alpha, theta):
                raise ValueError(f"解超出关节限位: alpha={alpha}, theta={theta}")

        self.alpha_last = alpha
        self.theta_last = theta
        return AnkleState(alpha=alpha, theta=theta)

    def _in_range(self, alpha: float, theta: float, margin: float = 1e-6) -> bool:
        """解是否在关节限位内（未设置限位时恒为 True）"""
        for value, rng in ((alpha, self.alpha_range), (theta, self.theta_range)):
            if rng is not None and not (rng[0] - margin <= value <= rng[1] + margin):
                return False
        return True

    def _solve_pose(
        self,
        l1: float,
        l2: float,
        alpha_init: float,
        theta_init: float,
        max_iter: int,
        tolerance: float,
    ) -> Vec2:
        """对两条长度约束做带阻尼线搜索的二维 Newton 迭代"""
        alpha, theta = alpha_init, theta_init

        for k in range(max_iter):
            current = self.inverse_kinematics(alpha, theta)
            residual = (current[0] - l1, current[1] - l2)
            error = max(abs(residual[0]), abs(residual[1]))
            if error < tolerance:
                self.last_iterations = k
                return alpha, theta

            # J · delta = residual，更新方向为 -delta
            try:
                da, dt = _solve2(self.jacobian(alpha, theta), residual)
            except SingularityError as exc:
                raise SingularityError(
                    f"接近运动学奇异位形: alpha={alpha}, theta={theta} ({exc})"
                ) from exc

            # 阻尼线搜索，避免 Newton 步跨过局部可行分支
            step = 1.0
            while step >= 1.0 / 128.0:
                candidate = (alpha - step * da, theta - step * dt)
                lengths = self.inverse_kinematics(*candidate)
                if max(abs(lengths[0] - l1), abs(lengths[1] - l2)) < error:
                    alpha, theta = candidate
                    break
                step *= 0.5
            else:
                raise ValueError(
                    f"Newton 迭代无法降低残差: residual={error}, "
                    f"alpha={alpha}, theta={theta}"
                )

        final = self.inverse_kinematics(alpha, theta)
        final_error = max(abs(final[0] - l1), abs(final[1] - l2))
        self.last_iterations = max_iter
        if final_error < tolerance * 10:
            return alpha, theta
        raise ValueError(
            f"Newton 迭代未收敛: max_iter={max_iter}, final_residual={final_error}"
        )

    # ------------------------------------------------------------------
    # 速度
    # ------------------------------------------------------------------
    def jacobian(self, alpha: float, theta: float) -> Mat2:
        """解析雅可比 J = ∂(l1, l2)/∂(alpha, theta)

        dl_i/dq = -e_iᵀ (dR/dq · b_i) / l_i，其中 e_i = a_i - R·b_i

        Returns:
            ((dl1/dα, dl1/dθ), (dl2/dα, dl2/dθ))
        """
        e1, e2 = self._link_vectors(alpha, theta)
        dR_da, dR_dt = _rotation_derivatives(alpha, theta)
        g = self.geom

        rows = []
        for e, b in ((e1, g.b1), (e2, g.b2)):
            length = math.sqrt(_dot(e, e))
            if length < 1e-12:
                raise ValueError("推杆长度接近 0，雅可比无定义")
            rows.append((
                -_dot(e, _matvec(dR_da, b)) / length,
                -_dot(e, _matvec(dR_dt, b)) / length,
            ))
        return (rows[0], rows[1])

    def inverse_velocity(
        self, alpha: float, theta: float, alpha_dot: float, theta_dot: float
    ) -> Vec2:
        """速度逆解：关节角速度 (α̇, θ̇) → 推杆速度 (l̇1, l̇2)，l̇ = J q̇

        Args:
            alpha, theta: 当前关节角（rad）
            alpha_dot, theta_dot: 关节角速度（rad/s）

        Returns:
            (l1_dot, l2_dot): 推杆伸长速度（m/s）
        """
        return _mul2(self.jacobian(alpha, theta), (alpha_dot, theta_dot))

    def forward_velocity(
        self, alpha: float, theta: float, l1_dot: float, l2_dot: float
    ) -> Vec2:
        """速度正解：推杆速度 (l̇1, l̇2) → 关节角速度 (α̇, θ̇)，q̇ = J⁻¹ l̇

        alpha, theta 通常取 forward_kinematics 的结果。

        Raises:
            SingularityError: 雅可比接近奇异
        """
        return _solve2(self.jacobian(alpha, theta), (l1_dot, l2_dot))

    # ------------------------------------------------------------------
    # 力
    # ------------------------------------------------------------------
    def force_to_torque(
        self, alpha: float, theta: float, f1: float, f2: float
    ) -> Vec2:
        """力正映射：推杆力 (F1, F2) → 关节力矩 (τ_roll, τ_pitch)，τ = Jᵀ F

        Args:
            f1, f2: 推杆力（N），沿伸长方向为正

        Returns:
            (tau_roll, tau_pitch): 关节力矩（N·m）
        """
        return _mul2(_transpose(self.jacobian(alpha, theta)), (f1, f2))

    def torque_to_force(
        self, alpha: float, theta: float, tau_roll: float, tau_pitch: float
    ) -> Vec2:
        """力逆映射：期望关节力矩 (τ_roll, τ_pitch) → 推杆力 (F1, F2)，F = J⁻ᵀ τ

        Raises:
            SingularityError: 雅可比接近奇异
        """
        return _solve2(_transpose(self.jacobian(alpha, theta)), (tau_roll, tau_pitch))

    # ------------------------------------------------------------------
    # 奇异性
    # ------------------------------------------------------------------
    def check_singularity(
        self, alpha: float, theta: float, threshold: float = 0.01
    ) -> Tuple[bool, float]:
        """检查是否接近运动学奇异位形

        Args:
            threshold: 最小奇异值阈值（m/rad）

        Returns:
            (is_singular, min_singular_value)
        """
        (J11, J12), (J21, J22) = self.jacobian(alpha, theta)

        # 奇异值是 JᵀJ 特征值的平方根
        gram11 = J11 * J11 + J21 * J21
        gram22 = J12 * J12 + J22 * J22
        gram12 = J11 * J12 + J21 * J22
        trace = gram11 + gram22
        discriminant = max(0.0, (gram11 - gram22) ** 2 + 4.0 * gram12 ** 2)
        lambda_min = 0.5 * (trace - math.sqrt(discriminant))
        min_singular_value = math.sqrt(max(0.0, lambda_min))

        return min_singular_value < threshold, min_singular_value
