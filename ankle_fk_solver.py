"""
踝关节直线模组机构正向运动学求解器
基于文档: doc/linear_module_lower_leg_fk_with_h2.md

旋转约定与 model/xml/calf_actuator.xml 的关节层级一致：
    ankle_pitch_joint（父，绕 y）→ ankle_roll_joint（子，绕 x）
    R(alpha, theta) = Ry(theta) · Rx(alpha)

长度约束（a、b 均为带符号坐标，见 ankle_geometry.py）：
    l_i = |a_i - R · b_i|,  i = 1, 2
"""
import math
from typing import Optional, Tuple
from ankle_geometry import AnkleGeometry, AnkleState, Vec3

Mat3 = Tuple[Vec3, Vec3, Vec3]
Mat2 = Tuple[Tuple[float, float], Tuple[float, float]]


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


class AnkleFKSolver:
    """踝关节正向运动学求解器

    给定两根直线模组长度 l1, l2，求解踝关节角度 alpha (roll) 和 theta (pitch)
    """

    def __init__(
        self,
        geometry: AnkleGeometry,
        alpha_range: Optional[Tuple[float, float]] = None,
        theta_range: Optional[Tuple[float, float]] = None,
    ):
        """初始化求解器

        Args:
            geometry: 机构几何参数
            alpha_range: roll 角限位 (min, max)，用于剔除长度方程的伪解；None 表示不限制
            theta_range: pitch 角限位 (min, max)，同上
        """
        self.geom = geometry
        self.alpha_range = alpha_range
        self.theta_range = theta_range

        # 上一次求解的姿态，用于连续轨迹的迭代初值
        self.alpha_last: Optional[float] = None
        self.theta_last: Optional[float] = None

    # ------------------------------------------------------------------
    # 正向：角度 -> 长度
    # ------------------------------------------------------------------
    def _link_vectors(self, alpha: float, theta: float) -> Tuple[Vec3, Vec3]:
        """返回两根推杆向量 e_i = a_i - R·b_i"""
        R = rotation_matrix(alpha, theta)
        g = self.geom
        Rb1 = _matvec(R, g.b1)
        Rb2 = _matvec(R, g.b2)
        e1 = (g.a1[0] - Rb1[0], g.a1[1] - Rb1[1], g.a1[2] - Rb1[2])
        e2 = (g.a2[0] - Rb2[0], g.a2[1] - Rb2[1], g.a2[2] - Rb2[2])
        return e1, e2

    def _forward_kinematics(self, alpha: float, theta: float) -> Tuple[float, float]:
        """从关节角度计算直线模组长度

        Args:
            alpha: roll 角
            theta: pitch 角

        Returns:
            (l1, l2): 两根直线模组长度
        """
        e1, e2 = self._link_vectors(alpha, theta)
        return math.sqrt(_dot(e1, e1)), math.sqrt(_dot(e2, e2))

    def compute_jacobian(self, alpha: float, theta: float) -> Mat2:
        """解析雅可比 J_l = ∂(l1,l2)/∂(alpha,theta)

        dl_i/dq = -e_i^T (dR/dq · b_i) / l_i，其中 e_i = a_i - R·b_i

        Returns:
            ((J11, J12), (J21, J22)): 2x2 雅可比矩阵
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

    # ------------------------------------------------------------------
    # 求解：长度 -> 角度
    # ------------------------------------------------------------------
    def solve_fk(
        self,
        l1: float,
        l2: float,
        theta_init: Optional[float] = None,
        max_iter: int = 50,
        tolerance: float = 1e-9,
        alpha_init: Optional[float] = None
    ) -> AnkleState:
        """求解正向运动学

        Args:
            l1: 第一根直线模组长度（+y 侧，actuator_b1）
            l2: 第二根直线模组长度（-y 侧，actuator_b2）
            theta_init: theta 初值（如不提供，使用上次结果或0）
            max_iter: 最大迭代次数
            tolerance: 收敛容差（长度残差，米）
            alpha_init: alpha 初值（如不提供，使用上次结果或0）

        Returns:
            AnkleState: 包含 alpha 和 theta 的关节状态

        Raises:
            ValueError: 如果求解失败
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
                raise ValueError(
                    f"解超出关节限位: alpha={alpha}, theta={theta}"
                )

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
    ) -> Tuple[float, float]:
        """对两条长度约束做带阻尼线搜索的二维 Newton 迭代"""
        alpha, theta = alpha_init, theta_init

        for _ in range(max_iter):
            current = self._forward_kinematics(alpha, theta)
            residual = (current[0] - l1, current[1] - l2)
            error = max(abs(residual[0]), abs(residual[1]))
            if error < tolerance:
                return alpha, theta

            (j11, j12), (j21, j22) = self.compute_jacobian(alpha, theta)
            det = j11 * j22 - j12 * j21
            if abs(det) < 1e-12:
                raise ValueError(
                    f"接近运动学奇异位形: det(J)={det}, "
                    f"alpha={alpha}, theta={theta}"
                )

            # J * delta = residual，更新方向为 -delta
            da = (j22 * residual[0] - j12 * residual[1]) / det
            dt = (-j21 * residual[0] + j11 * residual[1]) / det

            # 阻尼线搜索，避免 Newton 步跨过局部可行分支
            step = 1.0
            accepted = False
            while step >= 1.0 / 128.0:
                candidate = (alpha - step * da, theta - step * dt)
                lengths = self._forward_kinematics(*candidate)
                candidate_error = max(abs(lengths[0] - l1), abs(lengths[1] - l2))
                if candidate_error < error:
                    alpha, theta = candidate
                    accepted = True
                    break
                step *= 0.5

            if not accepted:
                raise ValueError(
                    f"Newton 迭代无法降低残差: residual={error}, "
                    f"alpha={alpha}, theta={theta}"
                )

        final = self._forward_kinematics(alpha, theta)
        final_error = max(abs(final[0] - l1), abs(final[1] - l2))
        if final_error < tolerance * 10:
            return alpha, theta
        raise ValueError(
            f"Newton 迭代未收敛: max_iter={max_iter}, "
            f"final_residual={final_error}"
        )

    def check_singularity(
        self,
        alpha: float,
        theta: float,
        threshold: float = 0.01
    ) -> Tuple[bool, float]:
        """检查是否接近运动学奇异位形

        Args:
            alpha: roll 角
            theta: pitch 角
            threshold: 奇异性阈值

        Returns:
            (is_singular, min_singular_value): 是否奇异及最小奇异值
        """
        (J11, J12), (J21, J22) = self.compute_jacobian(alpha, theta)

        # 奇异值是 J^T J 的特征值平方根
        gram11 = J11 * J11 + J21 * J21
        gram22 = J12 * J12 + J22 * J22
        gram12 = J11 * J12 + J21 * J22
        trace = gram11 + gram22
        discriminant = max(0.0, (gram11 - gram22) ** 2 + 4.0 * gram12 ** 2)
        lambda_min = 0.5 * (trace - math.sqrt(discriminant))
        min_singular_value = math.sqrt(max(0.0, lambda_min))

        return min_singular_value < threshold, min_singular_value
