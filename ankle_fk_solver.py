"""
踝关节直线模组机构正向运动学求解器
基于文档: linear_module_lower_leg_fk_with_h2.md
"""
import math
from typing import Optional, Tuple
from ankle_geometry import AnkleGeometry, AnkleState


class AnkleFKSolver:
    """踝关节正向运动学求解器

    给定两根直线模组长度 l1, l2，求解踝关节角度 alpha (roll) 和 theta (pitch)
    """

    def __init__(self, geometry: AnkleGeometry):
        """初始化求解器

        Args:
            geometry: 机构几何参数
        """
        self.geom = geometry

        # 预计算常量 C
        self.C = (
            geometry.m ** 2 +
            geometry.n ** 2 +
            geometry.h ** 2 +
            geometry.c ** 2 +
            geometry.d ** 2 +
            geometry.h2 ** 2
        )

        # 上一次求解的姿态，用于连续轨迹的迭代初值
        self.alpha_last: Optional[float] = None
        self.theta_last: Optional[float] = None

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
            l1: 第一根直线模组长度
            l2: 第二根直线模组长度
            theta_init: theta 初值（如不提供，使用上次结果或0）
            max_iter: 最大迭代次数
            tolerance: 收敛容差
            alpha_init: alpha 初值（如不提供，使用上次结果或0）

        Returns:
            AnkleState: 包含 alpha 和 theta 的关节状态

        Raises:
            ValueError: 如果求解失败
        """
        if not (math.isfinite(l1) and math.isfinite(l2)) or l1 <= 0 or l2 <= 0:
            raise ValueError("l1 和 l2 必须为有限的正数")

        # 直接求解两根长度方程，避免文档差分公式中遗漏的耦合项。
        if alpha_init is None:
            alpha_init = self.alpha_last if self.alpha_last is not None else 0.0
        if theta_init is None:
            theta_init = self.theta_last if self.theta_last is not None else 0.0

        alpha, theta = self._solve_pose(
            l1, l2, alpha_init, theta_init, max_iter, tolerance
        )

        self.alpha_last = alpha
        self.theta_last = theta

        return AnkleState(alpha=alpha, theta=theta)

    def _solve_pose(
        self,
        l1: float,
        l2: float,
        alpha_init: float,
        theta_init: float,
        max_iter: int,
        tolerance: float,
    ) -> Tuple[float, float]:
        """用当前几何模型的两条长度约束直接求解两个姿态角。"""
        alpha, theta = alpha_init, theta_init
        target = (l1, l2)

        for _ in range(max_iter):
            current = self._forward_kinematics(alpha, theta)
            residual = (current[0] - target[0], current[1] - target[1])
            error = max(abs(residual[0]), abs(residual[1]))
            if error < tolerance:
                return alpha, theta

            jacobian = self.compute_jacobian(alpha, theta)
            j11, j12 = jacobian[0]
            j21, j22 = jacobian[1]
            det = j11 * j22 - j12 * j21
            if abs(det) < 1e-12:
                raise ValueError(
                    f"接近运动学奇异位形: det(J)={det}, "
                    f"alpha={alpha}, theta={theta}"
                )

            # J * delta = residual，更新方向为 -delta。
            da = (j22 * residual[0] - j12 * residual[1]) / det
            dt = (-j21 * residual[0] + j11 * residual[1]) / det

            # 阻尼线搜索，避免 Newton 步跨过局部可行分支。
            step = 1.0
            accepted = False
            while step >= 1.0 / 128.0:
                candidate = (alpha - step * da, theta - step * dt)
                lengths = self._forward_kinematics(*candidate)
                candidate_error = max(
                    abs(lengths[0] - l1), abs(lengths[1] - l2)
                )
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

    def _solve_theta(
        self,
        S: float,
        D: float,
        theta_init: float,
        max_iter: int,
        tolerance: float
    ) -> float:
        """使用 Newton-Raphson 方法求解 theta

        Args:
            S: (l1^2 + l2^2) / 2
            D: (l1^2 - l2^2) / 4
            theta_init: 初值
            max_iter: 最大迭代次数
            tolerance: 收敛容差

        Returns:
            theta: pitch 角

        Raises:
            ValueError: 如果求解失败
        """
        theta = theta_init

        for i in range(max_iter):
            f, df = self._fk_equation_with_derivative(theta, S, D)

            if abs(f) < tolerance:
                return theta

            if abs(df) < 1e-12:
                raise ValueError(
                    f"数值奇异：导数过小 (iter={i}, df={df})"
                )

            # Newton 迭代
            theta -= f / df

        # 未收敛，检查最终残差
        f, _ = self._fk_equation_with_derivative(theta, S, D)
        if abs(f) < tolerance * 10:
            return theta

        raise ValueError(
            f"Newton 迭代未收敛: max_iter={max_iter}, "
            f"final_residual={abs(f)}"
        )

    def _fk_equation_with_derivative(
        self,
        theta: float,
        S: float,
        D: float
    ) -> Tuple[float, float]:
        """计算 FK 方程 f(theta) 及其导数 f'(theta)

        方程: (N/(2A))^2 + (D/B)^2 - 1 = 0

        Args:
            theta: pitch 角
            S: (l1^2 + l2^2) / 2
            D: (l1^2 - l2^2) / 4

        Returns:
            (f, df): 方程值和导数值
        """
        st = math.sin(theta)
        ct = math.cos(theta)

        p = self.geom

        # 中间变量 A, B, N
        A = p.d * p.n + p.h * p.c * st - p.h * p.h2 * ct
        B = p.n * p.h2 * ct - p.n * p.c * st - p.h * p.d
        N = self.C - S - 2.0 * p.c * p.m * ct - 2.0 * p.m * p.h2 * st

        # 检查奇异性
        if abs(A) < 1e-12 or abs(B) < 1e-12:
            raise ValueError(
                f"接近运动学奇异位形: A={A}, B={B}, theta={theta}"
            )

        # 计算 f(theta)
        term1 = N / (2.0 * A)
        term2 = D / B
        f = term1 ** 2 + term2 ** 2 - 1.0

        # 计算导数 dA/dtheta, dB/dtheta, dN/dtheta
        dA_dtheta = p.h * p.c * ct + p.h * p.h2 * st
        dB_dtheta = -p.n * p.h2 * st - p.n * p.c * ct
        dN_dtheta = 2.0 * p.c * p.m * st - 2.0 * p.m * p.h2 * ct

        # 计算 df/dtheta 使用链式法则
        # d/dtheta[(N/(2A))^2] = 2*(N/(2A)) * d/dtheta[N/(2A)]
        # d/dtheta[N/(2A)] = (N'*(2A) - N*2A') / (2A)^2
        dterm1_dtheta = (dN_dtheta * 2 * A - N * 2 * dA_dtheta) / (4 * A ** 2)
        df_term1 = 2 * term1 * dterm1_dtheta

        # d/dtheta[(D/B)^2] = 2*(D/B) * d/dtheta[D/B]
        # d/dtheta[D/B] = -D*B' / B^2
        dterm2_dtheta = -D * dB_dtheta / (B ** 2)
        df_term2 = 2 * term2 * dterm2_dtheta

        df = df_term1 + df_term2

        return f, df

    def _solve_alpha(self, S: float, D: float, theta: float) -> float:
        """根据求解的 theta 计算 alpha

        Args:
            S: (l1^2 + l2^2) / 2
            D: (l1^2 - l2^2) / 4
            theta: 已求解的 pitch 角

        Returns:
            alpha: roll 角
        """
        st = math.sin(theta)
        ct = math.cos(theta)

        p = self.geom

        # 计算中间变量
        A = p.d * p.n + p.h * p.c * st - p.h * p.h2 * ct
        B = p.n * p.h2 * ct - p.n * p.c * st - p.h * p.d
        N = self.C - S - 2.0 * p.c * p.m * ct - 2.0 * p.m * p.h2 * st

        # 计算 sin(alpha) 和 cos(alpha)
        sin_alpha = D / B
        cos_alpha = N / (2.0 * A)

        # 使用 atan2 获得正确的象限
        alpha = math.atan2(sin_alpha, cos_alpha)

        return alpha

    def compute_jacobian(
        self,
        alpha: float,
        theta: float
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """计算雅可比矩阵 J_l = ∂(l1,l2)/∂(alpha,theta)

        用于奇异性分析和力控制

        Args:
            alpha: roll 角
            theta: pitch 角

        Returns:
            ((J11, J12), (J21, J22)): 2x2 雅可比矩阵
        """
        # 数值微分计算雅可比
        eps = 1e-7

        l1_0, l2_0 = self._forward_kinematics(alpha, theta)
        l1_da, l2_da = self._forward_kinematics(alpha + eps, theta)
        l1_dt, l2_dt = self._forward_kinematics(alpha, theta + eps)

        J11 = (l1_da - l1_0) / eps  # ∂l1/∂alpha
        J12 = (l1_dt - l1_0) / eps  # ∂l1/∂theta
        J21 = (l2_da - l2_0) / eps  # ∂l2/∂alpha
        J22 = (l2_dt - l2_0) / eps  # ∂l2/∂theta

        return ((J11, J12), (J21, J22))

    def _forward_kinematics(self, alpha: float, theta: float) -> Tuple[float, float]:
        """正向运动学：从关节角度计算直线模组长度

        Args:
            alpha: roll 角
            theta: pitch 角

        Returns:
            (l1, l2): 两根直线模组长度
        """
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        st = math.sin(theta)
        ct = math.cos(theta)

        p = self.geom

        # 旋转矩阵
        R = [
            [ct, 0, st],
            [sa * st, ca, -sa * ct],
            [-ca * st, sa, ca * ct]
        ]

        # b1 和 b2 在局部坐标系
        b1 = [-p.c, -p.d, -p.h2]
        b2 = [-p.c, p.d, -p.h2]

        # 旋转后的 b1 和 b2
        Rb1 = [
            R[0][0] * b1[0] + R[0][1] * b1[1] + R[0][2] * b1[2],
            R[1][0] * b1[0] + R[1][1] * b1[1] + R[1][2] * b1[2],
            R[2][0] * b1[0] + R[2][1] * b1[1] + R[2][2] * b1[2]
        ]

        Rb2 = [
            R[0][0] * b2[0] + R[0][1] * b2[1] + R[0][2] * b2[2],
            R[1][0] * b2[0] + R[1][1] * b2[1] + R[1][2] * b2[2],
            R[2][0] * b2[0] + R[2][1] * b2[1] + R[2][2] * b2[2]
        ]

        # a1 和 a2
        a1 = [-p.m, -p.n, p.h]
        a2 = [-p.m, p.n, p.h]

        # 计算长度
        diff1 = [a1[i] - Rb1[i] for i in range(3)]
        diff2 = [a2[i] - Rb2[i] for i in range(3)]

        l1 = math.sqrt(sum(d ** 2 for d in diff1))
        l2 = math.sqrt(sum(d ** 2 for d in diff2))

        return l1, l2

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
        J = self.compute_jacobian(alpha, theta)

        # 计算 2x2 矩阵的奇异值
        J11, J12 = J[0]
        J21, J22 = J[1]

        # 奇异值是 J.T @ J 的特征值平方根，而不是 J 的特征值。
        gram11 = J11 * J11 + J21 * J21
        gram22 = J12 * J12 + J22 * J22
        gram12 = J11 * J12 + J21 * J22
        trace = gram11 + gram22
        discriminant = max(0.0, (gram11 - gram22) ** 2 + 4.0 * gram12 ** 2)
        lambda_min = 0.5 * (trace - math.sqrt(discriminant))
        min_singular_value = math.sqrt(max(0.0, lambda_min))

        is_singular = min_singular_value < threshold

        return is_singular, min_singular_value
