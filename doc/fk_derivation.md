# 人形机器人直线模组小腿机构 FK 推导

与 `model/xml/calf_actuator.xml` 保持一致：坐标系、安装点编号、关节层级全部对应。

------------------------------------------------------------------------

# 1. 坐标约定

原点取**踝关节旋转中心**（XML 中 `ankle_pitch_Link` 在 `calf_link` 下的位置
`(0.049, 0, -0.378)`），坐标轴方向与 XML 相同，$z$ 轴向上。

所有参数都是**带符号的真实坐标**，公式中不再额外加负号：

$$
a_1=(a_x,\ a_y,\ a_z),\qquad a_2=(a_x,\ -a_y,\ a_z)
$$

$$
b_1=(b_x,\ b_y,\ b_z),\qquad b_2=(b_x,\ -b_y,\ b_z)
$$

-   $a_i$：小腿上的固定安装点（踝中心系，不随脚踝转动）
-   $b_i$：脚踝上的安装点（脚踝局部系，随脚踝转动）
-   下标 1 位于 $+y$ 侧（XML `actuator_b1`），下标 2 位于 $-y$ 侧（`actuator_b2`），要求 $a_y>0,\ b_y>0$
-   $l_i$：连接 $a_i$ 与 $b_i$ 的直线模组长度

`calf_actuator.xml` 对应数值（米）：

| 参数 | 值 | 来源 |
|---|---|---|
| $a_x$ | −0.061 | site a1 的 x：−0.012 − 0.049 |
| $a_y$ | 0.0405 | site a1 的 y |
| $a_z$ | 0.291 | site a1 的 z：−0.087 − (−0.378) |
| $b_x$ | −0.075 | site b1 的 x |
| $b_y$ | 0.0405 | site b1 的 y |
| $b_z$ | 0.001 | site b1 的 z（比旋转中心高 1 mm） |

零位推杆长度 $|a_1-b_1|=\sqrt{0.014^2+0.290^2}\approx0.290338$ m，行程 ±0.04 m。

------------------------------------------------------------------------

# 2. 旋转约定

XML 中 `ankle_pitch_joint`（绕 $y$）是父关节，`ankle_roll_joint`（绕 $x$）挂在它下面，因此：

$$
R(\alpha,\theta)=R_y(\theta)\,R_x(\alpha)
$$

-   $\alpha$：roll 角（`ankle_roll_joint`）
-   $\theta$：pitch 角（`ankle_pitch_joint`）

记 $s_\alpha=\sin\alpha,\ c_\alpha=\cos\alpha,\ s_\theta=\sin\theta,\ c_\theta=\cos\theta$：

$$
R=
\begin{bmatrix}
c_\theta & s_\theta s_\alpha & s_\theta c_\alpha\\
0 & c_\alpha & -s_\alpha\\
-s_\theta & c_\theta s_\alpha & c_\theta c_\alpha
\end{bmatrix}
$$

> 旋转顺序不可交换。若写成 $R_x(\alpha)R_y(\theta)$，单轴转动时结果相同，
> 但两轴同时转动时与 MuJoCo 不一致（限位角附近误差可达 20° 量级）。

------------------------------------------------------------------------

# 3. 长度方程

## 3.1 向量形式

$$
l_i=\|a_i-R\,b_i\|,\qquad i=1,2
$$

由于 $R$ 为正交矩阵，$\|Rb_i\|=\|b_i\|$，平方展开为：

$$
\boxed{
l_i^2=\|a_i\|^2+\|b_i\|^2-2\,a_i^{T}R\,b_i
}
$$

两侧镜像对称，$\|a_1\|=\|a_2\|$、$\|b_1\|=\|b_2\|$，记常量：

$$
K=a_x^2+a_y^2+a_z^2+b_x^2+b_y^2+b_z^2
$$

## 3.2 标量展开

把 $a_i^TRb_i$ 按 $a_y, b_y$ 的符号拆开，定义三个只与 $\theta$ 有关的量：

$$
E(\theta)=b_x\left(a_x c_\theta-a_z s_\theta\right)
$$

$$
A(\theta)=a_y b_y+b_z\left(a_x s_\theta+a_z c_\theta\right)
$$

$$
B(\theta)=b_y\left(a_x s_\theta+a_z c_\theta\right)-a_y b_z
$$

则：

$$
\boxed{
\begin{aligned}
l_1^2&=K-2\left[E(\theta)+A(\theta)\cos\alpha\right]-2B(\theta)\sin\alpha\\
l_2^2&=K-2\left[E(\theta)+A(\theta)\cos\alpha\right]+2B(\theta)\sin\alpha
\end{aligned}}
$$

两式只在 $\sin\alpha$ 项上差一个符号：roll 使一侧推杆缩短、另一侧伸长。

------------------------------------------------------------------------

# 4. 化为关于 $\theta$ 的一维方程

定义：

$$
S=\frac{l_1^2+l_2^2}{2},\qquad D=\frac{l_2^2-l_1^2}{4}
$$

相加、相减得到（**精确成立，没有交叉项**）：

$$
\boxed{
\cos\alpha=\frac{K-S-2E(\theta)}{2A(\theta)}
},\qquad
\boxed{
\sin\alpha=\frac{D}{B(\theta)}
}
$$

利用 $\sin^2\alpha+\cos^2\alpha=1$：

$$
\boxed{
f(\theta)=
\left(\frac{K-S-2E(\theta)}{2A(\theta)}\right)^2
+\left(\frac{D}{B(\theta)}\right)^2-1=0
}
$$

解出 $\theta$ 后：

$$
\boxed{
\alpha=\operatorname{atan2}\!\left(\frac{D}{B(\theta)},\ \frac{K-S-2E(\theta)}{2A(\theta)}\right)
}
$$

> 旧版推导采用 $R_x R_y$ 顺序，相减后残留 $d\cos\alpha$ 交叉项，
> 所以 $\sin\alpha=D/B$ 不严格成立。按 XML 的 $R_yR_x$ 顺序展开后交叉项消失，上式是精确的。

------------------------------------------------------------------------

# 5. 数值求解（代码实现）

`ankle/kinematics.py` 没有走第 4 节的一维方程，而是直接对两条长度约束做二维 Newton 迭代：

$$
\mathbf{r}(\alpha,\theta)=
\begin{bmatrix}
l_1(\alpha,\theta)-l_1^{*}\\
l_2(\alpha,\theta)-l_2^{*}
\end{bmatrix}=0,
\qquad
\begin{bmatrix}\alpha\\\theta\end{bmatrix}
\leftarrow
\begin{bmatrix}\alpha\\\theta\end{bmatrix}-\lambda\,J_l^{-1}\mathbf{r}
$$

-   $\lambda$ 由阻尼线搜索确定（从 1 开始折半，直到残差下降）
-   初值取上一周期的解（实时控制中 1～3 次迭代即可收敛）
-   不需要除以 $A(\theta)$、$B(\theta)$，所以不存在第 4 节里的除零问题

**伪解处理**：长度方程在关节限位以外还有其他解（例如 $\alpha\approx2.6$ rad）。
热启动初值离真实姿态太远时，Newton 可能收敛到伪解。构造求解器时传入关节限位：

```python
solver = AnkleKinematics(
    get_geometry("calf_actuator"),
    alpha_range=CALF_ACTUATOR_ROLL_RANGE,
    theta_range=CALF_ACTUATOR_PITCH_RANGE,
)
```

热启动的结果超出限位（或迭代失败）时，会自动改用零位初值重解。

------------------------------------------------------------------------

# 6. 雅可比矩阵

记推杆向量 $e_i=a_i-Rb_i$，$l_i=\|e_i\|$，对任一关节角 $q\in\{\alpha,\theta\}$：

$$
\boxed{
\frac{\partial l_i}{\partial q}=-\frac{e_i^{T}\left(\dfrac{\partial R}{\partial q}\,b_i\right)}{l_i}
}
$$

其中：

$$
\frac{\partial R}{\partial\alpha}=
\begin{bmatrix}
0 & s_\theta c_\alpha & -s_\theta s_\alpha\\
0 & -s_\alpha & -c_\alpha\\
0 & c_\theta c_\alpha & -c_\theta s_\alpha
\end{bmatrix},
\qquad
\frac{\partial R}{\partial\theta}=
\begin{bmatrix}
-s_\theta & c_\theta s_\alpha & c_\theta c_\alpha\\
0 & 0 & 0\\
-c_\theta & -s_\theta s_\alpha & -s_\theta c_\alpha
\end{bmatrix}
$$

$$
J_l=\frac{\partial(l_1,l_2)}{\partial(\alpha,\theta)}=
\begin{bmatrix}
\partial l_1/\partial\alpha & \partial l_1/\partial\theta\\
\partial l_2/\partial\alpha & \partial l_2/\partial\theta
\end{bmatrix}
$$

代码中用的是这个解析式（和中心差分对比，误差约 1e-11）。

**奇异性**：监测 $\sigma_{\min}(J_l)$（`check_singularity`），它趋近 0 时，
推杆长度无法唯一确定关节角，力映射也会失效。

------------------------------------------------------------------------

# 7. 力映射

$$
\dot{\mathbf{l}}=J_l\,\dot{\mathbf{q}},\qquad
\mathbf{l}=\begin{bmatrix}l_1\\l_2\end{bmatrix},\quad
\mathbf{q}=\begin{bmatrix}\alpha\\\theta\end{bmatrix}
$$

由虚功原理 $F^T\delta\mathbf{l}=\tau^T\delta\mathbf{q}$：

$$
\boxed{
\begin{bmatrix}\tau_{roll}\\\tau_{pitch}\end{bmatrix}
=J_l^{T}
\begin{bmatrix}F_1\\F_2\end{bmatrix}
}
$$

$F_i$ 以推杆伸长方向为正。零位时两推杆推力相同，$\tau_{roll}=0$。

------------------------------------------------------------------------

# 8. C++ 实现框架

```cpp
struct AnkleGeometry
{
    // a1 = (ax,  ay, az), a2 = (ax, -ay, az)  踝中心系
    // b1 = (bx,  by, bz), b2 = (bx, -by, bz)  脚踝局部系
    double ax, ay, az;
    double bx, by, bz;
};

struct AnkleState { double alpha, theta; };

// R = Ry(theta) * Rx(alpha)
inline void lengths(double alpha, double theta, const AnkleGeometry& p,
                    double& l1, double& l2)
{
    const double sa = std::sin(alpha), ca = std::cos(alpha);
    const double st = std::sin(theta), ct = std::cos(theta);

    auto len = [&](double ay, double by) {
        // R * b
        const double rx =  ct * p.bx + st * sa * by + st * ca * p.bz;
        const double ry =              ca * by      - sa * p.bz;
        const double rz = -st * p.bx + ct * sa * by + ct * ca * p.bz;
        const double ex = p.ax - rx, ey = ay - ry, ez = p.az - rz;
        return std::sqrt(ex * ex + ey * ey + ez * ez);
    };

    l1 = len( p.ay,  p.by);
    l2 = len(-p.ay, -p.by);
}
```

求解流程和 Python 版一致：用上一周期的 $(\alpha,\theta)$ 作初值 → 二维 Newton 加线搜索 → 检查是否在限位内，
不在就用零位初值重解。

------------------------------------------------------------------------

# 9. 与旧参数的对应

旧版参数 $(m,n,h,c,d,h_2)$ 都取正值，符号写在公式里。换算关系：

| 新参数 | $a_x$ | $a_y$ | $a_z$ | $b_x$ | $b_y$ | $b_z$ |
|---|---|---|---|---|---|---|
| 旧参数 | $-m$ | $n$ | $h$ | $-c$ | $d$ | $-h_2$ |

另外，旧版的旋转顺序是 $R_x(\alpha)R_y(\theta)$，已改为与 XML 一致的 $R_y(\theta)R_x(\alpha)$。
