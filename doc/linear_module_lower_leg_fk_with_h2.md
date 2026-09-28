# 人形机器人直线模组小腿机构 FK 推导（含 h₂ 偏置）

## 1. 几何模型

小腿固定端连接点：

\[ a_1=(-m,-n,h) \]

\[ a_2=(-m,n,h) \]

脚踝局部坐标系下的连接点：

\[ b_1=(-c,-d,-h_2) \]

\[ b_2=(-c,d,-h_2) \]

两根直线模组长度：

\[ l_1=\|a_1-Rb_1\| \]

\[ l_2=\|a_2-Rb_2\| \]

其中：

-   $\alpha$：脚踝 roll 角
-   $\theta$：脚踝 pitch 角
-   $h_2$：脚踝连接点相对于脚踝旋转中心的局部 z 方向偏置
-   $l_1$：连接 $a_1$ 与 $b_1$ 的直线模组长度
-   $l_2$：连接 $a_2$ 与 $b_2$ 的直线模组长度

------------------------------------------------------------------------

# 2. 旋转约定

采用：

$$
R(\alpha,\theta)=R_x(\alpha)R_y(\theta)
$$

其中：

$$
R_x(\alpha)=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix}
$$

$$
R_y(\theta)=
\begin{bmatrix}
\cos\theta&0&\sin\theta\\
0&1&0\\
-\sin\theta&0&\cos\theta
\end{bmatrix}
$$

因此：

$$
R=
\begin{bmatrix}
\cos\theta&0&\sin\theta\\
\sin\alpha\sin\theta&\cos\alpha&-\sin\alpha\cos\theta\\
-\cos\alpha\sin\theta&\sin\alpha&\cos\alpha\cos\theta
\end{bmatrix}
$$

为了简化公式，定义：

\[
s\_`\alpha`{=tex}=`\sin`{=tex}`\alpha`{=tex},`\qquad `{=tex}c\_`\alpha`{=tex}=`\cos`{=tex}`\alpha`{=tex}
\]

\[
s\_`\theta`{=tex}=`\sin`{=tex}`\theta`{=tex},`\qquad `{=tex}c\_`\theta`{=tex}=`\cos`{=tex}`\theta`{=tex}
\]

------------------------------------------------------------------------

# 3. 旋转后的连接点

## 3.1 $b_1$

$$
b_1=
\begin{bmatrix}
-c\\
-d\\
-h_2
\end{bmatrix}
$$

旋转后：

$$
Rb_1=
\begin{bmatrix}
-c c_\theta-h_2s_\theta\\
-cs_\alpha s_\theta-dc_\alpha+h_2s_\alpha c_\theta\\
cc_\alpha s_\theta+ds_\alpha-h_2c_\alpha c_\theta
\end{bmatrix}
$$

------------------------------------------------------------------------

## 3.2 $b_2$

$$
b_2=
\begin{bmatrix}
-c\\
d\\
-h_2
\end{bmatrix}
$$

旋转后：

$$
Rb_2=
\begin{bmatrix}
-c c_\theta-h_2s_\theta\\
-cs_\alpha s_\theta+dc_\alpha+h_2s_\alpha c_\theta\\
cc_\alpha s_\theta+ds_\alpha-h_2c_\alpha c_\theta
\end{bmatrix}
$$

------------------------------------------------------------------------

# 4. 两根直线模组长度方程

## 4.1 第一根直线模组

定义：

\[ l_1=\|a_1-Rb_1\| \]

展开后：

$$
\boxed{
\begin{aligned}
l_1^2={}&
m^2+n^2+h^2+c^2+d^2+h_2^2\\
&-2cm\cos\theta\\
&-2mh_2\sin\theta\\
&-2cn\sin\alpha\sin\theta\\
&-2nd\cos\alpha\\
&+2nh_2\sin\alpha\cos\theta\\
&-2hc\cos\alpha\sin\theta\\
&-2hd\sin\alpha\\
&+2hh_2\cos\alpha\cos\theta
\end{aligned}}
$$

------------------------------------------------------------------------

## 4.2 第二根直线模组

定义：

\[ l_2=\|a_2-Rb_2\| \]

展开后：

$$
\boxed{
\begin{aligned}
l_2^2={}&
m^2+n^2+h^2+c^2+d^2+h_2^2\\
&-2cm\cos\theta\\
&-2mh_2\sin\theta\\
&+2cn\sin\alpha\sin\theta\\
&-2nd\cos\alpha\\
&-2nh_2\sin\alpha\cos\theta\\
&-2hc\cos\alpha\sin\theta\\
&-2hd\sin\alpha\\
&+2hh_2\cos\alpha\cos\theta
\end{aligned}}
$$

------------------------------------------------------------------------

# 5. 两式相加

定义：

\[ C=m^2+n^2+h^2+c^2+d^2+h_2^2 \]

以及：

$$
S=\frac{l_1^2+l_2^2}{2}
$$

将两式相加：

$$
\begin{aligned}
S={}&
C
-2cm\cos\theta
-2mh_2\sin\theta\\
&-2nd\cos\alpha
-2hc\cos\alpha\sin\theta\\
&+2hh_2\cos\alpha\cos\theta
\end{aligned}
$$

整理：

$$
\boxed{
S=
C
-2cm\cos\theta
-2mh_2\sin\theta
-2
\left(
dn+hc\sin\theta-hh_2\cos\theta
\right)
\cos\alpha
}
$$

因此：

$$
\boxed{
\cos\alpha=
\frac{
C-S-2cm\cos\theta-2mh_2\sin\theta
}{
2(dn+hc\sin\theta-hh_2\cos\theta)
}
}
$$

------------------------------------------------------------------------

# 6. 两式相减

定义：

$$
D=\frac{l_1^2-l_2^2}{4}
$$

将两式相减：

$$
\boxed{
D=
\left(
nh_2\cos\theta
-nc\sin\theta
-hd
\right)\sin\alpha
}
$$

因此：

$$
\boxed{
\sin\alpha=
\frac{
D
}{
nh_2\cos\theta
-nc\sin\theta
-hd
}
}
$$

也就是：

$$
\boxed{
\sin\alpha=
\frac{
l_1^2-l_2^2
}{
4(
nh_2\cos\theta
-nc\sin\theta
-hd
)
}
}
$$

------------------------------------------------------------------------

# 7. 当 $h_2=0$ 时的退化情况

如果：

$$
h_2=0
$$

则：

$$
\sin\alpha= -\frac{
l_1^2-l_2^2
}{
4(hd+nc\sin\theta)
}
$$

因此 $h_2$ 会改变 roll 方程中的几何耦合项。

------------------------------------------------------------------------

# 8. 定义 FK 中间变量

为了方便代码实现，定义：

$$
A(\theta) =
dn+hc\sin\theta-hh_2\cos\theta
$$

$$
B(\theta) =
nh_2\cos\theta-nc\sin\theta-hd
$$

$$
N(\theta) =
C-S-2cm\cos\theta-2mh_2\sin\theta
$$

则：

$$
\boxed{
\cos\alpha=\frac{N(\theta)}{2A(\theta)}
}
$$

$$
\boxed{
\sin\alpha=\frac{D}{B(\theta)}
}
$$

------------------------------------------------------------------------

# 9. 求解 Pitch 角 $\theta$

利用：

$$
\sin^2\alpha+\cos^2\alpha=1
$$

得到：

$$
\boxed{
\left(
\frac{N(\theta)}
{2A(\theta)}
\right)^2
+
\left(
\frac{D}
{B(\theta)}
\right)^2
=1
}
$$

展开：

$$
\boxed{
\frac{
\left[
C-S-2cm\cos\theta-2mh_2\sin\theta
\right]^2
}{
4
\left[
dn+hc\sin\theta-hh_2\cos\theta
\right]^2
}
+
\frac{
D^2
}{
\left[
nh_2\cos\theta-nc\sin\theta-hd
\right]^2
}
=1
}
$$

其中：

$$
S=\frac{l_1^2+l_2^2}{2}
$$

$$
D=\frac{l_1^2-l_2^2}{4}
$$

因此给定 $l_1,l_2$ 后，首先求解：

$$
\boxed{
f(\theta)=0
}
$$

------------------------------------------------------------------------

# 10. 最终 Roll 角 $\alpha$

求得 $\theta$ 后：

$$
\sin\alpha= \frac{D}{B(\theta)}
$$

$$
\cos\alpha= \frac{N(\theta)}{2A(\theta)}
$$

因此推荐使用：

$$
\boxed{
\alpha=
\operatorname{atan2}
\left(
\frac{D}{B(\theta)},
\frac{N(\theta)}{2A(\theta)}
\right)
}
$$

## 16. 差分公式修正

对本文第 4 节给出的连接点和旋转矩阵直接展开，两式相减还包含一个
`d*cos(alpha)` 交叉项。实际关系为：

$$
D = \sin\alpha\left[
B(\theta)+2d\left(c\sin\theta-h_2\cos\theta\right)\cos\alpha
\right]
$$

其中 `B(theta)` 与第 8 节定义相同。因此 `sin(alpha)=D/B(theta)` 只在
交叉项为零时成立，不能作为一般机构的正向运动学方程。实现应直接对
两条长度约束进行二维数值求解，并在求解后回代验证长度误差。

完整展开：

$$
\boxed{
\alpha=
\operatorname{atan2}
\left(
\frac{
l_1^2-l_2^2
}{
4(
nh_2\cos\theta
-nc\sin\theta
-hd
)
},
\frac{
C-\frac{l_1^2+l_2^2}{2}
-2cm\cos\theta
-2mh_2\sin\theta
}{
2(
dn+hc\sin\theta-hh_2\cos\theta
)
}
\right)
}
$$

------------------------------------------------------------------------

# 11. 最终 FK

给定：

\[ l_1,l_2 \]

以及机构参数：

\[ m,n,h,c,d,h_2 \]

首先计算：

$$
\boxed{
C=m^2+n^2+h^2+c^2+d^2+h_2^2
}
$$

$$
\boxed{
S=\frac{l_1^2+l_2^2}{2}
}
$$

$$
\boxed{
D=\frac{l_1^2-l_2^2}{4}
}
$$

然后求解：

$$
\boxed{
\frac{
\left[
C-S-2cm\cos\theta-2mh_2\sin\theta
\right]^2
}{
4
\left[
dn+hc\sin\theta-hh_2\cos\theta
\right]^2
}
+
\frac{
D^2
}{
\left[
nh_2\cos\theta-nc\sin\theta-hd
\right]^2
}
=1
}
$$

得到：

$$
\boxed{\theta}
$$

再计算：

$$
\boxed{
\alpha=
\operatorname{atan2}
\left(
\frac{D}{
nh_2\cos\theta-nc\sin\theta-hd
},
\frac{
C-S-2cm\cos\theta-2mh_2\sin\theta
}{
2(dn+hc\sin\theta-hh_2\cos\theta)
}
\right)
}
$$

最终：

$$
\boxed{
FK(l_1,l_2)=
\begin{bmatrix}
\alpha\\
\theta
\end{bmatrix}
}
$$

------------------------------------------------------------------------

# 12. C++ 实现框架

推荐不要把所有公式写成一个长表达式，而是拆成中间变量。

``` cpp
struct AnkleGeometry
{
    double m;
    double n;
    double h;

    double c;
    double d;
    double h2;
};

struct AnkleState
{
    double alpha;
    double theta;
};
```

## 12.1 Pitch 方程

``` cpp
double fkEquation(
    double theta,
    double l1,
    double l2,
    const AnkleGeometry& p)
{
    const double st = std::sin(theta);
    const double ct = std::cos(theta);

    const double C =
        p.m * p.m +
        p.n * p.n +
        p.h * p.h +
        p.c * p.c +
        p.d * p.d +
        p.h2 * p.h2;

    const double S =
        0.5 * (l1 * l1 + l2 * l2);

    const double D =
        0.25 * (l1 * l1 - l2 * l2);

    const double A =
        p.d * p.n +
        p.h * p.c * st -
        p.h * p.h2 * ct;

    const double B =
        p.n * p.h2 * ct -
        p.n * p.c * st -
        p.h * p.d;

    const double N =
        C - S -
        2.0 * p.c * p.m * ct -
        2.0 * p.m * p.h2 * st;

    return
        N * N / (4.0 * A * A) +
        D * D / (B * B) -
        1.0;
}
```

------------------------------------------------------------------------

# 13. 求解 $\theta$

可以使用 Newton、Secant、Bisection 或 Brent。

例如：

``` cpp
double theta = theta_last;

for (int i = 0; i < max_iter; ++i)
{
    double f = fkEquation(
        theta,
        l1,
        l2,
        params);

    double df = numericalDerivative(
        theta,
        l1,
        l2,
        params);

    theta -= f / df;

    if (std::abs(f) < tolerance)
        break;
}
```

在人形机器人实时控制中，可以使用上一周期：

$$
\theta_{k-1}
$$

作为当前周期初值：

$$
\theta_0=\theta_{k-1}
$$

这样通常比每次进行完整区间搜索更加适合实时控制。

------------------------------------------------------------------------

# 14. 求解 $\alpha$

得到 $\theta$ 后：

``` cpp
double st = std::sin(theta);
double ct = std::cos(theta);

double C =
    m*m + n*n + h*h +
    c*c + d*d + h2*h2;

double S =
    0.5 * (l1*l1 + l2*l2);

double D =
    0.25 * (l1*l1 - l2*l2);

double A =
    d*n +
    h*c*st -
    h*h2*ct;

double B =
    n*h2*ct -
    n*c*st -
    h*d;

double N =
    C - S -
    2*c*m*ct -
    2*m*h2*st;

double sin_alpha =
    D / B;

double cos_alpha =
    N / (2.0 * A);

double alpha =
    std::atan2(
        sin_alpha,
        cos_alpha);
```

------------------------------------------------------------------------

# 15. 完整 FK 接口

最终可以封装成：

``` cpp
AnkleState fk(
    double l1,
    double l2,
    const AnkleGeometry& p)
{
    // 1. Calculate constants
    const double C =
        p.m * p.m +
        p.n * p.n +
        p.h * p.h +
        p.c * p.c +
        p.d * p.d +
        p.h2 * p.h2;

    const double S =
        0.5 * (l1 * l1 + l2 * l2);

    const double D =
        0.25 * (l1 * l1 - l2 * l2);

    // 2. Solve theta
    double theta =
        solveTheta(l1, l2, p);

    // 3. Intermediate variables
    const double st = std::sin(theta);
    const double ct = std::cos(theta);

    const double A =
        p.d * p.n +
        p.h * p.c * st -
        p.h * p.h2 * ct;

    const double B =
        p.n * p.h2 * ct -
        p.n * p.c * st -
        p.h * p.d;

    const double N =
        C - S -
        2.0 * p.c * p.m * ct -
        2.0 * p.m * p.h2 * st;

    // 4. Solve alpha
    const double sin_alpha =
        D / B;

    const double cos_alpha =
        N / (2.0 * A);

    const double alpha =
        std::atan2(
            sin_alpha,
            cos_alpha);

    return {
        alpha,
        theta
    };
}
```

------------------------------------------------------------------------

# 16. 数值奇异性

需要重点检查：

$$
\boxed{
A(\theta)=
dn+hc\sin\theta-hh_2\cos\theta
}
$$

以及：

$$
\boxed{
B(\theta)=
nh_2\cos\theta-nc\sin\theta-hd
}
$$

如果：

$$
A(\theta)\rightarrow0
$$

或者：

$$
B(\theta)\rightarrow0
$$

显式 FK 表达式会出现数值问题。

实际控制中还应该进一步计算：

$$
J_l= \frac{\partial(l_1,l_2)}
{\partial(\alpha,\theta)}
$$

并监测：

$$
\sigma_{\min}(J_l)
$$

判断机构是否接近运动学奇异位形。

------------------------------------------------------------------------

# 17. 后续控制相关公式

定义：

$$
\mathbf{l}=
\begin{bmatrix}
l_1\\
l_2
\end{bmatrix}
,\qquad \mathbf{q}=
\begin{bmatrix}
\alpha\\
\theta
\end{bmatrix}
$$

则：

$$
\boxed{
\dot{\mathbf{l}}
=
J_l\dot{\mathbf{q}}
}
$$

其中：

$$
\boxed{
J_l=
\frac{\partial(l_1,l_2)}
{\partial(\alpha,\theta)}
}
$$

根据虚功原理：

$$
F^T\delta l = \tau^T\delta q
$$

因此：

$$
\boxed{
\tau=J_l^TF
}
$$

即：

$$
\boxed{
\begin{bmatrix}
\tau_{roll}\\
\tau_{pitch}
\end{bmatrix}
=
J_l^T
\begin{bmatrix}
F_1\\
F_2
\end{bmatrix}
}
$$

这可以作为后续直线电机推力控制与踝关节力矩控制之间的映射基础。

------------------------------------------------------------------------

# 18. FK 总结

最终机构的运动学关系为：

$$
\boxed{
(l_1,l_2)
\rightarrow
\theta
\rightarrow
\alpha
}
$$

核心公式为：

$$
\boxed{
f(\theta)=
\left(
\frac{N(\theta)}{2A(\theta)}
\right)^2+
\left(
\frac{D}{B(\theta)}
\right)^2-1=0
}
$$

其中：

$$
\boxed{
A(\theta)=dn+hc\sin\theta-hh_2\cos\theta
}
$$

$$
\boxed{
B(\theta)=nh_2\cos\theta-nc\sin\theta-hd
}
$$

$$
\boxed{
N(\theta)=
C-S-2cm\cos\theta-2mh_2\sin\theta
}
$$

$$
\boxed{
C=m^2+n^2+h^2+c^2+d^2+h_2^2
}
$$

$$
\boxed{
S=\frac{l_1^2+l_2^2}{2}
}
$$

$$
\boxed{
D=\frac{l_1^2-l_2^2}{4}
}
$$

求得 $\theta$ 后：

$$
\boxed{
\alpha=
\operatorname{atan2}
\left(
\frac{D}{B(\theta)},
\frac{N(\theta)}{2A(\theta)}
\right)
}
$$
