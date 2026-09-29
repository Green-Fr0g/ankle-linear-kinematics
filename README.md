# 踝关节直线模组运动学

人形机器人小腿的双直线模组并联踝关节：两根推杆（长度 l1、l2）驱动踝关节 roll（α）和 pitch（θ）。本仓库提供：

- 运动学库 `ankle/`：位置、速度、力的正逆解，纯 Python，只依赖 numpy，可直接移植。
- 三种位置正解方法：Newton 迭代、查表插值、查表初值加 Newton 精修。
- MuJoCo 仿真 `sim/`：用手柄或关节轨迹驱动模型，只用编码器数据做状态估计，并和仿真真值对比。

| | 逆解 q → l（解析） | 正解 l → q |
|---|---|---|
| 位置 | `inverse_kinematics` | `forward_kinematics`（Newton / 查表） |
| 速度 | `inverse_velocity`：l̇ = J q̇ | `forward_velocity`：q̇ = J⁻¹ l̇ |
| 力 | `torque_to_force`：F = J⁻ᵀ τ | `force_to_torque`：τ = Jᵀ F |

J = ∂l/∂q 是 2×2 雅可比矩阵。

## 目录结构

```
ankle/                  运动学库（不依赖 MuJoCo）
  geometry.py           几何参数 AnkleGeometry、状态 AnkleState、坐标约定
  params.py             机构参数：安装点、零位长度、行程、关节限位
  kinematics.py         AnkleKinematics：解析逆解 + Newton 正解 + 速度 / 力 / 奇异性
  lut.py                查表 AnkleLUT，以及查表正解 LutKinematics / LutNewtonKinematics
  factory.py            make_kinematics：按名称选择正解方法
  paths.py              模型、查表、日志的路径
sim/                    MuJoCo 仿真
  env.py                AnkleSim：加载模型、下发行程、读编码器并做状态估计
  controllers.py        JoystickController（手柄）/ TrajectoryController（关节轨迹）
  runner.py             仿真循环、CSV 记录
  report.py             误差统计、绘图
scripts/                命令行入口
  sim_control.py        运行仿真
  bench_fk.py           对比三种正解方法的精度和耗时
  build_lut.py          生成查表文件
tests/                  单元测试
model/xml/              MuJoCo 模型（calf_actuator.xml 是当前使用的闭链模型）
model/meshes/           STL 网格
model/lut/              位置正解查表文件（已提交）
doc/                    推导文档：fk_derivation.md（几何与公式）、fk_newton_solver.md（Newton 求解细节）
logs/                   仿真输出（不提交）
```

依赖方向：`scripts` → `sim` → `ankle`。`ankle` 不引用 `sim`，单独拿走也能用。

## 环境

需要 Python 3.10+。

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

以下命令都在仓库根目录执行。脚本本身不依赖当前目录，在别的目录运行也可以。

## 快速开始

### 1. 仿真

```bash
# 关节轨迹控制：circle / sine / pitch / roll
.venv/bin/python scripts/sim_control.py trajectory --shape circle --duration 10

# 不开查看器、不弹出图窗，适合批量跑
.venv/bin/python scripts/sim_control.py trajectory --shape sine --no-viewer --no-show

# 手柄控制：左摇杆上下控制模组 1，右摇杆上下控制模组 2，A 键回零
.venv/bin/python scripts/sim_control.py joystick

# 切换正解方法（默认 lut-newton）
.venv/bin/python scripts/sim_control.py trajectory --fk newton
```

每次运行输出到 `logs/<年月日_时分秒>/`，也可以用 `--out` 指定目录：

- `data.csv`：指令行程、编码器长度和速度、估计角度和角速度、仿真真值、参考轨迹
- `actuator.png`：推杆长度、伸长速度
- `angle.png`、`velocity.png`：关节角度、角速度（真值 / 估计 / 参考）
- `error.png`：估计误差，轨迹模式下还有跟踪误差

其他参数（幅值、频率、手柄轴号、死区、最大速度等）见 `--help`、`trajectory --help`、`joystick --help`。

跟踪误差主要来自 MuJoCo 位置执行器的一阶滞后，约为 (kv/kp)·ṡ（当前约 20 ms），不是正解误差。估计误差本身只有 0.01° 量级。

### 2. 在代码中使用运动学库

```python
from ankle import make_kinematics, calf_actuator_config

kin = make_kinematics("lut-newton", **calf_actuator_config())

# 控制下发：期望关节角 / 角速度 / 力矩 → 推杆
l1, l2 = kin.inverse_kinematics(alpha, theta)
l1_dot, l2_dot = kin.inverse_velocity(alpha, theta, alpha_dot, theta_dot)
f1, f2 = kin.torque_to_force(alpha, theta, tau_roll, tau_pitch)

# 状态估计：编码器 → 关节
st = kin.forward_kinematics(l1, l2)          # st.alpha, st.theta（rad）
alpha_dot, theta_dot = kin.forward_velocity(st.alpha, st.theta, l1_dot, l2_dot)
tau_roll, tau_pitch = kin.force_to_torque(st.alpha, st.theta, f1, f2)
```

单位：长度 m，角度 rad，力 N，力矩 N·m。输入超出查表范围、落入不可达区域、解超出关节限位或不收敛时，`forward_kinematics` 抛出 `ValueError`；雅可比接近奇异时，速度和力映射抛出 `SingularityError`（`ValueError` 的子类）。状态估计时捕获异常后调用 `kin.reset()`，下一帧会重新求解。

## 位置正解方法

| 方法 | 做法 | 精度 | 耗时 | 适用 |
|---|---|---|---|---|
| `newton` | 从上一帧结果热启动的 Newton 迭代，失败时从零位重解 | 约 1e-6° | 约 30 us | 不想依赖查表文件时 |
| `lut` | 查表 + 四角双线性插值，不迭代 | 约 7e-3° | 约 2 us | 对耗时要求严格、精度要求一般 |
| `lut-newton` | 查表值作初值，再做 Newton 精修，约 1 次迭代 | 约 5e-7° | 约 12 us | 默认，推荐 |

数据来自 `scripts/bench_fk.py` 的随机姿态冷启动测试，耗时是本机纯 Python 的结果。

查表思路参考 agibot_x1_infer 的 `AnkleParallelTransmission`，同时修正了它的几个问题：只用对角两点插值、超出表格时静默输出 0，以及用字符串做哈希键。

对比命令：

```bash
.venv/bin/python scripts/bench_fk.py                 # 读取查表文件
.venv/bin/python scripts/bench_fk.py --step 0.5      # 按 0.5 mm 步长现场建表对比
```

## 查表文件

`model/lut/calf_actuator_fk_lut.npz` 是 (l1, l2) 平面上的 81×81 网格，步长 1 mm，覆盖 ±40 mm 行程，大小 91 KB。每个节点存 (α, θ)，不可达节点存 NaN。

文件里记录了建表时的几何参数、关节限位和长度范围。每次读取都会和 `ankle/params.py` 比对，不一致就报错，提示重新生成，所以不会静默用到旧表。

修改 `ankle/params.py` 里的 `CALF_ACTUATOR_*` 参数后，需要：

```bash
.venv/bin/python scripts/build_lut.py               # 默认 1 mm；可用 --step 0.5 调整
.venv/bin/python -m unittest discover -s tests -t .
git add model/lut/calf_actuator_fk_lut.npz
```

## 修改机构参数

参数都在 `ankle/params.py`，坐标系原点是踝关节旋转中心，方向与 `model/xml/calf_actuator.xml` 相同：

- `CALF_ACTUATOR_GEOMETRY`：a1（小腿侧安装点）和 b1（脚踝侧安装点）。a2、b2 是它们关于 xz 平面的镜像。
- `CALF_ACTUATOR_STROKE_RANGE`：模组行程，相对零位长度。
- `CALF_ACTUATOR_ROLL_RANGE`、`CALF_ACTUATOR_PITCH_RANGE`：关节限位。
- `CALF_ACTUATOR_ESTIMATOR_MARGIN`：状态估计时额外放宽的限位。MuJoCo 的限位是软约束，撞限位时关节会略微越界。

修改后 XML 也要同步改。测试 `TestMatchesXml` 会检查 IK 算出的推杆长度和 MuJoCo 模型中 |a_i − b_i| 是否一致。改完后重新生成查表文件，见上一节。

## 测试

```bash
.venv/bin/python -m unittest discover -s tests -t . -v
```

测试覆盖以下内容：

- 零位长度
- 雅可比与有限差分一致
- 速度、力的往返一致，以及虚功关系
- 三种正解都能还原逆解输入
- 表外输入报错
- 查表参数不一致或文件缺失时报错
- 参数与 XML 对齐

