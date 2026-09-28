# MuJoCo 模型（踝关节直线模组机构）

用 MuJoCo 模型对 `ankle_fk_solver.py` 的运动学算法做交叉验证与闭环仿真。

## 文件说明

| 文件 | 说明 |
| --- | --- |
| `ankle_mechanism.xml` | MuJoCo MJCF 模型（由脚本自动生成，请勿手改数值） |
| `build_mjcf.py` | 读取 `param.py` 几何参数生成 XML |
| `verify_fk_mujoco.py` | 模型与解析算法一致性验证（4 项测试） |

## 使用

```bash
# 生成/更新模型（standard 参数）
python model/build_mjcf.py

# 生成 compact 参数的模型
python model/build_mjcf.py compact --out model/ankle_mechanism_compact.xml

# 运行一致性验证（与 XML 同配置）
python model/verify_fk_mujoco.py

# 打开交互式查看器
python -m mujoco.viewer --mjcf model/ankle_mechanism.xml
```

## 模型结构与推导文档的对应关系

- `lower_leg`（小腿机架，焊接在 world 上）
  - 站点 `a1 = (-m, -n, h)`、`a2 = (-m, +n, h)`
- `foot`（脚踝）经两个 hinge 与 `lower_leg` 相连
  - 先 `ankle_roll`（绕 x 轴），再 `ankle_pitch`（绕 y 轴）
  - 对应推导文档的 R = Rx(α)·Ry(θ)（MuJoCo 同 body 多 hinge 按定义顺序合成）
  - 站点 `b1 = (-c, -d, -h2)`、`b2 = (-c, +d, -h2)`
- spatial tendon `module_l1` / `module_l2`：长度 = `|a1-R·b1|`、`|a2-R·b2|`，即两根直线模组的实时长度
- 位置伺服 `motor_l1` / `motor_l2`：`ctrl` 就是目标模组长度（m），可直接作为底层控制量

## 验证内容（`verify_fk_mujoco.py`）

1. **几何一致性**：任意位姿下 MuJoCo 站点距离 = 求解器 `_forward_kinematics`（实测 ~1e-16 m）
2. **FK 求解**：模型测得 (l1, l2) → `solve_fk` → 姿态角一致（实测 ~4e-9 rad）
3. **雅可比一致性**：模型中心差分雅可比 vs `compute_jacobian`（实测 ~3e-9）
4. **伺服仿真**：位置伺服跟踪模组长度指令，仿真收敛姿态与解析解一致

## 备注

- `gravity = "0 0 0"`，便于运动学验证；做重力动力学仿真时改为 `"0 0 -9.81"`
- 所有 geom 关闭碰撞（`contype = 0`）
- 伺服增益 `kp = 5000 N/m`、`kv = 100 N·s/m`，可在 `build_mjcf.py` 常量区调整后重新生成
- 模组行程随关节范围（roll ±0.4 rad，pitch ±0.7 rad）自动计算并写入 `ctrlrange`
