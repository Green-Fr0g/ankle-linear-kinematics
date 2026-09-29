"""
踝关节直线模组仿真控制与记录（命令行入口，实现在 sim/ 包）

两种控制模式，对应实际控制中的两种情况：
    joystick    手柄直接控制两根直线模组（驱动空间控制）
                左摇杆上下 → 模组 1（+y 侧）伸缩速度，右摇杆上下 → 模组 2
    trajectory  给定踝关节角度曲线（关节空间控制），经 inverse_kinematics 换算成模组目标行程

运行过程中后台记录：
    编码器量  推杆长度 l、伸长速度 l̇（仅由 actuatorpos / actuatorvel 传感器得到）
    估计量    forward_kinematics / forward_velocity 算出的 α、θ、α̇、θ̇
    对照量    仿真关节真值、指令行程，以及 trajectory 模式下的参考角度
程序结束后在 logs/<时间>/ 下保存 data.csv，并按类别分别绘制曲线图（actuator/angle/velocity/error.png）。

用法：
    .venv/bin/python scripts/sim_control.py joystick
    .venv/bin/python scripts/sim_control.py trajectory --shape circle --duration 10
    .venv/bin/python scripts/sim_control.py trajectory --shape sine --no-viewer
    .venv/bin/python scripts/sim_control.py trajectory --fk newton        # 切换位置正解方法
    .venv/bin/python scripts/sim_control.py --help / joystick --help / trajectory --help
"""
import argparse
import math
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # 让脚本能导入 ankle / sim

from ankle import FK_METHODS  # noqa: E402
from ankle.paths import LOG_DIR, LUT_PATH  # noqa: E402
from sim import (AnkleSim, JoystickController, TrajectoryController,  # noqa: E402
                 plot, run, save_csv, summarize)


def parse_args():
    p = argparse.ArgumentParser(description="踝关节直线模组仿真控制与记录")
    sub = p.add_subparsers(dest="mode", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 查看器")
    common.add_argument("--no-show", action="store_true", help="只保存曲线图，不弹出窗口")
    common.add_argument("--log-rate", type=float, default=200.0, help="记录频率 Hz（默认 200）")
    common.add_argument("--out", type=Path, default=None,
                        help="输出目录（默认 logs/<时间>）")
    common.add_argument("--fk", choices=FK_METHODS, default="lut-newton",
                        help="位置正解方法：newton 迭代 / lut 查表插值 / "
                             "lut-newton 查表初值 + Newton 精修（默认 lut-newton）")
    common.add_argument("--lut", type=Path, default=LUT_PATH,
                        help="查表文件（默认 model/lut/calf_actuator_fk_lut.npz）")

    js = sub.add_parser("joystick", parents=[common], help="手柄控制两根直线模组")
    js.add_argument("--duration", type=float, default=None,
                    help="运行时长 s（默认一直运行，关闭查看器或 Ctrl+C 结束）")
    js.add_argument("--axis1", type=int, default=1, help="模组 1 对应的轴（默认 1：左摇杆上下）")
    js.add_argument("--axis2", type=int, default=4, help="模组 2 对应的轴（默认 4：右摇杆上下）")
    js.add_argument("--no-invert", action="store_true", help="不取反（默认上推 = 伸长）")
    js.add_argument("--deadzone", type=float, default=0.08, help="摇杆死区（默认 0.08）")
    js.add_argument("--max-speed", type=float, default=0.05,
                    help="摇杆推满时的伸缩速度 m/s（默认 0.05）")
    js.add_argument("--home-button", type=int, default=0, help="回零键编号（默认 0，Xbox 的 A）")
    js.add_argument("--index", type=int, default=0, help="手柄编号（默认 0）")

    tr = sub.add_parser("trajectory", parents=[common], help="给定踝关节角度曲线")
    tr.add_argument("--shape", choices=TrajectoryController.SHAPES, default="sine")
    tr.add_argument("--duration", type=float, default=10.0, help="运行时长 s（默认 10）")
    tr.add_argument("--roll-amp", type=float, default=20.0, help="roll 幅值 deg（默认 20）")
    tr.add_argument("--pitch-amp", type=float, default=25.0, help="pitch 幅值 deg（默认 25）")
    tr.add_argument("--freq", type=float, default=0.5, help="频率 Hz（默认 0.5）")
    tr.add_argument("--ramp", type=float, default=1.0, help="起步渐入时间 s（默认 1）")
    tr.add_argument("--fast", action="store_true",
                    help="不按实时速度运行（无查看器时默认就不按实时）")
    return p.parse_args()


def main():
    args = parse_args()
    sim = AnkleSim(args.fk, args.lut)
    print(f"[正解] {args.fk}" + (f"，查表文件 {args.lut}" if args.fk != "newton" else ""))
    viewer = not args.no_viewer

    if args.mode == "joystick":
        ctrl = JoystickController(args.axis1, args.axis2, not args.no_invert, args.deadzone,
                                  args.max_speed, args.home_button, args.index)
        realtime = True  # 人在回路，始终按实时速度
        title = "手柄控制（驱动空间）"
        if not viewer and args.duration is None:
            print("[提示] 无查看器且未指定 --duration，按 Ctrl+C 结束")
    else:
        ctrl = TrajectoryController(sim.kin, args.shape, math.radians(args.roll_amp),
                                    math.radians(args.pitch_amp), args.freq, args.ramp)
        ctrl.check(args.duration)
        realtime = viewer and not args.fast
        title = (f"关节轨迹控制：{args.shape}，roll ±{args.roll_amp:g}°，"
                 f"pitch ±{args.pitch_amp:g}°，{args.freq:g} Hz")

    title += f"，正解 {args.fk}"
    rows = run(sim, ctrl, args.duration, viewer, realtime, args.log_rate)
    if not rows:
        print("[结束] 没有记录到数据")
        return

    out_dir = args.out or LOG_DIR / f"{datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "data.csv"
    save_csv(rows, csv_path)
    print(f"[输出] 数据 {csv_path}")
    summarize(rows, sim.estimate_failures)
    plot(rows, title, out_dir, show=not args.no_show)


if __name__ == "__main__":
    main()
