"""
生成 calf_actuator 位置正解查表文件

修改 ankle/params.py 中的几何 / 行程 / 限位后需要重新运行，并提交生成的文件。

用法：
    .venv/bin/python scripts/build_lut.py               # 1 mm 步长，写入 model/lut/
    .venv/bin/python scripts/build_lut.py --step 0.5
    .venv/bin/python scripts/build_lut.py --out /tmp/lut.npz
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # 让脚本能导入 ankle

from ankle import LUT_PATH, ROOT, AnkleLUT, calf_actuator_config  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="生成 calf_actuator 位置正解查表文件")
    p.add_argument("--step", type=float, default=1.0, help="网格步长 mm（默认 1）")
    p.add_argument("--out", type=Path, default=LUT_PATH,
                   help=f"输出文件（默认 {LUT_PATH.relative_to(ROOT)}）")
    args = p.parse_args()

    cfg = calf_actuator_config()
    t0 = time.perf_counter()
    lut = AnkleLUT.build(cfg["geometry"], cfg["length_range"], args.step * 1e-3,
                         cfg["alpha_range"], cfg["theta_range"])
    lut.save(args.out)
    print(f"[查表] {lut.n}x{lut.n}，步长 {lut.step*1e3:.3f} mm，"
          f"l ∈ [{lut.l_min:.4f}, {lut.l_max:.4f}] m，可达节点 {lut.valid_ratio:.1%}，"
          f"建表 {time.perf_counter()-t0:.2f} s")
    print(f"[输出] {args.out}（{args.out.stat().st_size / 1024:.0f} KB）")


if __name__ == "__main__":
    main()
