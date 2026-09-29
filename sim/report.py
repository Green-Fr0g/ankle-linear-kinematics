"""仿真结果统计与绘图（列定义见 runner.FIELDS）"""
import math
from pathlib import Path

from ankle.params import CALF_ACTUATOR_NOMINAL_LENGTH as L0

from .runner import FIELDS

NAN = float("nan")


def _columns(rows):
    return {name: [r[i] for r in rows] for i, name in enumerate(FIELDS)}


def _err_stats(est, ref):
    """(最大绝对误差, RMS)，忽略 NaN"""
    e = [a - b for a, b in zip(est, ref) if not (math.isnan(a) or math.isnan(b))]
    if not e:
        return NAN, NAN
    return max(abs(x) for x in e), math.sqrt(sum(x * x for x in e) / len(e))


def summarize(rows, failures: int):
    c = _columns(rows)
    print(f"[统计] 记录 {len(rows)} 帧，时长 {c['t'][-1]:.2f} s，估计失败 {failures} 次")
    for key, unit, scale in (("alpha", "deg", 180 / math.pi), ("theta", "deg", 180 / math.pi),
                             ("alpha_dot", "deg/s", 180 / math.pi),
                             ("theta_dot", "deg/s", 180 / math.pi)):
        mx, rms = _err_stats(c[f"{key}_est"], c[f"{key}_true"])
        print(f"  估计误差 {key:<9}: 最大 {mx*scale:8.4f} {unit}, RMS {rms*scale:8.4f} {unit}")
    if not all(math.isnan(v) for v in c["alpha_ref"]):
        for key in ("alpha", "theta"):
            mx, rms = _err_stats(c[f"{key}_est"], c[f"{key}_ref"])
            print(f"  跟踪误差 {key:<9}: 最大 {math.degrees(mx):8.4f} deg, "
                  f"RMS {math.degrees(rms):8.4f} deg")


def _setup_matplotlib(show: bool):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    # 选一个系统里已有的中文字体，避免标签显示成方框。
    # Noto CJK 的 .ttc 在 matplotlib 中常被登记为 "Noto Sans CJK JP"，字形同样覆盖中文，
    # 所以按关键字匹配而不是精确名称。
    available = sorted({f.name for f in font_manager.fontManager.ttflist})
    for key in ("Noto Sans CJK", "WenQuanYi", "Microsoft YaHei", "SimHei", "PingFang",
                "Droid Sans Fallback", "Noto Serif CJK", "AR PL"):
        match = next((n for n in available if key in n), None)
        if match:
            plt.rcParams["font.sans-serif"] = [match] + plt.rcParams["font.sans-serif"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    # 字号与线宽：默认值在大图上显得太小
    plt.rcParams.update({
        "font.size": 15,
        "axes.labelsize": 16,
        "xtick.labelsize": 14,
        "ytick.labelsize": 14,
        "legend.fontsize": 13,
        "figure.titlesize": 19,
        "lines.linewidth": 2.0,
    })
    return plt


def _finish(plt, fig, axes, title: str, path: Path):
    """统一收尾：网格、图例、标题、横轴标签，并保存"""
    fig.suptitle(title)
    for a in axes:
        a.grid(True, alpha=0.3)
        a.legend(loc="best", framealpha=0.85)
    axes[-1].set_xlabel("时间 (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"[输出] 曲线图 {path}")


def plot(rows, title: str, out_dir: Path, show: bool):
    """每类数据单独一张图，保存到 out_dir：
    actuator.png  推杆长度 / 伸长速度
    angle.png     roll / pitch 角度（真值、估计、参考）
    velocity.png  roll / pitch 角速度（真值、估计）
    error.png     角度估计误差 / 角速度估计误差 [/ 跟踪误差]
    """
    plt = _setup_matplotlib(show)
    c = _columns(rows)
    t = c["t"]
    deg = lambda xs: [math.degrees(x) for x in xs]
    mm = lambda xs: [x * 1e3 for x in xs]
    diff = lambda a, b: [x - y for x, y in zip(a, b)]
    has_ref = not all(math.isnan(v) for v in c["alpha_ref"])
    joints = (("alpha", "roll α"), ("theta", "pitch θ"))

    # 1. 驱动空间：推杆长度与伸长速度
    fig, ax = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
    for i, color in ((1, "tab:blue"), (2, "tab:orange")):
        ax[0].plot(t, mm(c[f"l{i}"]), color=color, label=f"l{i} 编码器")
        ax[0].plot(t, mm([L0 + s for s in c[f"s{i}_cmd"]]), "--", color=color,
                   lw=1.5, label=f"l{i} 指令")
        ax[1].plot(t, mm(c[f"l{i}_dot"]), color=color, label=f"模组{i}")
    ax[0].set_ylabel("推杆长度 (mm)")
    ax[1].set_ylabel("伸长速度 (mm/s)")
    _finish(plt, fig, ax, f"{title}\n直线模组", out_dir / "actuator.png")

    # 2. 关节角度 / 3. 关节角速度
    for suffix, unit, name, fname in (("", "deg", "关节角度", "angle.png"),
                                      ("_dot", "deg/s", "关节角速度", "velocity.png")):
        fig, ax = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
        for a, (key, label) in zip(ax, joints):
            a.plot(t, deg(c[f"{key}{suffix}_true"]), color="0.6", lw=6, alpha=0.6,
                   label="仿真真值")
            a.plot(t, deg(c[f"{key}{suffix}_est"]), color="tab:green", lw=2,
                   label="正解估计")
            if has_ref and suffix == "":
                a.plot(t, deg(c[f"{key}_ref"]), "k--", lw=1.6, label="参考轨迹")
            a.set_ylabel(f"{label} ({unit})")
        _finish(plt, fig, ax, f"{title}\n{name}", out_dir / fname)

    # 4. 误差：估计误差（估计 - 真值），轨迹模式另加跟踪误差（估计 - 参考）
    panels = [("", "角度估计误差 (deg)"), ("_dot", "角速度估计误差 (deg/s)")]
    fig, ax = plt.subplots(len(panels) + has_ref, 1,
                           figsize=(16, 5 * (len(panels) + has_ref)), sharex=True)
    for a, (suffix, ylabel) in zip(ax, panels):
        for key, label in joints:
            a.plot(t, deg(diff(c[f"{key}{suffix}_est"], c[f"{key}{suffix}_true"])),
                   label=label)
        a.set_ylabel(ylabel)
    if has_ref:
        for key, label in joints:
            ax[-1].plot(t, deg(diff(c[f"{key}_est"], c[f"{key}_ref"])), label=label)
        ax[-1].set_ylabel("跟踪误差 (deg)")
    _finish(plt, fig, ax, f"{title}\n误差", out_dir / "error.png")

    if show:
        plt.show()
