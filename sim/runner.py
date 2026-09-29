"""仿真运行循环与 CSV 记录"""
import csv
import math
import time
from pathlib import Path

from .env import AnkleSim

FIELDS = (
    "t",
    "s1_cmd", "s2_cmd",                            # 指令行程 (m)
    "l1", "l2", "l1_dot", "l2_dot",                # 编码器：推杆长度 (m)、伸长速度 (m/s)
    "alpha_est", "theta_est", "alpha_dot_est", "theta_dot_est",  # 正解估计 (rad, rad/s)
    "alpha_true", "theta_true", "alpha_dot_true", "theta_dot_true",  # 仿真真值
    "alpha_ref", "theta_ref",                      # 参考角度（trajectory 模式）
)


def run(sim: AnkleSim, ctrl, duration, viewer: bool, realtime: bool,
        log_rate: float, fps: float = 60.0):
    """运行仿真，返回记录的数据行（列顺序见 FIELDS）

    duration=None 时一直运行，直到关闭查看器窗口或 Ctrl+C。
    """
    rows = []
    log_every = max(1, round(1.0 / (log_rate * sim.dt)))
    frame_steps = max(1, round(1.0 / (fps * sim.dt)))
    status_every = max(1, round(0.2 / (frame_steps * sim.dt)))  # 终端状态约 5 Hz

    handle = None
    if viewer:
        import mujoco.viewer
        handle = mujoco.viewer.launch_passive(sim.model, sim.data)

    step = 0
    frame = 0
    wall0 = time.perf_counter()
    try:
        while duration is None or step * sim.dt < duration:
            if handle is not None and not handle.is_running():
                break
            ctrl.poll()

            for _ in range(frame_steps):
                if duration is not None and step * sim.dt >= duration - 1e-9:
                    break
                t = step * sim.dt
                s1, s2, a_ref, t_ref = ctrl.command(t, sim.dt)
                sim.set_stroke(s1, s2)
                sim.step()
                step += 1
                if step % log_every == 0:
                    rows.append((step * sim.dt, s1, s2)
                                + sim.measure_and_estimate() + sim.truth()
                                + (a_ref, t_ref))

            frame += 1
            if handle is not None:
                handle.sync()
            if frame % status_every == 0 and rows:
                r = dict(zip(FIELDS, rows[-1]))
                print(f"\r t={r['t']:6.2f}s  {ctrl.status()}  |  估计 roll "
                      f"{math.degrees(r['alpha_est']):+6.1f}°  pitch "
                      f"{math.degrees(r['theta_est']):+6.1f}°   ", end="", flush=True)

            if realtime:
                lag = step * sim.dt - (time.perf_counter() - wall0)
                if lag > 0:
                    time.sleep(lag)
                elif lag < -4 * frame_steps * sim.dt:
                    # 严重落后（例如拖动窗口）：重设时间基准，不做突发追赶
                    wall0 = time.perf_counter() - step * sim.dt
    except KeyboardInterrupt:
        print("\n[结束] Ctrl+C")
    finally:
        print()
        if handle is not None:
            handle.close()
        ctrl.close()
    return rows


def save_csv(rows, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        writer.writerows(rows)
