import argparse
import csv
from collections import defaultdict
from pathlib import Path
import re

import matplotlib.pyplot as plt


def parse_args():
    p = argparse.ArgumentParser(description="Plot forklift continuity from trace CSV.")
    p.add_argument("--csv", default="logs/forklift-unit-trace.csv", help="Trace CSV path.")
    p.add_argument("--output", default="logs/forklift-unit-continuity.png", help="Output PNG path.")
    p.add_argument("--window-sec", type=float, default=8.0, help="Window length (sec) from start of each session.")
    p.add_argument("--velocity-only", action="store_true", help="Plot velocity-only overlay (baseline + resumed).")
    p.add_argument("--abs-velocity", action="store_true", help="Use absolute value of body_vx for velocity-only plot.")
    p.add_argument(
        "--align-resume-global",
        dest="align_resume_global",
        action="store_true",
        help="Shift resumed session X-axis by saved sim time (from recovery log START step).",
    )
    p.add_argument(
        "--no-align-resume-global",
        dest="align_resume_global",
        action="store_false",
        help="Do not shift resumed session X-axis (use per-session 0-based time).",
    )
    p.add_argument(
        "--recovery-log",
        default="logs/forklift-unit-recovery.log",
        help="Recovery log path used by --align-resume-global.",
    )
    p.set_defaults(align_resume_global=True)
    return p.parse_args()


def load_rows(csv_path):
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            # Skip partially written/corrupted rows (e.g., interrupted append on Ctrl+C).
            if r.get(None):
                continue
            try:
                rows.append(
                    {
                        "session_ts": r["session_ts"],
                        "restored": int(r["restored"]),
                        "step": int(r["step"]),
                        "sim_time_sec": float(r["sim_time_sec"]),
                        "pos_x": float(r["pos_x"]),
                        "yaw": float(r["yaw"]),
                        "body_vx": float(r["body_vx"]),
                        "body_wz": float(r["body_wz"]),
                        "lift_z": float(r["lift_z"]),
                        "target_v": float(r["target_v"]),
                        "phase": int(r["phase"]),
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def group_sessions(rows):
    by_session = defaultdict(list)
    for r in rows:
        by_session[r["session_ts"]].append(r)
    sessions = sorted(by_session.items(), key=lambda kv: kv[0])
    return sessions


def select_pair(sessions):
    if len(sessions) < 2:
        return None
    restored_indexes = [i for i, (_, rows) in enumerate(sessions) if rows and rows[0]["restored"] == 1]
    if restored_indexes:
        i = restored_indexes[-1]
        if i > 0:
            return sessions[i - 1], sessions[i]
    return sessions[-2], sessions[-1]


def trim_window(rows, window_sec):
    return [r for r in rows if r["sim_time_sec"] <= window_sec]


def estimate_timestep(rows):
    for r in rows:
        if r["step"] > 0 and r["sim_time_sec"] > 0.0:
            return r["sim_time_sec"] / float(r["step"])
    return 0.001


def parse_resume_start_step(recovery_log_path, resumed_session_ts):
    path = Path(recovery_log_path)
    if not path.exists():
        return None
    pat = re.compile(r"^\[(?P<ts>[^\]]+)\] START restored=yes .* step=(?P<step>\d+)")
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pat.search(line.strip())
            if not m:
                continue
            if m.group("ts") == resumed_session_ts:
                return int(m.group("step"))
    return None


def plot_pair(prev_session, curr_session, out_path, window_sec):
    (prev_name, prev_rows), (curr_name, curr_rows) = prev_session, curr_session
    prev_rows = trim_window(prev_rows, window_sec)
    curr_rows = trim_window(curr_rows, window_sec)

    if not prev_rows or not curr_rows:
        raise RuntimeError("Not enough data to plot in requested window.")

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(
        "Forklift Continuity (Pre-Restart vs Post-Restore)\n"
        f"prev={prev_name}  restored={curr_name}",
        fontsize=12,
    )

    def draw(ax, key, ylabel):
        ax.plot([r["sim_time_sec"] for r in prev_rows], [r[key] for r in prev_rows], label="pre-restart", linewidth=1.7)
        ax.plot([r["sim_time_sec"] for r in curr_rows], [r[key] for r in curr_rows], label="post-restore", linewidth=1.7)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    draw(axes[0], "pos_x", "pos_x [m]")
    draw(axes[1], "body_vx", "body_vx [m/s]")
    axes[1].plot([r["sim_time_sec"] for r in prev_rows], [r["target_v"] for r in prev_rows], "--", label="target_v(pre)", alpha=0.8)
    axes[1].plot([r["sim_time_sec"] for r in curr_rows], [r["target_v"] for r in curr_rows], "--", label="target_v(post)", alpha=0.8)
    draw(axes[2], "yaw", "yaw [rad]")
    draw(axes[3], "lift_z", "lift_z [m]")
    draw(axes[4], "phase", "phase [-]")

    # Vertical markers:
    # - t=0 of restored session (session switch point in normalized timeline)
    # - first non-zero target_v command in each session (control activation)
    for ax in axes:
        ax.axvline(0.0, color="black", linestyle=":", linewidth=1.0, alpha=0.8)

    def first_nonzero_target_time(rows, eps=0.1):
        for r in rows:
            if abs(r["target_v"]) > eps:
                return r["sim_time_sec"]
        return None

    t_prev_cmd = first_nonzero_target_time(prev_rows)
    t_curr_cmd = first_nonzero_target_time(curr_rows)
    if t_prev_cmd is not None:
        for ax in axes:
            ax.axvline(t_prev_cmd, color="#1f77b4", linestyle="--", linewidth=1.0, alpha=0.8)
    if t_curr_cmd is not None:
        for ax in axes:
            ax.axvline(t_curr_cmd, color="#ff7f0e", linestyle="--", linewidth=1.0, alpha=0.8)

    # Legend handle explanation on top panel
    axes[0].plot([], [], color="black", linestyle=":", label="restore start (t=0)")
    if t_prev_cmd is not None:
        axes[0].plot([], [], color="#1f77b4", linestyle="--", label=f"pre cmd start ({t_prev_cmd:.2f}s)")
    if t_curr_cmd is not None:
        axes[0].plot([], [], color="#ff7f0e", linestyle="--", label=f"post cmd start ({t_curr_cmd:.2f}s)")

    axes[4].set_xlabel("sim_time_sec from session start")
    axes[0].legend(loc="best", ncol=2)
    axes[1].legend(loc="best", ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)


def plot_velocity_only(
    prev_session,
    curr_session,
    out_path,
    window_sec,
    abs_velocity=False,
    align_resume_global=False,
    recovery_log_path="logs/forklift-unit-recovery.log",
):
    (prev_name, prev_rows), (curr_name, curr_rows) = prev_session, curr_session
    prev_rows = trim_window(prev_rows, window_sec)
    curr_rows = trim_window(curr_rows, window_sec)
    if not prev_rows or not curr_rows:
        raise RuntimeError("Not enough data to plot in requested window.")

    fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    fig.suptitle(
        "Forklift Velocity Overlay (Baseline vs Resumed)\n"
        f"baseline={prev_name}  resumed={curr_name}",
        fontsize=12,
    )

    def y(rows):
        if abs_velocity:
            return [abs(r["body_vx"]) for r in rows]
        return [r["body_vx"] for r in rows]

    ax.plot([r["sim_time_sec"] for r in prev_rows], y(prev_rows), label="baseline body_vx", linewidth=1.8)
    curr_x = [r["sim_time_sec"] for r in curr_rows]
    offset = 0.0
    if align_resume_global:
        step_saved = parse_resume_start_step(recovery_log_path, curr_name)
        if step_saved is not None:
            dt = estimate_timestep(curr_rows)
            expected_offset = float(step_saved) * dt
            first_t = curr_rows[0]["sim_time_sec"] if curr_rows else 0.0
            # If resumed trace already uses global/continued sim_time, avoid double-shifting.
            if abs(first_t - expected_offset) <= max(0.25, dt * 20.0):
                offset = 0.0
                print(
                    "[INFO] align_resume_global: resumed trace already in global time "
                    f"(first_t={first_t:.3f}s ~ expected_offset={expected_offset:.3f}s). "
                    "skip additional offset."
                )
            else:
                offset = expected_offset
                curr_x = [x + offset for x in curr_x]
                print(f"[INFO] align_resume_global: step_saved={step_saved}, dt={dt:.6f}, offset={offset:.3f}s")
        else:
            print("[WARN] align_resume_global requested, but resume START step was not found in recovery log.")
    ax.plot(curr_x, y(curr_rows), label="resumed body_vx", linewidth=1.8)
    ax.plot([r["sim_time_sec"] for r in prev_rows], [r["target_v"] for r in prev_rows], "--", label="baseline target_v", alpha=0.8)
    ax.plot(curr_x, [r["target_v"] for r in curr_rows], "--", label="resumed target_v", alpha=0.8)

    def first_nonzero_target_time(rows, eps=0.1):
        for r in rows:
            if abs(r["target_v"]) > eps:
                return r["sim_time_sec"]
        return None

    t_prev_cmd = first_nonzero_target_time(prev_rows)
    t_curr_cmd = first_nonzero_target_time(curr_rows)
    if t_prev_cmd is not None:
        ax.axvline(t_prev_cmd, color="#1f77b4", linestyle="--", linewidth=1.0, alpha=0.8)
    if t_curr_cmd is not None:
        ax.axvline(t_curr_cmd + offset, color="#ff7f0e", linestyle="--", linewidth=1.0, alpha=0.8)

    if align_resume_global:
        ax.set_xlabel("sim_time_sec (baseline) + resumed shifted by saved step time")
    else:
        ax.set_xlabel("sim_time_sec from session start")
    ax.set_ylabel("|body_vx| [m/s]" if abs_velocity else "body_vx [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)


def main():
    args = parse_args()
    csv_path = Path(args.csv)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    rows = load_rows(csv_path)
    sessions = group_sessions(rows)
    pair = select_pair(sessions)
    if pair is None:
        raise RuntimeError("Need at least 2 sessions in CSV for continuity comparison.")
    if args.velocity_only:
        plot_velocity_only(
            pair[0],
            pair[1],
            out_path,
            args.window_sec,
            abs_velocity=args.abs_velocity,
            align_resume_global=args.align_resume_global,
            recovery_log_path=args.recovery_log,
        )
    else:
        plot_pair(pair[0], pair[1], out_path, args.window_sec)
    print(f"[OK] plot saved: {out_path}")


if __name__ == "__main__":
    main()
