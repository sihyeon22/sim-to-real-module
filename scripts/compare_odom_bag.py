#!/usr/bin/env python3
"""
Compare /odom (GT), /odom1, /odom2 from a rosbag2 bag and plot graphs.

Usage:
    python3 compare_odom_bag.py ~/bag_odom_test
    python3 compare_odom_bag.py ~/bag_odom_test --max-dt 0.20
"""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


GT_TOPIC    = "/odom"
ODOM1_TOPIC = "/odom1"
ODOM2_TOPIC = "/odom2"


@dataclass
class Pose2D:
    stamp_ns: int
    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_odometry(msg) -> Pose2D:
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    return Pose2D(
        stamp_ns=stamp_ns,
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        yaw=quat_to_yaw(msg.pose.pose.orientation),
    )


def nearest_by_time(poses: Sequence[Pose2D], target_ns: int, max_dt_ns: int) -> Optional[Pose2D]:
    if not poses:
        return None
    stamps = [p.stamp_ns for p in poses]
    idx = bisect.bisect_left(stamps, target_ns)
    candidates: List[Pose2D] = []
    if idx < len(poses):
        candidates.append(poses[idx])
    if idx > 0:
        candidates.append(poses[idx - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda p: abs(p.stamp_ns - target_ns))
    return best if abs(best.stamp_ns - target_ns) <= max_dt_ns else None


def read_bag(bag_path: Path) -> Tuple[List[Pose2D], List[Pose2D], List[Pose2D]]:
    mcap_files = list(bag_path.glob("*.mcap"))
    storage_id = "mcap" if mcap_files else "sqlite3"

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    def get_type(topic):
        return get_message(topic_types[topic]) if topic in topic_types else None

    gt_type    = get_type(GT_TOPIC)
    odom1_type = get_type(ODOM1_TOPIC)
    odom2_type = get_type(ODOM2_TOPIC)

    gt_poses:    List[Pose2D] = []
    odom1_poses: List[Pose2D] = []
    odom2_poses: List[Pose2D] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == GT_TOPIC and gt_type:
            gt_poses.append(pose_from_odometry(deserialize_message(data, gt_type)))
        elif topic == ODOM1_TOPIC and odom1_type:
            odom1_poses.append(pose_from_odometry(deserialize_message(data, odom1_type)))
        elif topic == ODOM2_TOPIC and odom2_type:
            odom2_poses.append(pose_from_odometry(deserialize_message(data, odom2_type)))

    return gt_poses, odom1_poses, odom2_poses


def poses_to_arrays(poses: List[Pose2D], t0_ns: int):
    t   = np.array([(p.stamp_ns - t0_ns) * 1e-9 for p in poses])
    x   = np.array([p.x   for p in poses])
    y   = np.array([p.y   for p in poses])
    yaw = np.array([p.yaw for p in poses])
    return t, x, y, yaw


def match_to_gt(gt_poses, est_poses, max_dt_ns, t0_ns):
    t_list = []
    xg, yg, yawg = [], [], []
    xe, ye, yawe = [], [], []
    for est in est_poses:
        gt = nearest_by_time(gt_poses, est.stamp_ns, max_dt_ns)
        if gt is None:
            continue
        t_list.append((est.stamp_ns - t0_ns) * 1e-9)
        xg.append(gt.x);   yg.append(gt.y);   yawg.append(gt.yaw)
        xe.append(est.x);  ye.append(est.y);  yawe.append(est.yaw)
    return (
        np.array(t_list),
        np.array(xg), np.array(yg), np.array(yawg),
        np.array(xe), np.array(ye), np.array(yawe),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Path to rosbag2 folder (e.g. ~/bag_odom_test)")
    parser.add_argument("--max-dt", type=float, default=0.20,
                        help="Max timestamp diff [s] for matching (default: 0.20)")
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")

    print(f"Bag: {bag_path}")
    gt_poses, odom1_poses, odom2_poses = read_bag(bag_path)
    print(f"  /odom  (GT) : {len(gt_poses)} samples")
    print(f"  /odom1      : {len(odom1_poses)} samples")
    print(f"  /odom2      : {len(odom2_poses)} samples")

    if not gt_poses:
        print("[ERROR] /odom (GT) 데이터가 없습니다.")
        return

    max_dt_ns = int(args.max_dt * 1_000_000_000)
    t0_ns = gt_poses[0].stamp_ns

    t_gt, x_gt, y_gt, yaw_gt = poses_to_arrays(gt_poses, t0_ns)
    t1, xg1, yg1, yawg1, x1, y1, yaw1 = match_to_gt(gt_poses, odom1_poses, max_dt_ns, t0_ns)
    t2, xg2, yg2, yawg2, x2, y2, yaw2 = match_to_gt(gt_poses, odom2_poses, max_dt_ns, t0_ns)

    pos_err1 = np.hypot(x1 - xg1, y1 - yg1) if len(t1) else np.array([])
    pos_err2 = np.hypot(x2 - xg2, y2 - yg2) if len(t2) else np.array([])
    yaw_err1 = np.array([wrap_angle(a - b) for a, b in zip(yaw1, yawg1)])
    yaw_err2 = np.array([wrap_angle(a - b) for a, b in zip(yaw2, yawg2)])

    # 통계 출력
    print()
    for label, pos_err, yaw_err in [("odom1", pos_err1, yaw_err1), ("odom2", pos_err2, yaw_err2)]:
        if len(pos_err) == 0:
            print(f"  {label}: 매칭 데이터 없음")
            continue
        print(f"  {label} vs GT  ({len(pos_err)} samples)")
        print(f"    pos  avg={pos_err.mean():.4f}m  max={pos_err.max():.4f}m")
        print(f"    yaw  avg={math.degrees(float(np.abs(yaw_err).mean())):.3f}°  "
              f"max={math.degrees(float(np.abs(yaw_err).max())):.3f}°")

    # 그래프
    fig = plt.figure(figsize=(18, 13))
    fig.suptitle(f"Odometry Comparison  |  {bag_path.name}", fontsize=13, fontweight="bold")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    # X over time
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(t_gt, x_gt, "k-", lw=1.5, label="GT (/odom)")
    if len(t1): ax.plot(t1, x1, "b--", lw=1.0, label="odom1")
    if len(t2): ax.plot(t2, x2, "r:",  lw=1.0, label="odom2")
    ax.set_title("X position"); ax.set_xlabel("time [s]"); ax.set_ylabel("x [m]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)

    # Y over time
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t_gt, y_gt, "k-", lw=1.5, label="GT (/odom)")
    if len(t1): ax.plot(t1, y1, "b--", lw=1.0, label="odom1")
    if len(t2): ax.plot(t2, y2, "r:",  lw=1.0, label="odom2")
    ax.set_title("Y position"); ax.set_xlabel("time [s]"); ax.set_ylabel("y [m]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)

    # Yaw over time
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(t_gt, np.degrees(yaw_gt), "k-", lw=1.5, label="GT (/odom)")
    if len(t1): ax.plot(t1, np.degrees(yaw1), "b--", lw=1.0, label="odom1")
    if len(t2): ax.plot(t2, np.degrees(yaw2), "r:",  lw=1.0, label="odom2")
    ax.set_title("Yaw"); ax.set_xlabel("time [s]"); ax.set_ylabel("yaw [deg]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)

    # XY trajectory
    ax = fig.add_subplot(gs[1, :2])
    ax.plot(x_gt, y_gt, "k-", lw=1.5, label="GT (/odom)")
    if len(x1): ax.plot(x1, y1, "b--", lw=1.0, label="odom1")
    if len(x2): ax.plot(x2, y2, "r:",  lw=1.0, label="odom2")
    ax.plot(x_gt[0], y_gt[0], "ko", ms=7)
    ax.plot(x_gt[-1], y_gt[-1], "ks", ms=7)
    ax.set_title("XY Trajectory  (● start  ■ end)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    ax.set_aspect("equal", adjustable="datalim")

    # Position error over time
    ax = fig.add_subplot(gs[1, 2])
    if len(t1): ax.plot(t1, pos_err1, "b-", lw=1.0,
                        label=f"odom1  avg={pos_err1.mean():.3f}m")
    if len(t2): ax.plot(t2, pos_err2, "r-", lw=1.0,
                        label=f"odom2  avg={pos_err2.mean():.3f}m")
    ax.set_title("Position error vs GT")
    ax.set_xlabel("time [s]"); ax.set_ylabel("error [m]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)

    # Yaw error over time
    ax = fig.add_subplot(gs[2, 0])
    if len(t1): ax.plot(t1, np.degrees(yaw_err1), "b-", lw=1.0,
                        label=f"odom1  avg={np.degrees(np.abs(yaw_err1).mean()):.2f}°")
    if len(t2): ax.plot(t2, np.degrees(yaw_err2), "r-", lw=1.0,
                        label=f"odom2  avg={np.degrees(np.abs(yaw_err2).mean()):.2f}°")
    ax.axhline(0, color="k", lw=0.8, ls="--")
    ax.set_title("Yaw error vs GT")
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw error [deg]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)

    # Summary table
    ax = fig.add_subplot(gs[2, 1:])
    ax.axis("off")
    col_labels = ["", "avg pos [m]", "max pos [m]", "avg yaw [°]", "max yaw [°]", "samples"]
    rows = []
    for label, pos_err, yaw_err in [("odom1", pos_err1, yaw_err1), ("odom2", pos_err2, yaw_err2)]:
        if len(pos_err) == 0:
            rows.append([label, "N/A", "N/A", "N/A", "N/A", "0"])
        else:
            rows.append([
                label,
                f"{pos_err.mean():.4f}",
                f"{pos_err.max():.4f}",
                f"{np.degrees(np.abs(yaw_err).mean()):.3f}",
                f"{np.degrees(np.abs(yaw_err).max()):.3f}",
                str(len(pos_err)),
            ])
    table = ax.table(cellText=rows, colLabels=col_labels, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2.2)
    ax.set_title("Summary Statistics", pad=12)

    out_path = bag_path.parent / "odom_comparison.png"
    plt.savefig(str(out_path), dpi=150, bbox_inches="tight")
    print(f"\n[INFO] 그래프 저장: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
