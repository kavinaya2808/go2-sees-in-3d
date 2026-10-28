#!/usr/bin/env python3
"""Compare fused detections with Gazebo ground truth.

Protocol (one "viewpoint" per run):
  1. Drive the Go2 somewhere with teleop, then stop it (press k) and let it settle.
  2. Run this script. It reads the true poses of the robot and the objects from Gazebo,
     records N fused scans (/fusion/alerts JSON) and appends one CSV row per object per scan.
  3. Repeat from a few different viewpoints. Every run prints the summary over all
     viewpoints recorded so far.

Positions are compared in the LiDAR frame, horizontally (x, y). The LiDAR only sees the
near faces of an object, so the box centre it reports sits slightly in front of the true
centre; that bias is part of the error this script measures.

Usage (inside the container, with the sim and the perception launch running):
    python3 tools/evaluate.py --label person1=person
    python3 tools/evaluate.py --summary-only
"""

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from statistics import mean, median

import numpy as np

WORLD = 'default'
ROBOT_MODEL = 'go2'
DEFAULT_OBJECTS = 'person1,box1,box2,box3,cylinder1,cylinder2'
CAMERA_HALF_FOV_DEG = 40.0  # D455 colour camera is ~42.7 deg; keep a small margin
CSV_FIELDS = [
    'viewpoint', 'scan', 'object', 'expected_class', 'gt_x', 'gt_y', 'gt_range',
    'in_camera_fov', 'detected', 'est_x', 'est_y', 'est_class', 'sensors',
    'pos_error', 'range_error',
]


# ---------------------------------------------------------------- ground truth parsing

def _vec(block: str, keys: str, defaults: dict) -> dict:
    """Read x/y/z(/w) fields from a protobuf text block. Proto3 omits zero values."""
    out = dict(defaults)
    for key, value in re.findall(r'\b([xyzw]):\s*(-?[\d.]+(?:e[-+]?\d+)?)', block):
        if key in keys:
            out[key] = float(value)
    return out


def parse_pose_info(text: str) -> dict:
    """Parse `gz topic -e -t /world/<w>/pose/info -n 1` output into {name: (pos, quat)}.

    pos is np.array([x, y, z]); quat is np.array([x, y, z, w]). The first entry with a
    given name wins (top-level models come before their links).
    """
    poses = {}
    for body in re.findall(r'^pose \{\n(.*?)^\}', text, flags=re.M | re.S):
        name = re.search(r'name:\s*"([^"]+)"', body)
        if not name or name.group(1) in poses:
            continue
        pos_block = re.search(r'position \{(.*?)\}', body, flags=re.S)
        ori_block = re.search(r'orientation \{(.*?)\}', body, flags=re.S)
        p = _vec(pos_block.group(1), 'xyz', {'x': 0.0, 'y': 0.0, 'z': 0.0}) if pos_block \
            else {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if ori_block:
            q = _vec(ori_block.group(1), 'xyzw', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0})
        else:
            q = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}  # identity is omitted entirely
        poses[name.group(1)] = (
            np.array([p['x'], p['y'], p['z']]),
            np.array([q['x'], q['y'], q['z'], q['w']]),
        )
    return poses


def read_ground_truth() -> dict:
    cmd = ['gz', 'topic', '-e', '-t', f'/world/{WORLD}/pose/info', '-n', '1']
    text = subprocess.run(cmd, capture_output=True, text=True, timeout=20).stdout
    poses = parse_pose_info(text)
    if ROBOT_MODEL not in poses:
        sys.exit(f'Could not find model "{ROBOT_MODEL}" in {cmd}. Is the sim running?')
    return poses


# ---------------------------------------------------------------- geometry

def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def world_to_sensor(p_world, robot_pos, robot_quat, sensor_R, sensor_t) -> np.ndarray:
    """World point -> robot base frame -> LiDAR frame.

    sensor_R, sensor_t map base-frame points into the LiDAR frame
    (tf2 lookup_transform(target='velodyne', source='base_link')).
    """
    p_base = quat_to_matrix(robot_quat).T @ (np.asarray(p_world) - robot_pos)
    return sensor_R @ p_base + sensor_t


# ---------------------------------------------------------------- scoring

def score_scan(gt: list, estimates: list, gate: float) -> list:
    """Greedy nearest-neighbour association of ground-truth objects to estimates.

    gt:        [(name, expected_class, xy np.array)]
    estimates: [(class, sensors, xy np.array)]
    Returns one dict per ground-truth object.
    """
    pairs = sorted(
        (float(np.linalg.norm(g[2] - e[2])), gi, ei)
        for gi, g in enumerate(gt) for ei, e in enumerate(estimates))
    gt_match, used = {}, set()
    for dist, gi, ei in pairs:
        if dist > gate:
            break
        if gi not in gt_match and ei not in used:
            gt_match[gi] = ei
            used.add(ei)

    rows = []
    for gi, (name, expected, xy) in enumerate(gt):
        gt_range = float(np.linalg.norm(xy))
        bearing = math.degrees(math.atan2(xy[1], xy[0]))
        row = {
            'object': name, 'expected_class': expected,
            'gt_x': round(float(xy[0]), 3), 'gt_y': round(float(xy[1]), 3),
            'gt_range': round(gt_range, 3),
            'in_camera_fov': int(abs(bearing) <= CAMERA_HALF_FOV_DEG),
            'detected': 0, 'est_x': '', 'est_y': '', 'est_class': '', 'sensors': '',
            'pos_error': '', 'range_error': '',
        }
        if gi in gt_match:
            cls, sensors, est = estimates[gt_match[gi]]
            row.update({
                'detected': 1,
                'est_x': round(float(est[0]), 3), 'est_y': round(float(est[1]), 3),
                'est_class': cls, 'sensors': sensors,
                'pos_error': round(float(np.linalg.norm(est - xy)), 4),
                'range_error': round(float(np.linalg.norm(est)) - gt_range, 4),
            })
        rows.append(row)
    return rows


def summarize(rows: list) -> str:
    if not rows:
        return 'No results yet.'
    viewpoints = len({r['viewpoint'] for r in rows})
    scans = len({(r['viewpoint'], r['scan']) for r in rows})
    det = [r for r in rows if int(r['detected'])]
    pos = [float(r['pos_error']) for r in det]
    rng = [abs(float(r['range_error'])) for r in det]
    lines = [
        f'Viewpoints: {viewpoints}   scans: {scans}   object instances: {len(rows)}',
        f'LiDAR detection rate:       {100.0 * len(det) / len(rows):.1f} %',
    ]
    if pos:
        p95 = float(np.percentile(pos, 95))
        lines.append(
            f'Horizontal position error:  mean {mean(pos):.3f} m | median {median(pos):.3f} m'
            f' | p95 {p95:.3f} m')
        lines.append(f'Absolute range error:       mean {mean(rng):.3f} m | median {median(rng):.3f} m')
        signed = [float(r['range_error']) for r in det]
        lines.append(f'Mean signed range error:    {mean(signed):+.3f} m (negative = estimate nearer than true centre)')

    lines.append('Per object:')
    for name in sorted({r['object'] for r in rows}):
        obj = [r for r in det if r['object'] == name]
        total = sum(r['object'] == name for r in rows)
        err = f'mean error {mean(float(r["pos_error"]) for r in obj):.3f} m' if obj else 'not detected'
        lines.append(f'  {name:<12} detected {len(obj)}/{total}  {err}')

    labelled = [r for r in rows if r['expected_class'] and int(r['in_camera_fov'])]
    for cls in sorted({r['expected_class'] for r in labelled}):
        subset = [r for r in labelled if r['expected_class'] == cls]
        correct = [r for r in subset if r['est_class'] == cls]
        lines.append(
            f'"{cls}" labelled correctly in camera view: {len(correct)}/{len(subset)}'
            f' = {100.0 * len(correct) / len(subset):.1f} %')
    in_fov = [r for r in det if int(r['in_camera_fov'])]
    if in_fov:
        both = sum(r['sensors'] == 'lidar+camera' for r in in_fov)
        lines.append(
            f'In camera view, LiDAR and camera agreed on {100.0 * both / len(in_fov):.1f} % '
            f'of detected objects')
    return '\n'.join(lines)


def load_rows(path: str) -> list:
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------- ROS part

def record(args, labels: dict) -> list:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('go2_evaluator', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    scans = []
    node.create_subscription(
        String, '/fusion/alerts', lambda m: scans.append(json.loads(m.data)), 10)

    # Static LiDAR mount on the robot body.
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and not tf_buffer.can_transform(args.sensor_frame, args.base_frame,
                                                     Time()):
        if time.monotonic() > deadline:
            sys.exit(f'No TF {args.base_frame} -> {args.sensor_frame}')
        rclpy.spin_once(node, timeout_sec=0.1)
    tf = tf_buffer.lookup_transform(args.sensor_frame, args.base_frame, Time())
    r, t = tf.transform.rotation, tf.transform.translation
    sensor_R = quat_to_matrix(np.array([r.x, r.y, r.z, r.w]))
    sensor_t = np.array([t.x, t.y, t.z])

    before = read_ground_truth()
    robot_pos, robot_quat = before[ROBOT_MODEL]
    gt = []
    for name in args.objects.split(','):
        if name not in before:
            print(f'  warning: "{name}" not found in Gazebo, skipped')
            continue
        p = world_to_sensor(before[name][0], robot_pos, robot_quat, sensor_R, sensor_t)
        if 0.5 <= np.linalg.norm(p[:2]) <= args.max_range:
            gt.append((name, labels.get(name, ''), p[:2]))

    # A viewpoint is only new if the robot or some object moved since an earlier run.
    previous = {}
    for r in load_rows(args.csv):
        previous.setdefault(r['viewpoint'], {})[r['object']] = (float(r['gt_x']), float(r['gt_y']))
    for vp, objects in previous.items():
        same = [n for n, _, xy in gt
                if n in objects and np.linalg.norm(np.array(objects[n]) - xy) < 0.2]
        if gt and len(same) == len(gt):
            sys.exit(f'Same scene as viewpoint {vp} already in {args.csv}. '
                     'Move the robot or an object first, then run again.')

    scans.clear()
    print(f'Recording {args.scans} fused scans from /fusion/alerts ... keep the robot still')
    while rclpy.ok() and len(scans) < args.scans:
        rclpy.spin_once(node, timeout_sec=0.5)
    after = read_ground_truth()
    node.destroy_node()
    rclpy.shutdown()

    # The robot must not have moved, otherwise the ground truth doesn't match the scans.
    moved = np.linalg.norm(after[ROBOT_MODEL][0][:2] - before[ROBOT_MODEL][0][:2])
    turned = abs(math.degrees(yaw_of(after[ROBOT_MODEL][1]) - yaw_of(before[ROBOT_MODEL][1])))
    if moved > 0.05 or turned > 2.0:
        sys.exit(f'Robot moved {moved:.2f} m / {turned:.1f} deg during recording; discarded. '
                 'Stop teleop (k) and try again.')

    viewpoint = int(time.time())
    rows = []
    for i, scan in enumerate(scans):
        if scan.get('frame') != args.sensor_frame:
            sys.exit(f"Fusion output is in frame {scan.get('frame')}, expected {args.sensor_frame}")
        estimates = [
            (o['class'], o['sensors'], np.array([o['position']['x'], o['position']['y']]))
            for o in scan['objects'] if o['position'] is not None  # camera-only: no position
        ]
        for row in score_scan(gt, estimates, args.gate):
            row.update({'viewpoint': viewpoint, 'scan': i})
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--objects', default=DEFAULT_OBJECTS,
                        help='comma-separated Gazebo model names to evaluate')
    parser.add_argument('--label', action='append', default=[],
                        help='expected class for a model, e.g. person1=person (repeatable)')
    parser.add_argument('--scans', type=int, default=20)
    parser.add_argument('--gate', type=float, default=1.0,
                        help='max distance (m) to associate an estimate with an object')
    parser.add_argument('--max-range', type=float, default=8.0)
    parser.add_argument('--sensor-frame', default='velodyne')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--csv', default='results/eval.csv')
    parser.add_argument('--summary-only', action='store_true')
    args = parser.parse_args()
    labels = dict(item.split('=', 1) for item in args.label)

    if not args.summary_only:
        rows = record(args, labels)
        os.makedirs(os.path.dirname(args.csv) or '.', exist_ok=True)
        new_file = not os.path.exists(args.csv)
        with open(args.csv, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerows(rows)
        print(f'Appended {len(rows)} rows to {args.csv}\n')

    print(summarize(load_rows(args.csv)))


if __name__ == '__main__':
    main()
