#!/usr/bin/env python3
"""Patch a rosbag2 recording of femtobolt + live OKVIS2-X output into a bag
perceive_semantix_release can consume out of the box.

perceive_semantix_ros2 looks up a tf2 transform from `global_frame` (default
"map") to the color frame every frame -- it never reads a pose topic directly.
OKVIS2-X only publishes plain `okvis_odometry`/`okvis_transform` topics, never
a tf broadcast. This script converts whichever is present into real
`tf2_msgs/msg/TFMessage` /tf entries.

OKVIS's "body" pose corresponds to the femtobolt IMU frame
(femtobolt_accel_gyro_optical_frame), but that frame already has a parent in
the femtobolt tf tree, and tf2 forbids a second parent. So instead this script
computes the fixed offset from the tf tree's root (auto-detected) out to the
IMU frame, folds it into every pose, and publishes `map -> <root frame>`,
grafting cleanly onto the existing femtobolt static chain.

Everything else in the bag passes through unchanged.

Usage:
    pixi run -e ros2 python3 tools/patch_okvis_bag.py <input_bag_dir> <output_bag_dir>
"""

import argparse
import math
from collections import deque
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader, Writer
from rosbags.rosbag2.writer import StoragePlugin
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_JAZZY)
TF_MSGTYPE = "tf2_msgs/msg/TFMessage"


# --- minimal homogeneous-transform helpers (avoid a scipy dependency for this one tool) ---


def quat_to_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return x, y, z, w


def transform_to_matrix(translation, rotation):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(*rotation)
    T[:3, 3] = translation
    return T


def matrix_to_transform(T):
    return tuple(T[:3, 3]), matrix_to_quat(T[:3, :3])


def invert(T):
    R, t = T[:3, :3], T[:3, 3]
    Tinv = np.eye(4)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


# --- tf message construction ---


def make_tf_message(stamp_sec_nsec, parent_frame, child_frame, translation, rotation):
    Header = TS.types["std_msgs/msg/Header"]
    Time = TS.types["builtin_interfaces/msg/Time"]
    Vector3 = TS.types["geometry_msgs/msg/Vector3"]
    Quaternion = TS.types["geometry_msgs/msg/Quaternion"]
    Transform = TS.types["geometry_msgs/msg/Transform"]
    TransformStamped = TS.types["geometry_msgs/msg/TransformStamped"]
    TFMessage = TS.types["tf2_msgs/msg/TFMessage"]

    sec, nanosec = stamp_sec_nsec
    header = Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=parent_frame)
    tfs = TransformStamped(
        header=header,
        child_frame_id=child_frame,
        transform=Transform(
            translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
            rotation=Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]),
        ),
    )
    return TFMessage(transforms=[tfs])


def discover_attach_offset(bag_path, tf_topic, attach_frame, imu_frame, scan_limit):
    """Scan the start of the bag's tf stream to find the fixed transform from
    attach_frame (auto-detected tree root if None) out to imu_frame, by composing
    the chain of individually-published static edges. Returns (attach_frame, T_attach_imu)."""
    edges = {}
    parents, children = set(), set()
    tf_connections = None
    with Reader(bag_path) as reader:
        tf_connections = [c for c in reader.connections if c.topic == tf_topic]
        n = 0
        for connection, _timestamp, rawdata in reader.messages(connections=tf_connections):
            msg = TS.deserialize_cdr(rawdata, connection.msgtype)
            for tr in msg.transforms:
                p, c = tr.header.frame_id, tr.child_frame_id
                parents.add(p)
                children.add(c)
                if (p, c) not in edges:
                    t = tr.transform.translation
                    r = tr.transform.rotation
                    edges[(p, c)] = transform_to_matrix((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
            n += 1
            if n >= scan_limit:
                break

    if attach_frame is None:
        roots = parents - children
        if len(roots) != 1:
            raise SystemExit(
                f"Could not auto-detect a unique tf tree root in the first {scan_limit} /tf messages "
                f"(candidates: {sorted(roots)}); pass --attach-frame explicitly."
            )
        attach_frame = next(iter(roots))

    adjacency = {}
    for (p, c), m in edges.items():
        adjacency.setdefault(p, []).append((c, m))

    queue = deque([(attach_frame, np.eye(4))])
    visited = {attach_frame}
    path_matrix = None
    while queue:
        frame, mat = queue.popleft()
        if frame == imu_frame:
            path_matrix = mat
            break
        for child, edge_mat in adjacency.get(frame, []):
            if child not in visited:
                visited.add(child)
                queue.append((child, mat @ edge_mat))

    if path_matrix is None:
        raise SystemExit(
            f"No static tf path found from '{attach_frame}' to '{imu_frame}' in the first {scan_limit} "
            "/tf messages -- pass --scan-limit higher, or check --attach-frame/--imu-frame."
        )
    return attach_frame, path_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="Input rosbag2 directory")
    parser.add_argument("output", type=Path, help="Output rosbag2 directory to create (must not exist)")
    parser.add_argument("--odometry-topic", default="/okvis/okvis_odometry")
    parser.add_argument("--transform-topic", default="/okvis/okvis_transform")
    parser.add_argument(
        "--pose-source",
        choices=["auto", "odometry", "transform"],
        default="auto",
        help="'auto' prefers okvis_odometry if it has messages, else falls back to okvis_transform "
        "(odometry is disabled upstream by default -- see imu_propagated_state_publishing_rate).",
    )
    parser.add_argument("--map-frame", default="map", help="frame_id to publish as the /tf parent (matches perceive_semantix's default global_frame)")
    parser.add_argument(
        "--imu-frame",
        default="femtobolt_accel_gyro_optical_frame",
        help="frame_id OKVIS's published body pose physically corresponds to (must match the frame_id stamped "
        "on the raw IMU topic OKVIS was calibrated against -- used only to compute a fixed offset, never "
        "published as a tf parent itself, since it already has one from the femtobolt driver).",
    )
    parser.add_argument(
        "--attach-frame",
        default=None,
        help="Root frame of the existing femtobolt tf tree to publish map->this onto (default: auto-detect "
        "the one frame in the bag's /tf stream that never appears as a child).",
    )
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--storage", choices=["mcap", "sqlite3"], default="mcap")
    parser.add_argument(
        "--metadata-version",
        type=int,
        choices=[8, 9],
        default=8,
        help="rosbag2 metadata.yaml schema version. 9 (Jazzy-native) writes offered_qos_profiles as a nested "
        "YAML list; ROS 2 Humble's yaml-cpp reader expects it as a plain string and fails with "
        "'bad conversion' on a v9 file. perceive_semantix_release's pixi env runs Humble (ros-humble-ros2bag "
        "via RoboStack) -- default to 8 so bags play there. Use 9 only if playback happens under Jazzy.",
    )
    parser.add_argument("--drop-original-pose-topic", action="store_true", help="Don't pass the raw okvis_odometry/okvis_transform messages through to the output bag")
    parser.add_argument("--scan-limit", type=int, default=1000, help="Max /tf messages to scan when discovering the static attach-frame offset")
    args = parser.parse_args()

    storage_plugin = StoragePlugin.MCAP if args.storage == "mcap" else StoragePlugin.SQLITE3

    with Reader(args.input) as reader:
        msgcounts = {c.topic: c.msgcount for c in reader.connections}
        has_odometry = msgcounts.get(args.odometry_topic, 0) > 0
        has_transform = msgcounts.get(args.transform_topic, 0) > 0

        if args.pose_source == "auto":
            if has_odometry:
                pose_source = "odometry"
            elif has_transform:
                pose_source = "transform"
            else:
                raise SystemExit(
                    f"Neither {args.odometry_topic} ({msgcounts.get(args.odometry_topic, 0)} msgs) nor "
                    f"{args.transform_topic} ({msgcounts.get(args.transform_topic, 0)} msgs) has any messages "
                    "in this bag -- was OKVIS2-X actually running and connected during the recording? "
                    "(okvis_odometry needs the node param imu_propagated_state_publishing_rate > 0 to publish "
                    "at all; it's commented out / 0.0 by default in okvis2x_node_subscriber_orbbec.launch.xml.)"
                )
        else:
            pose_source = args.pose_source
        pose_topic = args.odometry_topic if pose_source == "odometry" else args.transform_topic
        print(f"Using pose source: {pose_topic} ({msgcounts.get(pose_topic, 0)} msgs)")

        if args.tf_topic not in msgcounts:
            raise SystemExit(f"{args.tf_topic} not found in input bag -- expected the femtobolt driver's tf stream to already be there.")
        if args.output.exists():
            raise SystemExit(f"Output path {args.output} already exists -- refusing to overwrite.")

    attach_frame, T_attach_imu = discover_attach_offset(args.input, args.tf_topic, args.attach_frame, args.imu_frame, args.scan_limit)
    T_imu_attach = invert(T_attach_imu)
    print(f"Static offset found: '{attach_frame}' -> '{args.imu_frame}', translation={T_attach_imu[:3, 3]}")
    print(f"Publishing pose as: {args.map_frame} -> {attach_frame}")

    with Reader(args.input) as reader:
        with Writer(args.output, version=args.metadata_version, storage_plugin=storage_plugin) as writer:
            out_connections = {}
            for c in reader.connections:
                out_connections[c.topic] = writer.add_connection(
                    c.topic,
                    c.msgtype,
                    typestore=TS,
                    offered_qos_profiles=c.ext.offered_qos_profiles if c.ext else (),
                )

            n_pose = 0
            n_pose_skipped = 0
            n_passthrough = 0
            for connection, timestamp, rawdata in reader.messages():
                out_conn = out_connections[connection.topic]

                if connection.topic == pose_topic:
                    msg = TS.deserialize_cdr(rawdata, connection.msgtype)
                    if pose_source == "odometry":
                        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                        pos = msg.pose.pose.position
                        ori = msg.pose.pose.orientation
                        translation = (pos.x, pos.y, pos.z)
                        rotation = (ori.x, ori.y, ori.z, ori.w)
                    else:  # transform
                        if msg.child_frame_id != "body":
                            n_pose_skipped += 1
                            if not args.drop_original_pose_topic:
                                writer.write(out_conn, timestamp, rawdata)
                            continue  # drop world -> body_<id> keyframe-correction spam
                        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                        t = msg.transform.translation
                        r = msg.transform.rotation
                        translation = (t.x, t.y, t.z)
                        rotation = (r.x, r.y, r.z, r.w)

                    T_map_imu = transform_to_matrix(translation, rotation)
                    T_map_attach = T_map_imu @ T_imu_attach
                    attach_translation, attach_rotation = matrix_to_transform(T_map_attach)

                    tf_msg = make_tf_message(stamp, args.map_frame, attach_frame, attach_translation, attach_rotation)
                    data = TS.serialize_cdr(tf_msg, TF_MSGTYPE)
                    writer.write(out_connections[args.tf_topic], timestamp, data)
                    n_pose += 1
                    if not args.drop_original_pose_topic:
                        writer.write(out_conn, timestamp, rawdata)
                    continue

                writer.write(out_conn, timestamp, rawdata)
                n_passthrough += 1

    print(
        f"Wrote {n_pose} synthesized /tf transforms ({args.map_frame} -> {attach_frame}), "
        f"skipped {n_pose_skipped} keyframe-correction messages, passed through {n_passthrough} other messages."
    )


if __name__ == "__main__":
    main()
