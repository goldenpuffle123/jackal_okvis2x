#!/usr/bin/env python3
"""Broadcast OKVIS2-X's live pose as a tf2 transform (REP 105 re-parenting).

OKVIS publishes its pose only on topics, never on tf. This node subscribes to
one of them and broadcasts `global_frame -> attach_frame`:

    map  (global_frame)
     └── femtobolt_link  (attach_frame)         <── broadcast by this node
          ├── femtobolt_color_optical_frame      ╮
          ├── ...                                │ camera driver's static tree
          └── femtobolt_accel_gyro_optical_frame ╯  = OKVIS "body" (body_frame)

OKVIS estimates map -> body (its IMU frame), but body already has a parent in
the camera driver's static tree and tf2 allows only one parent per frame. The
`attach_frame` is where the pose gets grafted on instead: the root of that
static tree. The fixed body -> attach offset is looked up from tf once and
folded into every pose: T_map_attach = T_map_body * T_body_attach.

Pose sources (pose_topic / pose_type):
- okvis_odometry (odometry, default): corrected IMU-propagated pose, 40 Hz,
  near-real-time. Best for live tf/RViz.
- okvis_pose (pose, needs tools/patches/Publisher.cpp.patch): optimised pose
  per state, stamped with the image timestamp. Frame rate + optimiser latency,
  so live consumers can't bracket fresh sensor data — offline/playback only.
- okvis_transform (transform): starved by its shared depth-1 publisher handle;
  in practice only body_<id> keyframe corrections reach the wire. Avoid.

Usage:
    pixi run -e okvis2x tf-bridge
    pixi run -e okvis2x tf-bridge --ros-args \
        -p pose_topic:=/okvis/okvis_pose -p pose_type:=pose
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def matrix_to_quat(R):
    """Rotation matrix to (x, y, z, w) quaternion."""
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        return np.array(
            [(R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s]
        )
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(max(1.0 + R[i, i] - R[j, j] - R[k, k], 1e-12))
    q = np.empty(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q

def quat_multiply(q1, q2):
    """Hamilton product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_rotate(q, v):
    """Rotate vector v by (x, y, z, w) quaternion q."""
    u = q[:3]
    w = q[3]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


class OkvisTfBridge(Node):
    def __init__(self):
        super().__init__("okvis_tf_bridge")

        self.declare_parameter("pose_topic", "/okvis/okvis_pose")
        self.declare_parameter("pose_type", "pose")  # "odometry", "transform" or "pose"
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("body_frame", "femtobolt_accel_gyro_optical_frame")
        self.declare_parameter("attach_frame", "femtobolt_link")
        self.declare_parameter("path_max_len", 20000)

        self.global_frame: str = self.get_parameter("global_frame").value # type: ignore
        self.body_frame: str = self.get_parameter("body_frame").value # type: ignore
        self.attach_frame: str = self.get_parameter("attach_frame").value # type: ignore
        pose_topic: str = self.get_parameter("pose_topic").value # type: ignore
        pose_type: str = self.get_parameter("pose_type").value # type: ignore
        self.path_max_len: int = self.get_parameter("path_max_len").value  # type: ignore
        self.path_min_dist: float = 0.02

        self.est_path = Path()
        self.est_path_pub = self.create_publisher(Path, "/okvis/est_path", 2)

        # body -> attach is static; resolved once on the first pose, then cached.
        # A camera on an actuated mount would need this looked up per message
        # at the pose stamp instead.
        self.offset = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        if pose_type == "odometry":
            self.create_subscription(Odometry, pose_topic, self._on_odometry, 10)
        elif pose_type == "transform":
            self.create_subscription(
                TransformStamped, pose_topic, self._on_transform, 10
            )
        elif pose_type == "pose":
            self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        else:
            raise ValueError(
                f"pose_type must be 'odometry', 'transform' or 'pose', "
                f"got {pose_type!r}"
            )

        self.get_logger().info(
            f"Bridging {pose_topic} ({pose_type}) to tf as "
            f"{self.global_frame} -> {self.attach_frame} (via {self.body_frame})"
        )

    def _resolve_offset(self):
        """Fixed transform from OKVIS's body frame out to the femtobolt tf root."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.body_frame, self.attach_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warning(
                f"Waiting for static tf {self.body_frame} -> {self.attach_frame}: {e}",
                throttle_duration_sec=5.0,
            )
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        self.get_logger().info(
            f"Resolved static offset {self.body_frame} -> {self.attach_frame}: "
            f"translation=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
        )
        return np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w])

    def _on_odometry(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._broadcast(msg.header.stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self._broadcast(msg.header.stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _on_transform(self, msg: TransformStamped):
        # OKVIS also emits world -> body_<id> keyframe corrections on this topic;
        # only the plain "body" child is the current live pose.
        if msg.child_frame_id != "body":
            return
        t = msg.transform.translation
        r = msg.transform.rotation
        self._broadcast(msg.header.stamp, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _broadcast(self, stamp, position, orientation):
        if self.offset is None:
            self.offset = self._resolve_offset()
            if self.offset is None:
                return
        offset_translation, offset_rotation = self.offset

        map_translation = np.array(position)
        map_rotation = np.array(orientation)

        translation = map_translation + quat_rotate(map_rotation, offset_translation)
        rotation = quat_multiply(map_rotation, offset_rotation)

        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.global_frame
        out.child_frame_id = self.attach_frame
        out.transform.translation.x = float(translation[0])
        out.transform.translation.y = float(translation[1])
        out.transform.translation.z = float(translation[2])
        out.transform.rotation.x = float(rotation[0])
        out.transform.rotation.y = float(rotation[1])
        out.transform.rotation.z = float(rotation[2])
        out.transform.rotation.w = float(rotation[3])

        self._append(self.est_path, self.est_path_pub, stamp, np.vstack([np.hstack([np.eye(3), translation.reshape(3, 1)]), [0, 0, 0, 1]]))

        self.tf_broadcaster.sendTransform(out)

    def _append(self, path: Path, pub, stamp, T):
            if path.poses:
                last = path.poses[-1].pose.position
                if np.linalg.norm(T[:3, 3] - np.array([last.x, last.y, last.z])) < self.path_min_dist:
                    return
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.global_frame
            ps.pose.position.x = float(T[0, 3])
            ps.pose.position.y = float(T[1, 3])
            ps.pose.position.z = float(T[2, 3])
            quat = matrix_to_quat(T[:3, :3])
            ps.pose.orientation.x = float(quat[0])
            ps.pose.orientation.y = float(quat[1])
            ps.pose.orientation.z = float(quat[2])
            ps.pose.orientation.w = float(quat[3])
            path.poses.append(ps)
            if len(path.poses) > self.path_max_len:
                del path.poses[: len(path.poses) - self.path_max_len]
            path.header.stamp = stamp
            path.header.frame_id = self.global_frame
            pub.publish(path)


def main():
    rclpy.init()
    node = OkvisTfBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
