#!/usr/bin/env python3
"""
aruco_detector_node.py
──────────────────────
Detects DICT_4X4_50 ArUco markers on both robot cameras and publishes:

  /aruco/front/debug   (sensor_msgs/Image)  annotated front image
  /aruco/rear/debug    (sensor_msgs/Image)  annotated rear  image

TF frames broadcast for every detected marker (parent = camera link):
  aruco_marker_<id>_front
  aruco_marker_<id>_rear

Parameters (set via ros2 param or launch):
  marker_size  (float, default 0.15)   physical side-length in metres
  dict_id      (int,   default 0)      0 = DICT_4X4_50
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


# ── rotation matrix → quaternion (x, y, z, w) ────────────────────────────────
def _rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


# ── per-camera handler ────────────────────────────────────────────────────────
class _CameraHandler:
    """Manages subscriptions, detection and publishing for one camera."""

    def __init__(self, node, name, image_topic, info_topic, camera_frame):
        self.node = node
        self.name = name
        self.camera_frame = camera_frame
        self.bridge = CvBridge()

        # Intrinsics — filled once CameraInfo arrives
        self.K = None
        self.D = None

        # QoS matching the Gazebo ros_gz_bridge (Best Effort, volatile)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        node.create_subscription(CameraInfo, info_topic,  self._info_cb,  qos)
        node.create_subscription(Image,      image_topic, self._image_cb, qos)

        self.debug_pub = node.create_publisher(Image, f'/aruco/{name}/debug', 5)

        node.get_logger().info(
            f'[aruco/{name}] listening on {image_topic}  |  {info_topic}')

    # ── camera info ──────────────────────────────────────────────────────────
    def _info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.node.get_logger().info(
                f'[aruco/{self.name}] intrinsics received  '
                f'fx={self.K[0,0]:.1f}  fy={self.K[1,1]:.1f}  '
                f'cx={self.K[0,2]:.1f}  cy={self.K[1,2]:.1f}')

    # ── image callback ────────────────────────────────────────────────────────
    def _image_cb(self, msg):
        if self.K is None:
            return   # wait for intrinsics first

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.node.get_logger().warn(f'[aruco/{self.name}] bridge: {exc}')
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # NOTE: do NOT pass a DetectorParameters object here. This OpenCV
        # 4.6.0 build segfaults when `parameters` is provided (verified:
        # both keyword and positional forms crash). Omitting it uses the
        # default detector parameters, which work correctly.
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.node.aruco_dict)

        annotated = bgr.copy()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.node.marker_size, self.K, self.D)

            for i, mid in enumerate(ids.flatten()):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                # draw 3-D axis on annotated image
                cv2.drawFrameAxes(annotated, self.K, self.D,
                                  rvec, tvec, self.node.marker_size * 0.5)

                # convert rodrigues → quaternion
                R, _ = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = _rot_to_quat(R)

                # broadcast TF: camera_frame → aruco_marker_<id>_<cam>
                ts = TransformStamped()
                ts.header.stamp    = msg.header.stamp
                ts.header.frame_id = self.camera_frame
                ts.child_frame_id  = f'aruco_marker_{mid}_{self.name}'
                ts.transform.translation.x = float(tvec[0])
                ts.transform.translation.y = float(tvec[1])
                ts.transform.translation.z = float(tvec[2])
                ts.transform.rotation.x = qx
                ts.transform.rotation.y = qy
                ts.transform.rotation.z = qz
                ts.transform.rotation.w = qw
                self.node.tf_broadcaster.sendTransform(ts)

                dist = float(np.linalg.norm(tvec))
                self.node.get_logger().info(
                    f'[aruco/{self.name}] ID={mid}  '
                    f'x={tvec[0]:+.3f}  y={tvec[1]:+.3f}  z={tvec[2]:+.3f}  '
                    f'dist={dist:.3f} m',
                    throttle_duration_sec=1.0)

        # always publish debug image (blank axes overlay if nothing detected)
        try:
            out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)
        except Exception as exc:
            self.node.get_logger().warn(f'[aruco/{self.name}] pub: {exc}')


# ── main node ─────────────────────────────────────────────────────────────────
class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('dict_id',     0)

        self.marker_size = self.get_parameter('marker_size').value
        dict_id          = self.get_parameter('dict_id').value

        # ── ArUco dictionary ─────────────────────────────────────────────────
        # (no custom DetectorParameters: passing one segfaults OpenCV 4.6.0)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        self.get_logger().info(
            f'ArUco detector node ready  '
            f'dict_id={dict_id} (DICT_4X4_50)  '
            f'marker_size={self.marker_size} m')

        # ── TF broadcaster ────────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── one handler per camera ────────────────────────────────────────────
        self._front = _CameraHandler(
            self,
            name='front',
            image_topic='/camera/front/image_raw',
            info_topic='/camera/front/camera_info',
            camera_frame='camera_link_1',
        )
        self._rear = _CameraHandler(
            self,
            name='rear',
            image_topic='/camera/rear/image_raw',
            info_topic='/camera/rear/camera_info',
            camera_frame='camera_link_2',
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
