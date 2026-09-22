#!/usr/bin/env python3
import os
import math
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

from pupil_apriltags import Detector

from .pose_filter import TemporalPoseFilter


def rpy_to_rot(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def make_T(xyz, rpy_rad):
    T = np.eye(4, dtype=float)
    T[:3, :3] = rpy_to_rot(*rpy_rad)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def correct_pnp_tag_rotation(rotation):
    """Convert pupil/OpenCV's planar-tag axes to the physical dock tag frame.

    With the generated tag textures and IPPE corner order, PnP returns tag X
    and Z opposite to the model's X-forward, Y-left, Z-up tag frame. A 180
    degree rotation about tag Y makes the transform physical and right-handed.
    """
    return rotation @ rpy_to_rot(0.0, math.pi, 0.0)


def rot_to_quat(R):
    # Quaternion [x, y, z, w]
    q = np.empty(4, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q[3] = (R[2, 1] - R[1, 2]) / s
        q[0] = 0.25 * s
        q[1] = (R[0, 1] + R[1, 0]) / s
        q[2] = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q[3] = (R[0, 2] - R[2, 0]) / s
        q[0] = (R[0, 1] + R[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q[3] = (R[1, 0] - R[0, 1]) / s
        q[0] = (R[0, 2] + R[2, 0]) / s
        q[1] = (R[1, 2] + R[2, 1]) / s
        q[2] = 0.25 * s
    q /= np.linalg.norm(q)
    return q


def average_rotations(rotations, weights):
    M = np.zeros((3, 3), dtype=float)
    for R, w in zip(rotations, weights):
        M += w * R
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


class DockLocalizer(Node):
    def __init__(self):
        super().__init__('dock_apriltag_localizer')

        self.declare_parameter('config_file', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dock_frame', 'dock_target')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/dock/debug_image')
        self.declare_parameter('pose_filter_enabled', True)
        self.declare_parameter('pose_filter_time_constant_s', 0.12)
        self.declare_parameter(
            'pose_filter_max_translation_speed_m_s', 2.0
        )
        self.declare_parameter('pose_filter_max_position_jump_m', 0.20)
        self.declare_parameter('pose_filter_max_gate_dt_s', 0.20)
        self.declare_parameter('pose_filter_reset_timeout_s', 1.0)
        self.declare_parameter('pose_filter_reacquire_frames', 3)

        self.bridge = CvBridge()
        self.K = None
        self.dist = None
        self.camera_frame = None
        self.tags = {}
        self.central_ids = set()
        self.outer_ids = set()
        self.central_good_frames = 0
        self.central_missing_frames = 0
        self.central_active = False

        self.config_file = self.get_parameter('config_file').value
        if not self.config_file:
            raise RuntimeError('Parameter config_file is required.')
        self._load_config()

        self.pose_filter = TemporalPoseFilter(
            enabled=bool(
                self.get_parameter('pose_filter_enabled').value
            ),
            time_constant_s=float(
                self.get_parameter('pose_filter_time_constant_s').value
            ),
            max_translation_speed_m_s=float(
                self.get_parameter(
                    'pose_filter_max_translation_speed_m_s'
                ).value
            ),
            max_position_jump_m=float(
                self.get_parameter(
                    'pose_filter_max_position_jump_m'
                ).value
            ),
            max_gate_dt_s=float(
                self.get_parameter('pose_filter_max_gate_dt_s').value
            ),
            reset_timeout_s=float(
                self.get_parameter('pose_filter_reset_timeout_s').value
            ),
            reacquire_frames=int(
                self.get_parameter('pose_filter_reacquire_frames').value
            ),
        )

        det = self.detector_cfg
        self.detector = Detector(
            families=det.get('family', 'tag36h11'),
            nthreads=int(det.get('nthreads', 2)),
            quad_decimate=float(det.get('quad_decimate', 1.0)),
            quad_sigma=float(det.get('quad_sigma', 0.0)),
            refine_edges=int(det.get('refine_edges', 1)),
            decode_sharpening=float(det.get('decode_sharpening', 0.25)),
            debug=0,
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/dock/pose', 10)
        self.mode_pub = self.create_publisher(String, '/dock/mode', 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, '/dock/visible_tag_ids', 10)
        self.debug_pub = None
        if self.get_parameter('publish_debug_image').value:
            self.debug_pub = self.create_publisher(
                Image,
                self.get_parameter('debug_image_topic').value,
                2,
            )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.reload_srv = self.create_service(Trigger, '~/reload_config', self._reload_cb)

        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(f'Loaded {len(self.tags)} tags from {self.config_file}')

    def _load_config(self):
        with open(os.path.expanduser(self.config_file), 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)

        self.detector_cfg = cfg.get('detector', {})
        self.logic = cfg.get('switching', {})
        self.fusion = cfg.get('fusion', {})
        self.tags = {}
        self.central_ids = set()
        self.outer_ids = set()

        for item in cfg['tags']:
            tag_id = int(item['id'])
            role = str(item.get('role', 'outer')).lower()
            size = float(item['size_m'])
            xyz = item.get('dock_to_tag', {}).get('xyz_m', [0.0, 0.0, 0.0])
            rpy_deg = item.get('dock_to_tag', {}).get('rpy_deg', [0.0, 0.0, 0.0])
            rpy = [math.radians(v) for v in rpy_deg]
            self.tags[tag_id] = {
                'id': tag_id,
                'role': role,
                'size_m': size,
                'T_dock_tag': make_T(xyz, rpy),
            }
            (self.central_ids if role == 'central' else self.outer_ids).add(tag_id)

    def _reload_cb(self, request, response):
        try:
            self._load_config()
            self.pose_filter.reset()
            self.central_active = False
            self.central_good_frames = 0
            self.central_missing_frames = 0
            response.success = True
            response.message = f'Reloaded {len(self.tags)} tags.'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _camera_info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if msg.d else np.zeros((5, 1))
        self.camera_frame = msg.header.frame_id

    def _estimate_tag_pose(self, detection, size_m):
        s = size_m / 2.0
        # pupil_apriltags homography uses ideal corners:
        # (-1,+1), (+1,+1), (+1,-1), (-1,-1), matching IPPE_SQUARE order.
        obj = np.array([
            [-s, +s, 0.0],
            [+s, +s, 0.0],
            [+s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)
        img = np.ascontiguousarray(detection.corners, dtype=np.float64).reshape(4, 2)

        ok, rvec, tvec = cv2.solvePnP(
            obj, img, self.K, self.dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return None

        R, _ = cv2.Rodrigues(rvec)
        R = correct_pnp_tag_rotation(R)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = tvec.reshape(3)
        return T

    def _candidate_from_detection(self, d):
        tag = self.tags.get(int(d.tag_id))
        if tag is None:
            return None
        min_margin = float(self.detector_cfg.get('min_decision_margin', 20.0))
        if float(d.decision_margin) < min_margin:
            return None

        T_cam_tag = self._estimate_tag_pose(d, tag['size_m'])
        if T_cam_tag is None or T_cam_tag[2, 3] <= 0.0:
            return None

        # T_cam_dock = T_cam_tag * T_tag_dock
        T_cam_dock = T_cam_tag @ np.linalg.inv(tag['T_dock_tag'])
        distance = float(np.linalg.norm(T_cam_tag[:3, 3]))
        weight = max(float(d.decision_margin), 1.0) / max(distance * distance, 0.04)
        return {
            'id': int(d.tag_id),
            'role': tag['role'],
            'T': T_cam_dock,
            'distance': distance,
            'margin': float(d.decision_margin),
            'weight': weight,
            'corners': d.corners,
            'center': d.center,
        }

    def _fuse(self, candidates):
        if not candidates:
            return None

        positions = np.array([c['T'][:3, 3] for c in candidates])
        if len(candidates) >= 3:
            med = np.median(positions, axis=0)
            max_err = float(self.fusion.get('max_candidate_position_error_m', 0.35))
            kept = [c for c in candidates if np.linalg.norm(c['T'][:3, 3] - med) <= max_err]
            if kept:
                candidates = kept

        weights = np.array([c['weight'] for c in candidates], dtype=float)
        weights /= np.sum(weights)
        t = sum(w * c['T'][:3, 3] for w, c in zip(weights, candidates))
        R = average_rotations([c['T'][:3, :3] for c in candidates], weights)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def _select_pose(self, candidates):
        outer = [c for c in candidates if c['role'] != 'central']
        central = [c for c in candidates if c['role'] == 'central']

        switch_dist = float(self.logic.get('central_switch_distance_m', 1.8))
        stable_frames = int(self.logic.get('central_stable_frames', 4))
        dropout_frames = int(self.logic.get('central_dropout_frames', 5))

        valid_central = [c for c in central if c['distance'] <= switch_dist]
        central_pose = self._fuse(valid_central)
        outer_pose = self._fuse(outer)

        if valid_central:
            self.central_good_frames += 1
            self.central_missing_frames = 0
        else:
            self.central_good_frames = 0
            self.central_missing_frames += 1

        if not self.central_active and self.central_good_frames >= stable_frames:
            self.central_active = True
            self.get_logger().info('Switching to CENTRAL docking target.')
        elif self.central_active and self.central_missing_frames > dropout_frames:
            self.central_active = False
            self.get_logger().warning(
                'Central tag lost: falling back to OUTER tags.'
            )

        if self.central_active and central_pose is not None:
            return central_pose, 'CENTRAL'
        if outer_pose is not None:
            return outer_pose, 'OUTER'
        if central_pose is not None:
            return central_pose, 'CENTRAL_ACQUIRING'
        return None, 'NO_TARGET'

    def _publish_pose(self, T, header, mode):
        frame_id = header.frame_id or self.camera_frame or 'camera_optical_frame'
        stamp = header.stamp
        q = rot_to_quat(T[:3, :3])

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3, 3].tolist()
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        self.pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header = pose.header
        tf.child_frame_id = self.get_parameter('dock_frame').value
        tf.transform.translation.x = pose.pose.position.x
        tf.transform.translation.y = pose.pose.position.y
        tf.transform.translation.z = pose.pose.position.z
        tf.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        m = String()
        m.data = mode
        self.mode_pub.publish(m)

    def _draw_debug(self, image, candidates, mode):
        for c in candidates:
            pts = np.asarray(c['corners'], dtype=int).reshape(-1, 2)
            for i in range(4):
                cv2.line(image, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 0), 2)
            cx, cy = np.asarray(c['center'], dtype=int)
            cv2.putText(image, f"ID {c['id']} {c['role']} {c['distance']:.2f}m",
                        (int(cx) + 5, int(cy) - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(image, f'MODE: {mode}', (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 255), 2, cv2.LINE_AA)
        return image

    def _image_cb(self, msg):
        if self.K is None:
            self.get_logger().warning(
                'Waiting for CameraInfo...', throttle_duration_sec=3.0
            )
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray, estimate_tag_pose=False)

        candidates = []
        visible_ids = []
        for d in detections:
            if int(d.tag_id) not in self.tags:
                continue
            c = self._candidate_from_detection(d)
            if c is not None:
                candidates.append(c)
                visible_ids.append(c['id'])

        ids_msg = Int32MultiArray()
        ids_msg.data = visible_ids
        self.ids_pub.publish(ids_msg)

        T, mode = self._select_pose(candidates)
        if T is not None:
            stamp_s = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1.0e-9
            )
            filtered_T, filter_status = self.pose_filter.update(T, stamp_s)
            if filtered_T is not None:
                self._publish_pose(filtered_T, msg.header, mode)
            else:
                self.get_logger().warning(
                    'Rejecting dock pose: '
                    f'{filter_status}; raw position='
                    f'({T[0, 3]:.2f}, {T[1, 3]:.2f}, {T[2, 3]:.2f}) m',
                    throttle_duration_sec=1.0,
                )
                m = String()
                m.data = mode
                self.mode_pub.publish(m)
        else:
            m = String()
            m.data = mode
            self.mode_pub.publish(m)

        if self.debug_pub is not None:
            dbg = self._draw_debug(bgr, candidates, mode)
            out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DockLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
