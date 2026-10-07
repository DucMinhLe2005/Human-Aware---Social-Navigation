#!/usr/bin/env python3
"""Multi-person 3D pose estimation (YOLO pose + depth camera), optimised for CPU-only computers.

- The model is ONNX (weights/yolo26n-pose.onnx, exported at 256x320).
- detection_period_sec skips inference until a minimum time has passed since the
  previous one, reducing CPU load without lowering the camera rate.
- inference_resize resizes the RGB image before inference. Keypoints are then in
  the resized image coordinates and must be scaled back (orig / resize) BEFORE
  using fx, fy, cx, cy from camera_info to back-project them to 3D.
- A cached transform tolerates short TF gaps (camera_transform_max_stale_seconds).
- Publishes MultiplePoseResult3D plus debug markers.
"""
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from thesis_msgs.msg import PoseKeypoint3D, PoseResult3D, MultiplePoseResult3D
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import cv2
from typing import Optional
from ament_index_python.packages import get_package_share_directory
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import math
import os


class PoseEstimationMultiple3DNode1(Node):
    """Multi-person detector (YOLO pose + depth), optimised for weak CPUs."""

    def __init__(self) -> None:
        super().__init__('yolo_pose_node1')

        self.declare_parameter("model_path", "")
        self.declare_parameter("detection_period_sec", 0.15)
        self.declare_parameter("inference_resize", [320, 240])  # [width, height], [] = no resize
        # Debug image with the drawn skeleton: OFF by default on the robot. Drawing and
        # publishing a 640x480x3 image every frame takes CPU from the Nav2 control loop.
        # Enable when needed: -p publish_debug_image:=true
        # Patch radius (pixels) for the median depth around each keypoint. 0 = single pixel.
        # Minimum confidence of a PERSON detection. 0 = accept all. Kept at 0: a 0.5
        # threshold removed 40% of correct detections while the wrong ones (which have
        # HIGH confidence) were barely affected.
        self.declare_parameter("person_conf_min", 0.0)
# Diagnostics: log the depth of each keypoint relative to the depth at the box
# centre. Logging only, no behaviour change. Off by default.
        self.declare_parameter("debug_depth_log", False)
        self.declare_parameter("depth_patch_radius", 2)
        # Maximum depth deviation (m) between a keypoint and the person's median depth.
        # 0 = gate disabled.
        self.declare_parameter("depth_consistency_m", 0.5)
        self.declare_parameter("publish_debug_image", False)
        # Number of ONNX Runtime threads per inference.
        #
        # The node only infers ~3.3 times per second and one inference takes ~10-15 ms,
        # so limiting threads costs no throughput. Without a limit ONNX Runtime creates
        # one thread per CPU core; on a many-core machine this took >1000% CPU and
        # starved the Nav2 container until goals were aborted.
        #
        # 0 = let onnxruntime decide (not recommended).
        self.declare_parameter("onnx_num_threads", 2)

        model_path_param = self.get_parameter("model_path").value
        self.detection_period_sec = float(self.get_parameter("detection_period_sec").value)
        resize_param = list(self.get_parameter("inference_resize").value)
        self.inference_resize = tuple(int(v) for v in resize_param) if len(resize_param) == 2 else None

        if model_path_param:
            model_path = model_path_param
        else:
            # Default: the model used by the launch file.
            model_path = os.path.join(
                get_package_share_directory('social_nav_perception'),
                'weights', 'yolo26n-pose.onnx',
            )

        self._limit_onnx_threads(
            int(self.get_parameter("onnx_num_threads").value)
        )

        self.get_logger().info(f"Loading pose model from: {model_path}")
        self.model = YOLO(model_path)
        self._last_infer_ns = 0

        self.camera_info: Optional[CameraInfo] = None
        self.camera_info_received = False
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        # The TransformListener is attached to this node, and lookups never wait (see
        # image_callback). The buffer is filled by this node's executor between
        # callbacks; /tf arrives at ~50 Hz and images at ~30 Hz, so almost every lookup
        # finds data, and the stale-transform cache
        # (camera_transform_max_stale_seconds) covers the rest. A listener with its
        # own spin thread proved unreliable: in some runs its buffer stayed empty for
        # the whole session and no person was ever published.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._last_good_camera_transform = None
        self._last_good_camera_transform_ns = 0
        self.declare_parameter("camera_transform_max_stale_seconds", 0.15)
        self.camera_transform_max_stale_seconds = float(
            self.get_parameter("camera_transform_max_stale_seconds").value
        )

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        rgb_sub = Subscriber(
            self,
            Image,
            '/camera/color/image_raw',
            qos_profile=image_qos,
        )
        depth_sub = Subscriber(
            self,
            Image,
            '/camera/depth/image_rect_raw',
            qos_profile=image_qos,
        )

        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=2,
            slop=0.03,
        )
        self.sync.registerCallback(self.image_callback)

        self.multiple_pose_publisher = self.create_publisher(
            MultiplePoseResult3D,
            '/perception/yolov8_multiple_pose_results_3d_ori',
            1,
        )
        self.image_publisher = self.create_publisher(
            Image,
            '/perception/image/pose_estimation_result_multiple_3d_ori',
            1,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/perception/skeleton_markers_multiple_3d_ori',
            1,
        )
        self.orientation_publisher = self.create_publisher(
            MarkerArray,
            '/perception/orientation_markers_multiple_3d',
            1,
        )
        self.person_conf_min = float(
            self.get_parameter("person_conf_min").value
        )
        self.debug_depth_log = bool(
            self.get_parameter("debug_depth_log").value
        )
        self.depth_patch_radius = int(
            self.get_parameter("depth_patch_radius").value
        )
        self.depth_consistency_m = float(
            self.get_parameter("depth_consistency_m").value
        )
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.bridge = CvBridge()
        # Encoding of the last depth image -- only used to log once when it changes,
        # see _depth_image_to_meters().
        self._depth_encoding_seen = None

        self.skeleton_connections = [
            (0, 1), (0, 2), (1, 3), (2, 4),
            (5, 6), (5, 11), (6, 12), (11, 12),
            (5, 7), (7, 9),
            (6, 8), (8, 10),
            (11, 13), (13, 15),
            (12, 14), (14, 16),
        ]

        self.NOSE_ID = 0
        self.LEFT_SHOULDER_ID = 5
        self.RIGHT_SHOULDER_ID = 6
        self.LEFT_HIP_ID = 11
        self.RIGHT_HIP_ID = 12

        self.min_confidence_for_yaw = 0.5
        self.min_shoulder_distance = 0.15
        self.max_shoulder_distance = 0.6
        self.min_hip_distance = 0.15
        self.max_hip_distance = 0.5

        self.yaw_history = []
        self.max_history_size = 5
        self.yaw_smoothing_alpha = 0.3

        self.get_logger().info(
            f"detection_period_sec={self.detection_period_sec}, "
            f"inference_resize={self.inference_resize}"
        )

    def _limit_onnx_threads(self, num_threads: int) -> None:
        """Limit onnxruntime to `num_threads` threads per inference.

        ultralytics does not accept session_options through YOLO(...), so the
        InferenceSession class is wrapped before YOLO() is called. This depends only on
        onnxruntime (not on the ultralytics version) and has no effect for non-ONNX
        models. OMP_NUM_THREADS is not enough: onnxruntime uses its own thread pool.
        """
        if num_threads <= 0:
            self.get_logger().warn(
                "onnx_num_threads <= 0: onnxruntime chooses the thread count. "
                "On many-core machines this can take >1000% CPU and make "
                "Nav2 abort goals."
            )
            return
        try:
            import onnxruntime as ort
        except ImportError:
            return

        original = ort.InferenceSession

        def limited(path_or_bytes, sess_options=None, providers=None, **kwargs):
            options = sess_options or ort.SessionOptions()
            options.intra_op_num_threads = num_threads
            options.inter_op_num_threads = 1
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            return original(path_or_bytes, options, providers=providers, **kwargs)

        ort.InferenceSession = limited
        self.get_logger().info(
            f"ONNX Runtime limited to {num_threads} threads per inference"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_info = msg
            self.camera_info_received = True

    def validate_keypoint_pair(self, kpt1, kpt2, min_dist: float, max_dist: float) -> bool:
        if kpt1.confidence < self.min_confidence_for_yaw or kpt2.confidence < self.min_confidence_for_yaw:
            return False
        dx = kpt2.x - kpt1.x
        dy = kpt2.y - kpt1.y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < min_dist or distance > max_dist:
            return False
        return True

    def calculate_yaw_from_pair(self, kpt1, kpt2, is_perpendicular: bool = True) -> Optional[float]:
        dx = kpt2.x - kpt1.x
        dy = kpt2.y - kpt1.y
        angle = math.atan2(dy, dx)
        if is_perpendicular:
            yaw = angle + math.pi / 2.0
        else:
            yaw = angle
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        return yaw

    def smooth_yaw_angle(self, new_yaw: float) -> float:
        if len(self.yaw_history) == 0:
            self.yaw_history.append(new_yaw)
            return new_yaw
        prev_yaw = self.yaw_history[-1]
        angle_diff = new_yaw - prev_yaw
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        smoothed_yaw = prev_yaw + self.yaw_smoothing_alpha * angle_diff
        smoothed_yaw = math.atan2(math.sin(smoothed_yaw), math.cos(smoothed_yaw))
        self.yaw_history.append(smoothed_yaw)
        if len(self.yaw_history) > self.max_history_size:
            self.yaw_history.pop(0)
        return smoothed_yaw

    def calculate_yaw_angle(self, keypoint_dict: dict) -> Optional[float]:
        yaw_candidates = []
        weights = []

        if self.LEFT_HIP_ID in keypoint_dict and self.RIGHT_HIP_ID in keypoint_dict:
            left_hip = keypoint_dict[self.LEFT_HIP_ID]
            right_hip = keypoint_dict[self.RIGHT_HIP_ID]
            if self.validate_keypoint_pair(left_hip, right_hip, self.min_hip_distance, self.max_hip_distance):
                yaw = self.calculate_yaw_from_pair(left_hip, right_hip, is_perpendicular=True)
                weight = (left_hip.confidence + right_hip.confidence) / 2.0 * 2.0
                yaw_candidates.append(yaw)
                weights.append(weight)

        if self.LEFT_SHOULDER_ID in keypoint_dict and self.RIGHT_SHOULDER_ID in keypoint_dict:
            left_shoulder = keypoint_dict[self.LEFT_SHOULDER_ID]
            right_shoulder = keypoint_dict[self.RIGHT_SHOULDER_ID]
            if self.validate_keypoint_pair(left_shoulder, right_shoulder,
                                           self.min_shoulder_distance, self.max_shoulder_distance):
                yaw = self.calculate_yaw_from_pair(left_shoulder, right_shoulder, is_perpendicular=True)
                weight = (left_shoulder.confidence + right_shoulder.confidence) / 2.0 * 1.5
                yaw_candidates.append(yaw)
                weights.append(weight)

        if self.NOSE_ID in keypoint_dict and keypoint_dict[self.NOSE_ID].confidence > 0.6:
            nose = keypoint_dict[self.NOSE_ID]
            torso_center_x, torso_center_y = None, None
            if self.LEFT_SHOULDER_ID in keypoint_dict and self.RIGHT_SHOULDER_ID in keypoint_dict:
                ls = keypoint_dict[self.LEFT_SHOULDER_ID]
                rs = keypoint_dict[self.RIGHT_SHOULDER_ID]
                if ls.confidence > self.min_confidence_for_yaw and rs.confidence > self.min_confidence_for_yaw:
                    torso_center_x = (ls.x + rs.x) / 2.0
                    torso_center_y = (ls.y + rs.y) / 2.0
            elif self.LEFT_HIP_ID in keypoint_dict and self.RIGHT_HIP_ID in keypoint_dict:
                lh = keypoint_dict[self.LEFT_HIP_ID]
                rh = keypoint_dict[self.RIGHT_HIP_ID]
                if lh.confidence > self.min_confidence_for_yaw and rh.confidence > self.min_confidence_for_yaw:
                    torso_center_x = (lh.x + rh.x) / 2.0
                    torso_center_y = (lh.y + rh.y) / 2.0
            if torso_center_x is not None:
                class TorsoCenter:
                    def __init__(self, x, y, z, conf):
                        self.x, self.y, self.z, self.confidence = x, y, z, conf
                torso_center = TorsoCenter(torso_center_x, torso_center_y, nose.z, 0.5)
                yaw = self.calculate_yaw_from_pair(torso_center, nose, is_perpendicular=False)
                weight = nose.confidence * 0.5
                yaw_candidates.append(yaw)
                weights.append(weight)

        if len(yaw_candidates) > 0 and sum(weights) > 0:
            sum_sin = sum(w * math.sin(y) for y, w in zip(yaw_candidates, weights))
            sum_cos = sum(w * math.cos(y) for y, w in zip(yaw_candidates, weights))
            final_yaw = math.atan2(sum_sin, sum_cos)
            return self.smooth_yaw_angle(final_yaw)
        self.yaw_history.clear()
        return None

    def quaternion_from_yaw(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _depth_at(self, depth_img, x, y):
        """Depth at (x, y): median of a small patch around it, ignoring NaN and <= 0.

        A single pixel reads the wall behind the person as soon as the keypoint is off
        by a few pixels; the median of a patch is robust to a few bad pixels.
        """
        r = self.depth_patch_radius
        if r <= 0:
            d = depth_img[y, x]
            return float(d) if (not np.isnan(d) and d > 0) else None
        y0 = max(0, y - r)
        y1 = min(depth_img.shape[0], y + r + 1)
        x0 = max(0, x - r)
        x1 = min(depth_img.shape[1], x + r + 1)
        manh = depth_img[y0:y1, x0:x1].reshape(-1)
        manh = manh[np.isfinite(manh) & (manh > 0)]
        if manh.size == 0:
            return None
        return float(np.median(manh))

    def _depth_image_to_meters(self, depth_msg: Image):
        """Return the depth image in METRES, whatever the camera sends.

        This is where simulation and the real robot differ:
            Gazebo (ros_gz_bridge) : 32FC1, metres
            RealSense D435         : 16UC1 (Z16), millimetres
        cv_bridge only converts the type, not the unit (1500 mm becomes 1500.0, not
        1.5), so every real depth value would be 1000x too large and fail the
        `depth < 10.0` filter. The conversion is therefore done here explicitly.
        """
        encoding = depth_msg.encoding
        first_time = encoding != self._depth_encoding_seen
        self._depth_encoding_seen = encoding

        if encoding in ('16UC1', 'mono16'):
            if first_time:
                self.get_logger().info(
                    f"Depth image '{encoding}': millimetres -> scaled by 0.001 to metres."
                )
            # Read the raw encoding and scale here. Do not let cv_bridge convert to 32FC1:
            # that conversion loses the unit.
            raw = self.bridge.imgmsg_to_cv2(depth_msg, encoding)
            return raw.astype(np.float32) * 0.001

        if encoding == '32FC1':
            if first_time:
                self.get_logger().info("Depth image '32FC1': already in metres.")
            return self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')

        if first_time:
            self.get_logger().error(
                f"Unsupported depth encoding: '{encoding}'. Only 16UC1/"
                "mono16 (millimetres) or 32FC1 (metres) are accepted. Depth frames are "
                "skipped until the camera sends a supported type."
            )
        return None

    def image_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self.camera_info is None:
            if not hasattr(self, '_camera_info_warned'):
                self.get_logger().warn("Camera info not received yet, skipping...")
                self._camera_info_warned = True
            return

        # Throttle: skip YOLO inference until detection_period_sec has passed since the
        # previous one, leaving CPU for the controller and costmaps.
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_infer_ns) / 1e9 < self.detection_period_sec:
            return
        self._last_infer_ns = now_ns

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self._depth_image_to_meters(depth_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting images: {e}")
            return

        if depth_img is None:
            return

        # Resize before inference to reduce CPU. fx, fy, cx, cy and depth_img stay at
        # the ORIGINAL resolution -- coordinates must be scaled back before use.
        if self.inference_resize is not None:
            infer_img = cv2.resize(rgb_img, self.inference_resize)
            scale_x = rgb_img.shape[1] / self.inference_resize[0]
            scale_y = rgb_img.shape[0] / self.inference_resize[1]
        else:
            infer_img = rgb_img
            scale_x = 1.0
            scale_y = 1.0

        results = self.model(infer_img, verbose=False)

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        has_detections = (
            results
            and len(results) > 0
            and hasattr(results[0], 'keypoints')
            and results[0].keypoints.xy.shape[0] > 0
        )

        transform = None
        if has_detections:
            try:
                image_time = rclpy.time.Time.from_msg(rgb_msg.header.stamp)

                # Use the LATEST transform (Time() = 0), not the image timestamp: the image is
                # stamped at capture and the transform arrives tens of ms later, so an exact
                # lookup always fails with "extrapolation into the future", and waiting would
                # block the single-threaded executor. The cost is ~1 cm (0.4 m/s x ~30 ms),
                # negligible compared to the person position error.
                transform = self.tf_buffer.lookup_transform(
                    "odom",
                    rgb_msg.header.frame_id,
                    rclpy.time.Time(),
                )
                self._last_good_camera_transform = transform
                self._last_good_camera_transform_ns = (
                    self.get_clock().now().nanoseconds
                )
            except tf2_ros.TransformException as e:
                now_ns2 = self.get_clock().now().nanoseconds
                age = (
                    now_ns2 - self._last_good_camera_transform_ns
                ) / 1e9
                if (
                    self._last_good_camera_transform is not None
                    and age <= self.camera_transform_max_stale_seconds
                ):
                    transform = self._last_good_camera_transform
                else:
                    self.get_logger().warn(f"Transform error: {e}")

        all_pose_results: list = []  # list of (person_id, PoseResult3D)

        if has_detections and transform is not None:
            all_keypoints_2d = results[0].keypoints.xy.cpu().numpy()
            all_confidence = results[0].keypoints.conf.cpu().numpy()

            # Confidence threshold of the PERSON detection itself (box confidence), in
            # addition to the per-keypoint confidence filter (0.1).
            box_conf = None
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                box_conf = results[0].boxes.conf.cpu().numpy()

            for person_id in range(all_keypoints_2d.shape[0]):
                if (
                    box_conf is not None
                    and person_id < len(box_conf)
                    and float(box_conf[person_id]) < self.person_conf_min
                ):
                    continue
                keypoints_2d = all_keypoints_2d[person_id]
                confidence = all_confidence[person_id]

                keypoints_3d_camera = []
                # Diagnostics: log the raw depth of EVERY keypoint tried, including rejected
                # ones, so failures are visible too.
                raw_depths = []
                for idx, kpt_2d in enumerate(keypoints_2d):
                    if confidence[idx] < 0.1:
                        continue
                    # Scale back to the ORIGINAL resolution (rgb_img) before looking up
                    # depth_img / camera_info. Mandatory with inference_resize; without it the
                    # 3D back-projection is completely wrong.
                    x_2d = int(kpt_2d[0] * scale_x)
                    y_2d = int(kpt_2d[1] * scale_y)
                    if x_2d == 0 and y_2d == 0:
                        continue
                    if 0 <= y_2d < depth_img.shape[0] and 0 <= x_2d < depth_img.shape[1]:
                        # MEDIAN of a small patch around the pixel, not a single pixel: a keypoint off
                        # by 1-2 pixels reads the wall behind the person and puts the 3D point metres
                        # away. A few bad pixels cannot flip the median.
                        depth = self._depth_at(depth_img, x_2d, y_2d)
                        raw_depths.append(depth)
                        if depth is not None and depth < 10.0:
                            x_3d_camera = (x_2d - cx) * depth / fx
                            y_3d_camera = (y_2d - cy) * depth / fy
                            z_3d_camera = depth
                            keypoints_3d_camera.append({
                                'id': idx,
                                'x': x_3d_camera,
                                'y': y_3d_camera,
                                'z': z_3d_camera,
                                'confidence': float(confidence[idx]),
                            })

                try:
                    self.debug_depth_log = bool(
                        self.get_parameter("debug_depth_log").value
                    )
                except Exception:
                    pass
                if self.debug_depth_log:
                    try:
                        bx = results[0].boxes.xyxy.cpu().numpy()[person_id]
                        # Scale like the keypoints: boxes are in the RESIZED image (256x192) while
                        # depth_img is 640x480.
                        bx1 = int(bx[0] * scale_x)
                        by1 = int(bx[1] * scale_y)
                        bx2 = int(bx[2] * scale_x)
                        by2 = int(bx[3] * scale_y)
                        bw = bx2 - bx1
                        bh = by2 - by1
                        core = depth_img[
                            by1 + bh // 3: by1 + 2 * bh // 3,
                            bx1 + bw // 3: bx1 + 2 * bw // 3,
                        ].reshape(-1)
                        core = core[np.isfinite(core) & (core > 0)]
                        z_box = float(np.median(core)) if core.size else float('nan')
                        zs = [round(float(k['z']), 2) for k in keypoints_3d_camera]
                        n_tried = len(raw_depths)
                        n_none = sum(1 for d in raw_depths if d is None)
                        n_far = sum(
                            1 for d in raw_depths if d is not None and d >= 10.0
                        )
                        n_passed = n_tried - n_none - n_far
                        raw_txt = ",".join(
                            ("None" if d is None else str(round(float(d), 2)))
                            for d in raw_depths
                        )
                        self.get_logger().info(
                            "DEPTH_DEBUG z_box=%.2f n_tried=%d n_none=%d n_far10=%d "
                            "n_passed=%d n_kept=%d raw=%s"
                            % (z_box, n_tried, n_none, n_far, n_passed, len(zs), raw_txt)
                        )
                    except Exception as e:
                        self.get_logger().warn("DEPTH_DEBUG error: %s" % e)

                if len(keypoints_3d_camera) == 0:
                    continue

                # Common depth gate: all keypoints of ONE person must be at a similar depth.
                # A keypoint farther than depth_consistency_m from the median hit the
                # background -> drop it.
                if self.depth_consistency_m > 0.0 and len(keypoints_3d_camera) >= 3:
                    zs = sorted(k['z'] for k in keypoints_3d_camera)
                    z_anchor = zs[len(zs) // 2]
                    kept = [
                        k for k in keypoints_3d_camera
                        if abs(k['z'] - z_anchor) <= self.depth_consistency_m
                    ]
                    if len(kept) >= 2:
                        keypoints_3d_camera = kept

                person_pose_result = PoseResult3D()
                person_pose_result.header.stamp = rgb_msg.header.stamp
                person_pose_result.header.frame_id = 'odom'

                for kpt_camera in keypoints_3d_camera:
                    point_camera = PointStamped()
                    point_camera.header.frame_id = rgb_msg.header.frame_id
                    point_camera.header.stamp = transform.header.stamp
                    point_camera.point.x = float(kpt_camera['x'])
                    point_camera.point.y = float(kpt_camera['y'])
                    point_camera.point.z = float(kpt_camera['z'])
                    point_world = do_transform_point(point_camera, transform)
                    keypoint = PoseKeypoint3D()
                    keypoint.id = kpt_camera['id']
                    keypoint.x = point_world.point.x
                    keypoint.y = point_world.point.y
                    keypoint.z = point_world.point.z
                    keypoint.confidence = kpt_camera['confidence']
                    person_pose_result.keypoints.append(keypoint)

                if len(person_pose_result.keypoints) > 0:
                    all_pose_results.append((person_id, person_pose_result))

        multiple_msg = MultiplePoseResult3D()
        multiple_msg.header.stamp = rgb_msg.header.stamp
        multiple_msg.header.frame_id = 'odom'
        multiple_msg.poses_result_3d = [
            pose_result for _, pose_result in all_pose_results
        ]
        self.multiple_pose_publisher.publish(multiple_msg)

        if len(all_pose_results) > 0:
            self.publish_skeleton_markers(all_pose_results)
            all_orientation_markers = MarkerArray()
            for person_id, pose_result in all_pose_results:
                yaw_angle = self.calculate_yaw_angle({kpt.id: kpt for kpt in pose_result.keypoints})
                if yaw_angle is not None:
                    markers = self._create_orientation_markers_internal(pose_result, yaw_angle, person_id)
                    all_orientation_markers.markers.extend(markers)
                else:
                    delete_marker = Marker()
                    delete_marker.action = Marker.DELETEALL
                    delete_marker.ns = f"person_{person_id}_orientation"
                    all_orientation_markers.markers.append(delete_marker)
            self.orientation_publisher.publish(all_orientation_markers)
        else:
            self.publish_skeleton_markers([])
            all_orientation_markers = MarkerArray()
            for pid in range(10):
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                delete_marker.ns = f"person_{pid}_orientation"
                all_orientation_markers.markers.append(delete_marker)
            self.orientation_publisher.publish(all_orientation_markers)

        # Read the parameter at RUN time, so it can be toggled with
        # `ros2 param set /yolo_pose_node1 publish_debug_image true` without a restart.
        # Publish even when nothing is detected, so RViz always shows an image --
        # that is exactly when one wants to see what YOLO sees.
        try:
            self.publish_debug_image = bool(
                self.get_parameter("publish_debug_image").value
            )
        except Exception:
            pass

        if self.publish_debug_image and results and len(results) > 0:
            annotated_frame = np.ascontiguousarray(results[0].plot())
            # Build the message by hand instead of cv_bridge.cv2_to_imgmsg(): with
            # ultralytics, OpenCV 5 in ~/.local can shadow the system OpenCV 4 that
            # cv_bridge was built against, and the mismatching type codes raise KeyError.
            # bgr8 is just a contiguous byte array, so no cv_bridge is needed.
            img_msg = Image()
            img_msg.header = rgb_msg.header
            img_msg.height, img_msg.width = annotated_frame.shape[:2]
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3
            img_msg.data = annotated_frame.tobytes()
            self.image_publisher.publish(img_msg)
        elif self.publish_debug_image:
            # No detection -> still publish the ORIGINAL image so RViz always shows something.
            annotated_frame = np.ascontiguousarray(rgb_img)
            img_msg = Image()
            img_msg.header = rgb_msg.header
            img_msg.height, img_msg.width = annotated_frame.shape[:2]
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3
            img_msg.data = annotated_frame.tobytes()
            self.image_publisher.publish(img_msg)

    def publish_skeleton_markers(self, all_pose_results: list) -> None:
        marker_array = MarkerArray()
        if len(all_pose_results) == 0:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.marker_publisher.publish(marker_array)
            return

        max_line_length_m = 0.75

        for person_id, pose_result in all_pose_results:
            if pose_result is None or len(pose_result.keypoints) == 0:
                continue
            keypoint_dict = {kpt.id: kpt for kpt in pose_result.keypoints}

            valid_line_tuples = []
            connected_ids = set()
            for start_id, end_id in self.skeleton_connections:
                if start_id not in keypoint_dict or end_id not in keypoint_dict:
                    continue
                start_kpt = keypoint_dict[start_id]
                end_kpt = keypoint_dict[end_id]
                if start_kpt.confidence <= 0.3 or end_kpt.confidence <= 0.3:
                    continue
                length = math.sqrt(
                    (end_kpt.x - start_kpt.x) ** 2
                    + (end_kpt.y - start_kpt.y) ** 2
                    + (end_kpt.z - start_kpt.z) ** 2
                )
                if length <= max_line_length_m:
                    valid_line_tuples.append((start_kpt, end_kpt))
                    connected_ids.add(start_id)
                    connected_ids.add(end_id)

            for kpt in pose_result.keypoints:
                if kpt.id not in connected_ids:
                    continue
                marker = Marker()
                marker.header = pose_result.header
                marker.ns = f"person_{person_id}_keypoints"
                marker.id = kpt.id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = kpt.x
                marker.pose.position.y = kpt.y
                marker.pose.position.z = kpt.z
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime.sec = 1
                marker_array.markers.append(marker)

            line_id = 1000 * (person_id + 1)
            for start_kpt, end_kpt in valid_line_tuples:
                marker = Marker()
                marker.header = pose_result.header
                marker.ns = f"person_{person_id}_lines"
                marker.id = line_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.02
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime.sec = 1
                start_point = Point()
                start_point.x = start_kpt.x
                start_point.y = start_kpt.y
                start_point.z = start_kpt.z
                end_point = Point()
                end_point.x = end_kpt.x
                end_point.y = end_kpt.y
                end_point.z = end_kpt.z
                marker.points = [start_point, end_point]
                marker_array.markers.append(marker)
                line_id += 1

        self.marker_publisher.publish(marker_array)

    def _create_orientation_markers_internal(
        self, pose_result: PoseResult3D, yaw_angle: float, person_id: int
    ) -> list:
        markers = []
        keypoint_dict = {kpt.id: kpt for kpt in pose_result.keypoints}
        center_x, center_y, center_z = 0.0, 0.0, 0.0
        count = 0

        if self.LEFT_SHOULDER_ID in keypoint_dict and self.RIGHT_SHOULDER_ID in keypoint_dict:
            ls = keypoint_dict[self.LEFT_SHOULDER_ID]
            rs = keypoint_dict[self.RIGHT_SHOULDER_ID]
            center_x += (ls.x + rs.x) / 2.0
            center_y += (ls.y + rs.y) / 2.0
            center_z += (ls.z + rs.z) / 2.0
            count += 1
        if self.LEFT_HIP_ID in keypoint_dict and self.RIGHT_HIP_ID in keypoint_dict:
            lh = keypoint_dict[self.LEFT_HIP_ID]
            rh = keypoint_dict[self.RIGHT_HIP_ID]
            center_x += (lh.x + rh.x) / 2.0
            center_y += (lh.y + rh.y) / 2.0
            center_z += (lh.z + rh.z) / 2.0
            count += 1

        if count > 0:
            center_x /= count
            center_y /= count
            center_z /= count
        else:
            for kpt in pose_result.keypoints:
                center_x += kpt.x
                center_y += kpt.y
                center_z += kpt.z
            n = len(pose_result.keypoints)
            center_x /= n
            center_y /= n
            center_z /= n

        arrow_marker = Marker()
        arrow_marker.header = pose_result.header
        arrow_marker.ns = f"person_{person_id}_orientation"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = center_x
        arrow_marker.pose.position.y = center_y
        arrow_marker.pose.position.z = center_z
        arrow_marker.pose.orientation = self.quaternion_from_yaw(yaw_angle)
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.15
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 1.0
        arrow_marker.color.a = 1.0
        arrow_marker.lifetime.sec = 1
        markers.append(arrow_marker)

        text_marker = Marker()
        text_marker.header = pose_result.header
        text_marker.ns = f"person_{person_id}_orientation"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = center_x
        text_marker.pose.position.y = center_y
        text_marker.pose.position.z = center_z + 0.3
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = f"P{person_id} Yaw: {math.degrees(yaw_angle):.1f} deg"
        text_marker.lifetime.sec = 1
        markers.append(text_marker)

        return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseEstimationMultiple3DNode1()

    # Single-threaded spin, not MultiThreadedExecutor(2): measured with no images
    # arriving, the multi-threaded executor alone used 91.5% of a core versus 1.9%
    # for rclpy.spin. It also removes a data race: image_callback writes
    # self._last_infer_ns, self.yaw_history and self._last_good_camera_transform
    # without locks. camera_info_callback only does work on its first call.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
