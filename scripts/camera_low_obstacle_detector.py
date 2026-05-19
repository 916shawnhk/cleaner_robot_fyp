#!/usr/bin/env python3

import math
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger


class CameraLowObstacleDetector(Node):
    """Detect low floor obstacles using edge/texture anomaly in the lower camera ROI."""

    def __init__(self):
        super().__init__('camera_low_obstacle_detector')

        self.declare_parameter('enabled', True)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('debug_image_topic', '/low_obstacle/debug_image')
        self.declare_parameter('detection_topic', '/low_obstacle/detected')
        self.declare_parameter('confidence_topic', '/low_obstacle/confidence')
        self.declare_parameter('obstacle_scan_topic', '/low_obstacle/scan')
        self.declare_parameter('processing_rate_hz', 6.0)
        self.declare_parameter('debug_publish_rate_hz', 3.0)
        self.declare_parameter('roi_top_ratio', 0.56)
        self.declare_parameter('roi_bottom_ratio', 0.98)
        self.declare_parameter('roi_left_ratio', 0.08)
        self.declare_parameter('roi_right_ratio', 0.92)
        self.declare_parameter('danger_left_ratio', 0.30)
        self.declare_parameter('danger_right_ratio', 0.70)
        self.declare_parameter('blur_kernel', 5)
        self.declare_parameter('canny_sigma', 0.33)
        self.declare_parameter('edge_density_threshold', 0.035)
        self.declare_parameter('center_edge_density_threshold', 0.045)
        self.declare_parameter('texture_variance_threshold', 280.0)
        self.declare_parameter('min_contour_area', 80.0)
        self.declare_parameter('min_contour_count', 2)
        self.declare_parameter('use_floor_model', True)
        self.declare_parameter('floor_model_learning', True)
        self.declare_parameter('floor_model_warmup_frames', 20)
        self.declare_parameter('floor_model_update_alpha', 0.02)
        self.declare_parameter('floor_model_l_weight', 0.20)
        self.declare_parameter('floor_model_ab_weight', 1.0)
        self.declare_parameter('floor_model_gradient_weight', 0.8)
        self.declare_parameter('floor_model_l_contrast', 1.0)
        self.declare_parameter('floor_model_ab_contrast', 1.0)
        self.declare_parameter('floor_model_normalize_illumination', True)
        self.declare_parameter('floor_model_illumination_blur', 41)
        self.declare_parameter('floor_model_use_clahe', True)
        self.declare_parameter('floor_model_clahe_clip_limit', 2.0)
        self.declare_parameter('floor_model_diff_threshold', 28.0)
        self.declare_parameter('floor_model_area_ratio_threshold', 0.045)
        self.declare_parameter('floor_model_min_contour_area', 450.0)
        self.declare_parameter('floor_model_min_contours', 1)
        self.declare_parameter('floor_model_morph_kernel', 5)
        self.declare_parameter('detect_frames', 2)
        self.declare_parameter('clear_frames', 4)
        self.declare_parameter('overlay_scale', 1.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_obstacle_scan', True)
        self.declare_parameter('obstacle_scan_frame_id', 'base_link')
        self.declare_parameter('obstacle_scan_angle_min', -3.14159)
        self.declare_parameter('obstacle_scan_angle_max', 3.14159)
        self.declare_parameter('obstacle_scan_mark_angle_min', -0.55)
        self.declare_parameter('obstacle_scan_mark_angle_max', 0.55)
        self.declare_parameter('obstacle_scan_angle_increment', 0.02)
        self.declare_parameter('obstacle_scan_range_min', 0.05)
        self.declare_parameter('obstacle_scan_range_max', 1.0)
        self.declare_parameter('obstacle_scan_distance', 0.35)
        self.declare_parameter('obstacle_scan_padding_angle', 0.08)

        self._bridge = CvBridge()
        self._last_process_monotonic = 0.0
        self._last_debug_monotonic = 0.0
        self._detected_streak = 0
        self._clear_streak = 0
        self._detected = False
        self._confidence = 0.0
        self._last_obstacle_angle_min: Optional[float] = None
        self._last_obstacle_angle_max: Optional[float] = None
        self._floor_model: Optional[np.ndarray] = None
        self._floor_model_shape = None
        self._floor_model_frames = 0
        self._reset_floor_model_requested = False

        self._read_parameters()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._detected_pub = self.create_publisher(Bool, self._detection_topic, 10)
        self._confidence_pub = self.create_publisher(Float32, self._confidence_topic, 10)
        self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 2)
        self._obstacle_scan_pub = self.create_publisher(LaserScan, self._obstacle_scan_topic, 10)
        self.create_subscription(Image, self._image_topic, self._image_callback, 5)
        self.create_service(Trigger, '/low_obstacle/reset_floor_model', self._reset_floor_model_callback)
        self.create_timer(0.2, self._publish_state)

        self.get_logger().info(
            f'CameraLowObstacleDetector started: image={self._image_topic}, '
            f'detection={self._detection_topic}, debug={self._debug_image_topic}'
        )

    def _read_parameters(self):
        self._enabled = bool(self.get_parameter('enabled').value)
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self._detection_topic = str(self.get_parameter('detection_topic').value)
        self._confidence_topic = str(self.get_parameter('confidence_topic').value)
        self._obstacle_scan_topic = str(self.get_parameter('obstacle_scan_topic').value)
        self._processing_period = 1.0 / max(0.1, float(self.get_parameter('processing_rate_hz').value))
        self._debug_period = 1.0 / max(0.1, float(self.get_parameter('debug_publish_rate_hz').value))
        self._roi_top_ratio = self._clamp_ratio('roi_top_ratio')
        self._roi_bottom_ratio = self._clamp_ratio('roi_bottom_ratio')
        self._roi_left_ratio = self._clamp_ratio('roi_left_ratio')
        self._roi_right_ratio = self._clamp_ratio('roi_right_ratio')
        self._danger_left_ratio = self._clamp_ratio('danger_left_ratio')
        self._danger_right_ratio = self._clamp_ratio('danger_right_ratio')
        self._blur_kernel = max(1, int(self.get_parameter('blur_kernel').value))
        if self._blur_kernel % 2 == 0:
            self._blur_kernel += 1
        self._canny_sigma = max(0.01, float(self.get_parameter('canny_sigma').value))
        self._edge_density_threshold = max(0.0, float(self.get_parameter('edge_density_threshold').value))
        self._center_edge_density_threshold = max(
            0.0,
            float(self.get_parameter('center_edge_density_threshold').value),
        )
        self._texture_variance_threshold = max(0.0, float(self.get_parameter('texture_variance_threshold').value))
        self._min_contour_area = max(0.0, float(self.get_parameter('min_contour_area').value))
        self._min_contour_count = max(0, int(self.get_parameter('min_contour_count').value))
        self._use_floor_model = bool(self.get_parameter('use_floor_model').value)
        self._floor_model_learning = bool(self.get_parameter('floor_model_learning').value)
        self._floor_model_warmup_frames = max(1, int(self.get_parameter('floor_model_warmup_frames').value))
        self._floor_model_update_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter('floor_model_update_alpha').value)),
        )
        self._floor_model_l_weight = max(0.0, float(self.get_parameter('floor_model_l_weight').value))
        self._floor_model_ab_weight = max(0.0, float(self.get_parameter('floor_model_ab_weight').value))
        self._floor_model_gradient_weight = max(
            0.0,
            float(self.get_parameter('floor_model_gradient_weight').value),
        )
        self._floor_model_l_contrast = max(0.1, float(self.get_parameter('floor_model_l_contrast').value))
        self._floor_model_ab_contrast = max(0.1, float(self.get_parameter('floor_model_ab_contrast').value))
        self._floor_model_normalize_illumination = bool(
            self.get_parameter('floor_model_normalize_illumination').value
        )
        self._floor_model_illumination_blur = max(
            3,
            int(self.get_parameter('floor_model_illumination_blur').value),
        )
        if self._floor_model_illumination_blur % 2 == 0:
            self._floor_model_illumination_blur += 1
        self._floor_model_use_clahe = bool(self.get_parameter('floor_model_use_clahe').value)
        self._floor_model_clahe_clip_limit = max(
            0.1,
            float(self.get_parameter('floor_model_clahe_clip_limit').value),
        )
        self._floor_model_diff_threshold = max(0.0, float(self.get_parameter('floor_model_diff_threshold').value))
        self._floor_model_area_ratio_threshold = max(
            0.0,
            float(self.get_parameter('floor_model_area_ratio_threshold').value),
        )
        self._floor_model_min_contour_area = max(
            0.0,
            float(self.get_parameter('floor_model_min_contour_area').value),
        )
        self._floor_model_min_contours = max(0, int(self.get_parameter('floor_model_min_contours').value))
        self._floor_model_morph_kernel = max(1, int(self.get_parameter('floor_model_morph_kernel').value))
        if self._floor_model_morph_kernel % 2 == 0:
            self._floor_model_morph_kernel += 1
        self._detect_frames = max(1, int(self.get_parameter('detect_frames').value))
        self._clear_frames = max(1, int(self.get_parameter('clear_frames').value))
        self._overlay_scale = max(0.2, float(self.get_parameter('overlay_scale').value))
        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._publish_obstacle_scan = bool(self.get_parameter('publish_obstacle_scan').value)
        self._obstacle_scan_frame_id = str(self.get_parameter('obstacle_scan_frame_id').value)
        self._obstacle_scan_angle_min = float(self.get_parameter('obstacle_scan_angle_min').value)
        self._obstacle_scan_angle_max = float(self.get_parameter('obstacle_scan_angle_max').value)
        if self._obstacle_scan_angle_max <= self._obstacle_scan_angle_min:
            self._obstacle_scan_angle_max = self._obstacle_scan_angle_min + 0.1
        self._obstacle_scan_mark_angle_min = float(self.get_parameter('obstacle_scan_mark_angle_min').value)
        self._obstacle_scan_mark_angle_max = float(self.get_parameter('obstacle_scan_mark_angle_max').value)
        if self._obstacle_scan_mark_angle_max <= self._obstacle_scan_mark_angle_min:
            self._obstacle_scan_mark_angle_max = self._obstacle_scan_mark_angle_min + 0.1
        self._obstacle_scan_angle_increment = max(
            0.005,
            float(self.get_parameter('obstacle_scan_angle_increment').value),
        )
        self._obstacle_scan_range_min = max(0.0, float(self.get_parameter('obstacle_scan_range_min').value))
        self._obstacle_scan_range_max = max(
            self._obstacle_scan_range_min + 0.01,
            float(self.get_parameter('obstacle_scan_range_max').value),
        )
        self._obstacle_scan_distance = min(
            self._obstacle_scan_range_max,
            max(self._obstacle_scan_range_min, float(self.get_parameter('obstacle_scan_distance').value)),
        )
        self._obstacle_scan_padding_angle = max(
            0.0,
            float(self.get_parameter('obstacle_scan_padding_angle').value),
        )

    def _clamp_ratio(self, name: str) -> float:
        return min(1.0, max(0.0, float(self.get_parameter(name).value)))

    def _on_set_parameters(self, _params):
        self._read_parameters()
        return SetParametersResult(successful=True)

    def _reset_floor_model_callback(self, _request, response):
        self._reset_floor_model()
        response.success = True
        response.message = 'Floor model reset; keep the ROI clear for warmup frames.'
        return response

    def _reset_floor_model(self):
        self._floor_model = None
        self._floor_model_shape = None
        self._floor_model_frames = 0
        self._reset_floor_model_requested = False

    def _image_callback(self, msg: Image):
        now = time.monotonic()
        if now - self._last_process_monotonic < self._processing_period:
            return
        self._last_process_monotonic = now

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert camera image: {exc}')
            return

        detected_now, confidence, debug = self._analyze(frame)
        self._confidence = confidence

        if detected_now and self._enabled:
            self._detected_streak += 1
            self._clear_streak = 0
        else:
            self._clear_streak += 1
            self._detected_streak = 0

        if self._detected_streak >= self._detect_frames:
            self._detected = True
        elif self._clear_streak >= self._clear_frames:
            self._detected = False

        self._publish_state()
        self._publish_obstacle_scan_msg()

        if self._publish_debug_image and now - self._last_debug_monotonic >= self._debug_period:
            self._last_debug_monotonic = now
            if abs(self._overlay_scale - 1.0) > 1e-3:
                debug = cv2.resize(
                    debug,
                    None,
                    fx=self._overlay_scale,
                    fy=self._overlay_scale,
                    interpolation=cv2.INTER_AREA,
                )
            debug_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

    def _analyze(self, frame):
        h, w = frame.shape[:2]
        x0 = int(round(w * min(self._roi_left_ratio, self._roi_right_ratio)))
        x1 = int(round(w * max(self._roi_left_ratio, self._roi_right_ratio)))
        y0 = int(round(h * min(self._roi_top_ratio, self._roi_bottom_ratio)))
        y1 = int(round(h * max(self._roi_top_ratio, self._roi_bottom_ratio)))
        x0, x1 = max(0, x0), min(w, max(x0 + 1, x1))
        y0, y1 = max(0, y0), min(h, max(y0 + 1, y1))

        roi = frame[y0:y1, x0:x1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self._blur_kernel, self._blur_kernel), 0)
        floor_features = self._floor_features(roi)

        median = float(np.median(blurred))
        lower = int(max(0, (1.0 - self._canny_sigma) * median))
        upper = int(min(255, (1.0 + self._canny_sigma) * median))
        if upper <= lower:
            upper = min(255, lower + 30)

        edges = cv2.Canny(blurred, lower, upper)
        lap = cv2.Laplacian(blurred, cv2.CV_64F)
        texture_variance = float(lap.var())

        roi_h, roi_w = edges.shape[:2]
        danger_x0 = int(round(roi_w * min(self._danger_left_ratio, self._danger_right_ratio)))
        danger_x1 = int(round(roi_w * max(self._danger_left_ratio, self._danger_right_ratio)))
        danger_x0, danger_x1 = max(0, danger_x0), min(roi_w, max(danger_x0 + 1, danger_x1))
        danger_edges = edges[:, danger_x0:danger_x1]

        edge_density = float(np.count_nonzero(edges)) / float(edges.size)
        center_edge_density = float(np.count_nonzero(danger_edges)) / float(danger_edges.size)

        contours, _ = cv2.findContours(danger_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        significant_contours = [c for c in contours if cv2.contourArea(c) >= self._min_contour_area]
        contour_count = len(significant_contours)

        edge_score = edge_density / self._edge_density_threshold if self._edge_density_threshold > 0 else 0.0
        center_score = (
            center_edge_density / self._center_edge_density_threshold
            if self._center_edge_density_threshold > 0
            else 0.0
        )
        texture_score = (
            texture_variance / self._texture_variance_threshold
            if self._texture_variance_threshold > 0
            else 0.0
        )
        contour_score = contour_count / max(1, self._min_contour_count)
        # Detection requires all cues to pass, so confidence should represent the weakest cue.
        # This keeps floor grain from showing "conf=1.00" just because it has many thin edges.
        confidence = min(1.0, min(edge_score, center_score, texture_score, contour_score))

        detected = (
            edge_density >= self._edge_density_threshold
            and center_edge_density >= self._center_edge_density_threshold
            and texture_variance >= self._texture_variance_threshold
            and contour_count >= self._min_contour_count
        )
        edge_detected = detected

        (
            floor_detected,
            floor_confidence,
            floor_mask,
            floor_area_ratio,
            floor_contour_count,
            floor_model_ready,
            floor_diff_stats,
        ) = self._analyze_floor_model(floor_features, danger_x0, danger_x1, edge_detected)

        if self._use_floor_model:
            detected = floor_detected if floor_model_ready else False
            confidence = floor_confidence if floor_model_ready else 0.0

        debug = self._draw_debug(
            frame,
            x0,
            y0,
            x1,
            y1,
            danger_x0,
            danger_x1,
            detected,
            confidence,
            floor_mask,
            floor_area_ratio,
            floor_contour_count,
            floor_model_ready,
            floor_diff_stats,
        )
        return detected, confidence, debug

    def _floor_features(self, roi):
        lab_u8 = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l_u8, a_u8, b_u8 = cv2.split(lab_u8)

        if self._floor_model_normalize_illumination:
            l_float = l_u8.astype(np.float32)
            low_frequency = cv2.GaussianBlur(
                l_float,
                (self._floor_model_illumination_blur, self._floor_model_illumination_blur),
                0,
            )
            l_u8 = np.clip(l_float - low_frequency + 128.0, 0, 255).astype(np.uint8)

        if self._floor_model_use_clahe:
            clahe = cv2.createCLAHE(
                clipLimit=self._floor_model_clahe_clip_limit,
                tileGridSize=(8, 8),
            )
            l_u8 = clahe.apply(l_u8)

        if abs(self._floor_model_l_contrast - 1.0) > 1e-3:
            l_u8 = np.clip(
                (l_u8.astype(np.float32) - 128.0) * self._floor_model_l_contrast + 128.0,
                0,
                255,
            ).astype(np.uint8)

        if abs(self._floor_model_ab_contrast - 1.0) > 1e-3:
            a_u8 = np.clip(
                (a_u8.astype(np.float32) - 128.0) * self._floor_model_ab_contrast + 128.0,
                0,
                255,
            ).astype(np.uint8)
            b_u8 = np.clip(
                (b_u8.astype(np.float32) - 128.0) * self._floor_model_ab_contrast + 128.0,
                0,
                255,
            ).astype(np.uint8)

        l = l_u8.astype(np.float32)
        a = a_u8.astype(np.float32)
        b = b_u8.astype(np.float32)

        grad_x = cv2.Sobel(l, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(l, cv2.CV_32F, 0, 1, ksize=3)
        grad = cv2.magnitude(grad_x, grad_y)
        grad = np.clip(grad, 0, 255)

        features = cv2.merge((
            l * self._floor_model_l_weight,
            a * self._floor_model_ab_weight,
            b * self._floor_model_ab_weight,
            grad * self._floor_model_gradient_weight,
        ))
        return cv2.GaussianBlur(features, (self._blur_kernel, self._blur_kernel), 0)

    def _analyze_floor_model(self, features, danger_x0, danger_x1, edge_detected):
        if self._reset_floor_model_requested:
            self._reset_floor_model()

        if self._floor_model_shape != features.shape:
            self._reset_floor_model()
            self._floor_model_shape = features.shape

        if self._floor_model is None:
            self._floor_model = features.copy()
            self._floor_model_shape = features.shape
            self._floor_model_frames = 1
            empty_mask = np.zeros(features.shape[:2], dtype=np.uint8)
            return False, 0.0, empty_mask, 0.0, 0, False, self._empty_floor_diff_stats()

        if self._floor_model_frames < self._floor_model_warmup_frames:
            alpha = 1.0 / float(self._floor_model_frames + 1)
            cv2.accumulateWeighted(features, self._floor_model, alpha)
            self._floor_model_frames += 1
            empty_mask = np.zeros(features.shape[:2], dtype=np.uint8)
            return False, 0.0, empty_mask, 0.0, 0, False, self._empty_floor_diff_stats()

        diff = features - self._floor_model
        floor_diff_stats = self._floor_diff_stats(diff, danger_x0, danger_x1)
        diff_mag = np.sqrt(np.sum(diff * diff, axis=2))
        mask = np.zeros(diff_mag.shape, dtype=np.uint8)
        mask[diff_mag >= self._floor_model_diff_threshold] = 255

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self._floor_model_morph_kernel, self._floor_model_morph_kernel),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        danger_mask = mask[:, danger_x0:danger_x1]
        danger_pixels = max(1, danger_mask.size)
        area_ratio = float(np.count_nonzero(danger_mask)) / float(danger_pixels)
        contours, _ = cv2.findContours(danger_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        significant = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self._floor_model_min_contour_area
        ]
        contour_count = len(significant)

        area_score = (
            area_ratio / self._floor_model_area_ratio_threshold
            if self._floor_model_area_ratio_threshold > 0
            else 0.0
        )
        contour_score = contour_count / max(1, self._floor_model_min_contours)
        confidence = min(1.0, min(area_score, contour_score))
        detected = (
            area_ratio >= self._floor_model_area_ratio_threshold
            and contour_count >= self._floor_model_min_contours
        )
        self._last_obstacle_angle_min = None
        self._last_obstacle_angle_max = None

        if detected and np.count_nonzero(danger_mask) > 0:
            cols = np.where(danger_mask > 0)[1]
            if cols.size > 0:
                width = max(1, danger_mask.shape[1] - 1)
                left_ratio = float(np.min(cols)) / float(width)
                right_ratio = float(np.max(cols)) / float(width)
                angle_span = self._obstacle_scan_mark_angle_max - self._obstacle_scan_mark_angle_min
                self._last_obstacle_angle_min = (
                    self._obstacle_scan_mark_angle_min
                    + left_ratio * angle_span
                    - self._obstacle_scan_padding_angle
                )
                self._last_obstacle_angle_max = (
                    self._obstacle_scan_mark_angle_min
                    + right_ratio * angle_span
                    + self._obstacle_scan_padding_angle
                )

        if self._floor_model_learning and not detected and not edge_detected:
            cv2.accumulateWeighted(features, self._floor_model, self._floor_model_update_alpha)

        return detected, confidence, mask, area_ratio, contour_count, True, floor_diff_stats

    @staticmethod
    def _empty_floor_diff_stats():
        return {
            'l_mean': 0.0,
            'l_max': 0.0,
            'ab_mean': 0.0,
            'ab_max': 0.0,
            'gradient_mean': 0.0,
            'gradient_max': 0.0,
        }

    def _floor_diff_stats(self, diff, danger_x0, danger_x1):
        danger_diff = diff[:, danger_x0:danger_x1, :]
        if danger_diff.size == 0:
            return self._empty_floor_diff_stats()

        l_diff = np.abs(danger_diff[:, :, 0])
        ab_diff = cv2.magnitude(danger_diff[:, :, 1], danger_diff[:, :, 2])
        gradient_diff = np.abs(danger_diff[:, :, 3])
        return {
            'l_mean': float(np.mean(l_diff)),
            'l_max': float(np.max(l_diff)),
            'ab_mean': float(np.mean(ab_diff)),
            'ab_max': float(np.max(ab_diff)),
            'gradient_mean': float(np.mean(gradient_diff)),
            'gradient_max': float(np.max(gradient_diff)),
        }

    def _publish_obstacle_scan_msg(self):
        if not self._publish_obstacle_scan:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._obstacle_scan_frame_id
        scan.angle_min = self._obstacle_scan_angle_min
        scan.angle_max = self._obstacle_scan_angle_max
        scan.angle_increment = self._obstacle_scan_angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self._processing_period
        scan.range_min = self._obstacle_scan_range_min
        scan.range_max = self._obstacle_scan_range_max

        count = int(math.floor((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        scan.ranges = [math.inf] * max(1, count)
        scan.intensities = [0.0] * max(1, count)

        if self._enabled and self._detected and self._last_obstacle_angle_min is not None:
            angle_min = max(scan.angle_min, self._last_obstacle_angle_min)
            angle_max = min(scan.angle_max, self._last_obstacle_angle_max)
            start = max(0, int(math.floor((angle_min - scan.angle_min) / scan.angle_increment)))
            end = min(count - 1, int(math.ceil((angle_max - scan.angle_min) / scan.angle_increment)))
            for index in range(start, end + 1):
                scan.ranges[index] = self._obstacle_scan_distance
                scan.intensities[index] = 1.0

        self._obstacle_scan_pub.publish(scan)

    def _draw_debug(
        self,
        frame,
        x0,
        y0,
        x1,
        y1,
        danger_x0,
        danger_x1,
        detected,
        confidence,
        floor_mask,
        floor_area_ratio,
        floor_contour_count,
        floor_model_ready,
        floor_diff_stats,
    ):
        debug = frame.copy()
        overlay = debug.copy()
        color = (0, 0, 255) if detected else (0, 180, 0)
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (80, 160, 255), -1)
        cv2.addWeighted(overlay, 0.18, debug, 0.82, 0, debug)
        cv2.rectangle(debug, (x0, y0), (x1, y1), (80, 160, 255), 2)

        dx0 = x0 + danger_x0
        dx1 = x0 + danger_x1
        cv2.rectangle(debug, (dx0, y0), (dx1, y1), color, 3)

        if floor_mask is not None and floor_mask.any():
            danger_floor = floor_mask[:, danger_x0:danger_x1]
            colored_mask = np.zeros((danger_floor.shape[0], danger_floor.shape[1], 3), dtype=np.uint8)
            colored_mask[np.where(danger_floor > 0)] = (255, 255, 0)
            target = debug[y0:y1, dx0:dx1]
            cv2.addWeighted(colored_mask, 0.40, target, 1.0, 0, target)

        status = 'LOW OBSTACLE' if self._detected else ('detecting' if detected else 'clear')
        lines = [
            f'{status} conf={confidence:.2f}',
            f'floor={"ready" if floor_model_ready else str(self._floor_model_frames) + "/" + str(self._floor_model_warmup_frames)} '
            f'area={floor_area_ratio:.3f}/{self._floor_model_area_ratio_threshold:.3f} '
            f'fcontours={floor_contour_count}/{self._floor_model_min_contours}',
            f'L diff mean/max={floor_diff_stats["l_mean"]:.1f}/{floor_diff_stats["l_max"]:.1f}',
            f'AB diff mean/max={floor_diff_stats["ab_mean"]:.1f}/{floor_diff_stats["ab_max"]:.1f}',
            f'gradient diff mean/max={floor_diff_stats["gradient_mean"]:.1f}/{floor_diff_stats["gradient_max"]:.1f}',
        ]
        text_y = 28
        for line in lines:
            cv2.putText(debug, line, (12, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(debug, line, (12, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 1, cv2.LINE_AA)
            text_y += 24
        return debug

    def _publish_state(self):
        msg = Bool()
        msg.data = bool(self._enabled and self._detected)
        self._detected_pub.publish(msg)

        conf = Float32()
        conf.data = float(self._confidence if self._enabled else 0.0)
        self._confidence_pub.publish(conf)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLowObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
