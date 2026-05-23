"""CSV trial logger for Project C evaluation metrics."""

import csv
import json
import math
import os
import struct
import time
from datetime import datetime

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String


RESPONSE_STATES = {'OBSERVE', 'DODGE', 'ROTATE', 'BACKUP', 'EMERGENCY'}

EVENT_FIELDS = [
    'trial_id',
    'wall_time_iso',
    'elapsed_s',
    'ros_time_s',
    'event_type',
    'state',
    'detail',
    'front_m',
    'left_m',
    'right_m',
    'rear_m',
    'best_gap_angle_deg',
    'dynamic_obstacle',
    'emergency',
    'dead_end',
    'source',
    'odom_x_m',
    'odom_y_m',
    'odom_yaw_rad',
    'odom_linear_mps',
    'odom_angular_radps',
    'path_length_m',
    'coverage_area_m2',
    'collision_count',
    'dead_end_count',
    'recovery_success_count',
    'dynamic_latency_s',
]

ODOM_FIELDS = [
    'trial_id',
    'wall_time_iso',
    'elapsed_s',
    'ros_time_s',
    'state',
    'x_m',
    'y_m',
    'yaw_rad',
    'linear_mps',
    'angular_radps',
    'path_length_m',
    'coverage_area_m2',
]

OBSTACLE_FIELDS = [
    'trial_id',
    'wall_time_iso',
    'elapsed_s',
    'ros_time_s',
    'state',
    'front_m',
    'left_m',
    'right_m',
    'rear_m',
    'best_gap_angle_deg',
    'dynamic_obstacle',
    'emergency',
    'dead_end',
    'source',
]

SUMMARY_FIELDS = [
    'trial_id',
    'wall_time_iso',
    'elapsed_s',
    'ros_time_s',
    'summary_type',
    'trial_label',
    'collision_count',
    'collision_rate_per_min',
    'dead_end_count',
    'recovery_success_count',
    'recovery_success_rate',
    'dynamic_response_count',
    'mean_dynamic_latency_s',
    'path_length_m',
    'coverage_area_m2',
    'events_csv',
    'odom_csv',
    'obstacles_csv',
    'sensor_fallback_csv',
]

SENSOR_FIELDS = [
    'trial_id',
    'wall_time_iso',
    'elapsed_s',
    'ros_time_s',
    'sensor_set',
    'oak_control',
    'lidar_status',
    'lidar_front_m',
    'lidar_left_m',
    'lidar_right_m',
    'lidar_rear_m',
    'lidar_age_s',
    'tof_status',
    'tof_distance_m',
    'tof_emergency',
    'tof_age_s',
    'depth_status',
    'depth_topic',
    'depth_front_m',
    'depth_samples',
    'depth_age_s',
    'pointcloud_status',
    'pointcloud_topic',
    'pointcloud_front_m',
    'pointcloud_low_front_m',
    'pointcloud_low_obstacle',
    'pointcloud_low_count',
    'pointcloud_points_sampled',
    'pointcloud_age_s',
]


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _finite_or_none(value):
    if value is None:
        return None
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return None
    return value_f if math.isfinite(value_f) else None


def _finite(value) -> float:
    value_f = _finite_or_none(value)
    return value_f if value_f is not None else math.inf


def _fmt_m(value: float) -> str:
    return f'{value:.2f}m' if math.isfinite(value) else 'inf'


def _fmt_age(value: float | None) -> str:
    return f'{value:.2f}s' if value is not None and math.isfinite(value) else 'na'


def _fmt_float(value, precision: int = 3) -> str:
    value_f = _finite_or_none(value)
    if value_f is None:
        return ''
    return f'{value_f:.{precision}f}'


def _fmt_bool(value) -> str:
    return '1' if bool(value) else '0'


def _default_trial_id() -> str:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    return f'project_c_trial_{stamp}'


def _safe_trial_id(value) -> str:
    text = str(value or '').strip()
    if not text:
        return _default_trial_id()
    safe = ''.join(
        char if char.isalnum() or char in ('_', '-', '.') else '_'
        for char in text
    ).strip('._')
    return safe or _default_trial_id()


def _min_finite(values) -> float:
    finite = [_finite(value) for value in values]
    finite = [value for value in finite if math.isfinite(value)]
    return min(finite) if finite else math.inf


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObstacleTrialLoggerNode(Node):

    def __init__(self):
        super().__init__('obstacle_trial_logger')

        self.declare_parameter('obstacle_topic', '/obstacle_representation')
        self.declare_parameter('state_topic', '/obstacle_avoidance_state')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('collision_topic', '/collision_event')
        self.declare_parameter('summary_topic', '/obstacle_trial_summary')
        self.declare_parameter('log_dir', 'record')
        self.declare_parameter('trial_id', '')
        self.declare_parameter('trial_label', '')
        self.declare_parameter('summary_period_sec', 5.0)
        self.declare_parameter('odom_log_period_sec', 0.2)
        self.declare_parameter('log_obstacle_samples', False)
        self.declare_parameter('obstacle_sample_period_sec', 0.2)
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('pointcloud_topic', '/oak/points')
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_tof', True)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('use_pointcloud', True)
        self.declare_parameter('control_use_depth', True)
        self.declare_parameter('control_use_pointcloud', True)
        self.declare_parameter('depth_qos', 'auto')
        self.declare_parameter('pointcloud_qos', 'auto')
        self.declare_parameter('sensor_csv_enabled', True)
        self.declare_parameter('sensor_log_period_sec', 1.0)
        self.declare_parameter('sensor_stale_sec', 1.0)
        self.declare_parameter('sensor_set_label', '')
        self.declare_parameter('tof_emergency_distance', 0.12)
        self.declare_parameter('depth_roi_width_fraction', 0.25)
        self.declare_parameter('depth_roi_height_fraction', 0.25)
        self.declare_parameter('depth_sample_stride_px', 4)
        self.declare_parameter('depth_process_hz', 5.0)
        self.declare_parameter('depth_percentile', 10.0)
        self.declare_parameter('depth_min_valid_m', 0.05)
        self.declare_parameter('depth_max_valid_m', 10.0)
        self.declare_parameter('pointcloud_frame', 'optical')
        self.declare_parameter('pointcloud_max_points', 500)
        self.declare_parameter('pointcloud_process_hz', 2.0)
        self.declare_parameter('pointcloud_front_half_width_m', 0.35)
        self.declare_parameter('pointcloud_low_center_half_width_m', 0.32)
        self.declare_parameter('pointcloud_low_min_height_m', -0.90)
        self.declare_parameter('pointcloud_low_max_height_m', 0.70)
        self.declare_parameter('pointcloud_min_forward_m', 0.02)
        self.declare_parameter('pointcloud_max_forward_m', 4.00)
        self.declare_parameter('pointcloud_low_obstacle_distance_m', 0.60)

        p = self.get_parameter
        obstacle_topic = p('obstacle_topic').value
        state_topic = p('state_topic').value
        odom_topic = p('odom_topic').value
        collision_topic = p('collision_topic').value
        summary_topic = p('summary_topic').value
        summary_period = float(p('summary_period_sec').value)
        self._odom_log_period = max(
            0.0,
            float(p('odom_log_period_sec').value),
        )
        self._log_obstacle_samples = _as_bool(
            p('log_obstacle_samples').value
        )
        self._obstacle_sample_period = max(
            0.0,
            float(p('obstacle_sample_period_sec').value),
        )
        self._trial_label = str(p('trial_label').value or '')
        self._depth_topic = str(p('depth_topic').value)
        self._pointcloud_topic = str(p('pointcloud_topic').value)
        self._use_lidar = _as_bool(p('use_lidar').value)
        self._use_tof = _as_bool(p('use_tof').value)
        self._use_depth = _as_bool(p('use_depth').value)
        self._use_pointcloud = _as_bool(p('use_pointcloud').value)
        self._control_use_depth = _as_bool(p('control_use_depth').value)
        self._control_use_pointcloud = _as_bool(p('control_use_pointcloud').value)
        self._control_uses_oak = self._control_use_depth or self._control_use_pointcloud
        self._depth_qos = str(p('depth_qos').value).strip().lower()
        self._pointcloud_qos = str(p('pointcloud_qos').value).strip().lower()
        self._sensor_csv_enabled = _as_bool(p('sensor_csv_enabled').value)
        self._sensor_log_period = max(0.1, float(p('sensor_log_period_sec').value))
        self._stale_sec = max(0.1, float(p('sensor_stale_sec').value))
        self._sensor_set_label = str(p('sensor_set_label').value).strip()
        if not self._sensor_set_label:
            self._sensor_set_label = self._default_sensor_set_label()
        self._tof_emergency_distance = max(0.0, float(p('tof_emergency_distance').value))
        self._depth_roi_width = max(
            0.01,
            min(1.0, float(p('depth_roi_width_fraction').value)),
        )
        self._depth_roi_height = max(
            0.01,
            min(1.0, float(p('depth_roi_height_fraction').value)),
        )
        self._depth_stride = max(1, int(p('depth_sample_stride_px').value))
        self._depth_process_period = 1.0 / max(0.1, float(p('depth_process_hz').value))
        self._depth_percentile = max(
            0.0,
            min(100.0, float(p('depth_percentile').value)),
        )
        self._depth_min_valid = max(0.0, float(p('depth_min_valid_m').value))
        self._depth_max_valid = max(
            self._depth_min_valid,
            float(p('depth_max_valid_m').value),
        )
        self._pointcloud_frame = str(p('pointcloud_frame').value).strip().lower()
        self._pointcloud_max_points = max(1, int(p('pointcloud_max_points').value))
        self._pointcloud_process_period = 1.0 / max(
            0.1,
            float(p('pointcloud_process_hz').value),
        )
        self._pointcloud_front_half_width = max(
            0.0,
            float(p('pointcloud_front_half_width_m').value),
        )
        self._pointcloud_low_half_width = max(
            0.0,
            float(p('pointcloud_low_center_half_width_m').value),
        )
        self._pointcloud_low_min_height = float(p('pointcloud_low_min_height_m').value)
        self._pointcloud_low_max_height = float(p('pointcloud_low_max_height_m').value)
        self._pointcloud_min_forward = float(p('pointcloud_min_forward_m').value)
        self._pointcloud_max_forward = float(p('pointcloud_max_forward_m').value)
        self._pointcloud_low_obstacle_distance = max(
            0.0,
            float(p('pointcloud_low_obstacle_distance_m').value),
        )

        self._trial_id = _safe_trial_id(p('trial_id').value)
        log_root = os.path.expanduser(str(p('log_dir').value))
        self._trial_dir = os.path.join(log_root, self._trial_id)
        os.makedirs(self._trial_dir, exist_ok=True)

        self._events_path = os.path.join(self._trial_dir, 'events.csv')
        self._odom_path = os.path.join(self._trial_dir, 'odom.csv')
        self._summary_path = os.path.join(self._trial_dir, 'summary.csv')
        self._obstacles_path = os.path.join(self._trial_dir, 'obstacles.csv')
        self._sensor_fallback_path = os.path.join(
            self._trial_dir,
            'sensor_fallback.csv',
        )

        self._csv_files = []
        self._events_file, self._events_writer = self._open_csv(
            self._events_path,
            EVENT_FIELDS,
        )
        self._odom_file, self._odom_writer = self._open_csv(
            self._odom_path,
            ODOM_FIELDS,
        )
        self._summary_file, self._summary_writer = self._open_csv(
            self._summary_path,
            SUMMARY_FIELDS,
        )
        self._obstacles_file = None
        self._obstacles_writer = None
        if self._log_obstacle_samples:
            self._obstacles_file, self._obstacles_writer = self._open_csv(
                self._obstacles_path,
                OBSTACLE_FIELDS,
            )
        self._sensor_csv_file = None
        self._sensor_csv_writer = None
        if self._sensor_csv_enabled:
            self._sensor_csv_file, self._sensor_csv_writer = self._open_csv(
                self._sensor_fallback_path,
                SENSOR_FIELDS,
            )

        self._state = 'UNKNOWN'
        self._latest_report = {}
        self._latest_fused = {}
        self._latest_source = []
        self._latest_odom = {}
        self._trial_start_time = time.monotonic()
        self._collision_count = 0
        self._dead_end_count = 0
        self._recovery_success_count = 0
        self._recovery_active = False
        self._dynamic_active = False
        self._dynamic_seen_time: float | None = None
        self._emergency_active = False
        self._latencies: list[float] = []
        self._prev_odom_xy: tuple[float, float] | None = None
        self._path_length = 0.0
        self._min_x: float | None = None
        self._max_x: float | None = None
        self._min_y: float | None = None
        self._max_y: float | None = None
        self._last_odom_log = 0.0
        self._last_obstacle_log = 0.0
        self._last_report_time: float | None = None
        self._last_bad_report_warn = 0.0
        self._depth_front = math.inf
        self._depth_samples = 0
        self._last_depth_time: float | None = None
        self._last_depth_process_time = 0.0
        self._last_depth_warn = 0.0
        self._pointcloud_front = math.inf
        self._pointcloud_low_front = math.inf
        self._pointcloud_low_count = 0
        self._pointcloud_points_sampled = 0
        self._last_pointcloud_time: float | None = None
        self._last_pointcloud_process_time = 0.0
        self._last_pointcloud_warn = 0.0
        self._closed = False

        self.create_subscription(String, obstacle_topic, self._on_obstacles, 10)
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(String, collision_topic, self._on_collision, 10)
        if self._use_depth:
            self.create_subscription(
                Image,
                self._depth_topic,
                self._on_depth,
                self._depth_qos_profile(),
            )
        if self._use_pointcloud:
            self.create_subscription(
                PointCloud2,
                self._pointcloud_topic,
                self._on_pointcloud,
                self._pointcloud_qos_profile(),
            )
        self._summary_pub = self.create_publisher(String, summary_topic, 10)
        self.create_timer(max(summary_period, 1.0), self._publish_summary)
        self.create_timer(self._sensor_log_period, self._log_sensors)

        self._write_event('TRIAL_START', detail=self._trial_label)
        self._write_summary('start')
        self.get_logger().info(
            f'Project C trial logger writing to folder: {self._trial_dir}; '
            f'sensors={self._sensor_set_label}; '
            f'oak_control={"yes" if self._control_uses_oak else "no"}'
        )

    def _open_csv(self, path: str, fieldnames: list[str]):
        csv_file = open(path, 'w', newline='', encoding='utf-8')
        self._csv_files.append(csv_file)
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        csv_file.flush()
        return csv_file, writer

    def _default_sensor_set_label(self) -> str:
        sensors = []
        if self._use_lidar:
            sensors.append('lidar')
        if self._use_tof:
            sensors.append('tof')
        if self._use_depth:
            sensors.append('depth')
        if self._use_pointcloud:
            sensors.append('pointcloud')
        return '+'.join(sensors) if sensors else 'none'

    def _depth_qos_profile(self) -> QoSProfile:
        if self._depth_qos in ('reliable', 'reliable_volatile'):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=2,
            )
        if self._depth_qos in (
                'auto',
                'sensor_data',
                'best_effort',
                'best-effort',
                ''):
            return _sensor_qos()
        self.get_logger().warn(
            f'Unknown depth_qos={self._depth_qos!r}; using BEST_EFFORT depth=1.'
        )
        return _sensor_qos()

    def _pointcloud_qos_profile(self) -> QoSProfile:
        if self._pointcloud_qos in ('reliable', 'reliable_volatile'):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        if self._pointcloud_qos in (
                'auto',
                'sensor_data',
                'best_effort',
                'best-effort',
                ''):
            return _sensor_qos()
        self.get_logger().warn(
            f'Unknown pointcloud_qos={self._pointcloud_qos!r}; using BEST_EFFORT depth=1.'
        )
        return _sensor_qos()

    def _on_obstacles(self, msg: String):
        now = time.monotonic()
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            if now - self._last_bad_report_warn > 2.0:
                self._last_bad_report_warn = now
                self.get_logger().warn('Invalid obstacle JSON; trial logger kept last sample.')
            return

        self._latest_report = data if isinstance(data, dict) else {}
        self._last_report_time = now
        fused = data.get('fused', {})
        self._latest_fused = fused if isinstance(fused, dict) else {}
        source = self._latest_fused.get('source', [])
        self._latest_source = source if isinstance(source, list) else [source]

        dynamic = bool(self._latest_fused.get('dynamic_obstacle', False))
        dead_end = bool(self._latest_fused.get('dead_end', False))
        emergency = bool(self._latest_fused.get('emergency', False))

        if dynamic and not self._dynamic_active:
            self._dynamic_active = True
            self._dynamic_seen_time = now
            self._write_event('DYNAMIC_SEEN')
        elif not dynamic:
            self._dynamic_active = False
            if (
                    self._dynamic_seen_time is not None
                    and now - self._dynamic_seen_time > 3.0):
                self._dynamic_seen_time = None

        if dead_end and not self._recovery_active:
            self._dead_end_count += 1
            self._recovery_active = True
            self._write_event('DEAD_END')

        if emergency and not self._emergency_active:
            self._emergency_active = True
            self._write_event('EMERGENCY')
        elif not emergency:
            self._emergency_active = False

        if self._log_obstacle_samples and self._should_log(
                now,
                self._last_obstacle_log,
                self._obstacle_sample_period):
            self._last_obstacle_log = now
            self._write_obstacle_sample()

    def _on_state(self, msg: String):
        parts = msg.data.split(',', 1)
        state = parts[1] if len(parts) == 2 else msg.data
        state = state.strip() or 'UNKNOWN'
        self._state = state

        latency = None
        if (
                self._dynamic_seen_time is not None
                and state in RESPONSE_STATES):
            latency = time.monotonic() - self._dynamic_seen_time
            self._latencies.append(latency)
            self._dynamic_seen_time = None

        if self._recovery_active and state == 'DRIVE':
            self._recovery_success_count += 1
            self._recovery_active = False

        self._write_event('STATE', detail=state, dynamic_latency=latency)

    def _on_collision(self, msg: String):
        self._collision_count += 1
        self._write_event('COLLISION', detail=msg.data)

    def _on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = _yaw_from_odom(msg)
        linear = math.hypot(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        angular = float(msg.twist.twist.angular.z)

        if self._prev_odom_xy is not None:
            px, py = self._prev_odom_xy
            self._path_length += math.hypot(x - px, y - py)
        self._prev_odom_xy = (x, y)

        self._min_x = x if self._min_x is None else min(self._min_x, x)
        self._max_x = x if self._max_x is None else max(self._max_x, x)
        self._min_y = y if self._min_y is None else min(self._min_y, y)
        self._max_y = y if self._max_y is None else max(self._max_y, y)

        self._latest_odom = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'linear': linear,
            'angular': angular,
        }

        now = time.monotonic()
        if self._should_log(now, self._last_odom_log, self._odom_log_period):
            self._last_odom_log = now
            self._write_odom_sample()

    def _on_depth(self, msg: Image):
        now = time.monotonic()
        self._last_depth_time = now
        if now - self._last_depth_process_time < self._depth_process_period:
            return
        self._last_depth_process_time = now

        try:
            self._depth_front, self._depth_samples = self._depth_front_from_msg(msg)
        except (ValueError, TypeError, struct.error) as exc:
            self._depth_front = math.inf
            self._depth_samples = 0
            if now - self._last_depth_warn > 2.0:
                self._last_depth_warn = now
                self.get_logger().warn(f'Depth sample failed: {exc}')

    def _depth_front_from_msg(self, msg: Image) -> tuple[float, int]:
        encoding = msg.encoding.strip().lower()
        if encoding in ('16uc1', 'mono16'):
            dtype = np.uint16
            scale = 0.001
        elif encoding in ('32fc1',):
            dtype = np.float32
            scale = 1.0
        else:
            raise ValueError(f'unsupported depth encoding {msg.encoding!r}')

        item_size = np.dtype(dtype).itemsize
        if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * item_size:
            raise ValueError('invalid depth image dimensions')

        row_items = msg.step // item_size
        expected_items = row_items * msg.height
        data = np.frombuffer(msg.data, dtype=dtype, count=expected_items)
        if data.size < expected_items:
            raise ValueError('depth image data is shorter than expected')

        image = data.reshape((msg.height, row_items))[:, :msg.width]
        roi_width = max(1, int(msg.width * self._depth_roi_width))
        roi_height = max(1, int(msg.height * self._depth_roi_height))
        x0 = max(0, (msg.width - roi_width) // 2)
        y0 = max(0, (msg.height - roi_height) // 2)
        x1 = min(msg.width, x0 + roi_width)
        y1 = min(msg.height, y0 + roi_height)

        roi = image[y0:y1:self._depth_stride, x0:x1:self._depth_stride]
        values = roi.astype(np.float32, copy=False) * scale
        valid = values[
            np.isfinite(values)
            & (values >= self._depth_min_valid)
            & (values <= self._depth_max_valid)
        ]
        if valid.size == 0:
            return math.inf, 0
        return float(np.percentile(valid, self._depth_percentile)), int(valid.size)

    def _on_pointcloud(self, msg: PointCloud2):
        now = time.monotonic()
        self._last_pointcloud_time = now
        if now - self._last_pointcloud_process_time < self._pointcloud_process_period:
            return
        self._last_pointcloud_process_time = now

        try:
            self._sample_pointcloud(msg)
        except (ValueError, struct.error) as exc:
            self._pointcloud_front = math.inf
            self._pointcloud_low_front = math.inf
            self._pointcloud_low_count = 0
            self._pointcloud_points_sampled = 0
            if now - self._last_pointcloud_warn > 2.0:
                self._last_pointcloud_warn = now
                self.get_logger().warn(f'PointCloud sample failed: {exc}')

    def _sample_pointcloud(self, msg: PointCloud2):
        fields = {field.name: field for field in msg.fields}
        try:
            x_field = fields['x']
            y_field = fields['y']
            z_field = fields['z']
        except KeyError as exc:
            raise ValueError(f'missing PointCloud2 field {exc.args[0]!r}') from exc

        for field in (x_field, y_field, z_field):
            if field.datatype != PointField.FLOAT32:
                raise ValueError('PointCloud2 x/y/z must be FLOAT32')

        if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
            raise ValueError('invalid PointCloud2 dimensions')

        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0:
            raise ValueError('empty PointCloud2')

        sample_step = max(1, total_points // self._pointcloud_max_points)
        endian = '>' if msg.is_bigendian else '<'
        unpack_float = struct.Struct(f'{endian}f').unpack_from
        max_offset = max(x_field.offset, y_field.offset, z_field.offset) + 4

        front_min = math.inf
        low_front_min = math.inf
        low_count = 0
        sampled = 0
        data = msg.data

        for index in range(0, total_points, sample_step):
            row = index // msg.width
            col = index % msg.width
            offset = row * msg.row_step + col * msg.point_step
            if offset + max_offset > len(data):
                continue

            x = unpack_float(data, offset + x_field.offset)[0]
            y = unpack_float(data, offset + y_field.offset)[0]
            z = unpack_float(data, offset + z_field.offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            sampled += 1
            forward, lateral, height = self._point_axes(x, y, z)
            if (
                    self._pointcloud_min_forward <= forward <= self._pointcloud_max_forward
                    and abs(lateral) <= self._pointcloud_front_half_width):
                front_min = min(front_min, forward)

            if (
                    self._pointcloud_min_forward <= forward <= self._pointcloud_max_forward
                    and abs(lateral) <= self._pointcloud_low_half_width
                    and self._pointcloud_low_min_height <= height <= self._pointcloud_low_max_height):
                low_count += 1
                low_front_min = min(low_front_min, forward)

            if sampled >= self._pointcloud_max_points:
                break

        self._pointcloud_front = front_min
        self._pointcloud_low_front = low_front_min
        self._pointcloud_low_count = low_count
        self._pointcloud_points_sampled = sampled

    def _point_axes(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        if 'optical' in self._pointcloud_frame or 'camera' in self._pointcloud_frame:
            return z, -x, -y
        return x, y, z

    def _publish_summary(self):
        msg = String()
        msg.data = json.dumps(self._summary(), separators=(',', ':'))
        self._summary_pub.publish(msg)
        self._write_summary('periodic')

    def _log_sensors(self):
        now = time.monotonic()
        lidar = self._latest_report.get('lidar', {}) if isinstance(self._latest_report, dict) else {}
        tof = self._latest_report.get('tof', {}) if isinstance(self._latest_report, dict) else {}

        lidar_age = self._report_age('scan', now)
        tof_age = self._report_age('tof', now)
        depth_age = self._topic_age(self._last_depth_time, now)
        pointcloud_age = self._topic_age(self._last_pointcloud_time, now)

        lidar_status = self._report_status(lidar, lidar_age)
        tof_status = self._report_status(tof, tof_age)
        if tof_status == 'missing' and tof_age is not None:
            tof_status = 'clear'
        depth_status = self._topic_status(self._use_depth, self._last_depth_time, now)
        pointcloud_status = self._topic_status(
            self._use_pointcloud,
            self._last_pointcloud_time,
            now,
        )

        lidar_front = _finite(lidar.get('front_control', lidar.get('front_min')))
        lidar_left = _finite(lidar.get('left_body_clearance', lidar.get('left_mean')))
        lidar_right = _finite(lidar.get('right_body_clearance', lidar.get('right_mean')))
        lidar_rear = _finite(lidar.get('rear_control', lidar.get('rear_mean')))

        tof_dist = _finite(tof.get('range'))
        if not math.isfinite(tof_dist):
            topics = tof.get('topics', {}) if isinstance(tof, dict) else {}
            if isinstance(topics, dict):
                tof_dist = _min_finite(topics.values())
        tof_emergency = (
            math.isfinite(tof_dist)
            and tof_dist <= self._tof_emergency_distance
        )

        pc_low = (
            math.isfinite(self._pointcloud_low_front)
            and self._pointcloud_low_count > 0
            and self._pointcloud_low_front <= self._pointcloud_low_obstacle_distance
        )

        line = (
            f'[SENSOR_LOG] sensors={self._sensor_set_label} '
            f'oak_control={"yes" if self._control_uses_oak else "no"} | '
            f'lidar={lidar_status} front={_fmt_m(lidar_front)} '
            f'left={_fmt_m(lidar_left)} right={_fmt_m(lidar_right)} '
            f'rear={_fmt_m(lidar_rear)} age={_fmt_age(lidar_age)} | '
            f'tof={tof_status} dist={_fmt_m(tof_dist)} '
            f'emergency={"yes" if tof_emergency else "no"} age={_fmt_age(tof_age)} | '
            f'depth={depth_status} topic={self._depth_topic} '
            f'front={_fmt_m(self._depth_front)} age={_fmt_age(depth_age)} | '
            f'pointcloud={pointcloud_status} topic={self._pointcloud_topic} '
            f'pc_front={_fmt_m(self._pointcloud_front)} '
            f'pc_low={"yes" if pc_low else "no"} '
            f'points_sampled={self._pointcloud_points_sampled} '
            f'age={_fmt_age(pointcloud_age)}'
        )
        self.get_logger().info(line)
        self._write_sensor_row(
            lidar_status=lidar_status,
            lidar_front=lidar_front,
            lidar_left=lidar_left,
            lidar_right=lidar_right,
            lidar_rear=lidar_rear,
            lidar_age=lidar_age,
            tof_status=tof_status,
            tof_dist=tof_dist,
            tof_emergency=tof_emergency,
            tof_age=tof_age,
            depth_status=depth_status,
            depth_age=depth_age,
            pointcloud_status=pointcloud_status,
            pointcloud_age=pointcloud_age,
            pc_low=pc_low,
        )

    def _write_sensor_row(
            self,
            *,
            lidar_status: str,
            lidar_front: float,
            lidar_left: float,
            lidar_right: float,
            lidar_rear: float,
            lidar_age: float | None,
            tof_status: str,
            tof_dist: float,
            tof_emergency: bool,
            tof_age: float | None,
            depth_status: str,
            depth_age: float | None,
            pointcloud_status: str,
            pointcloud_age: float | None,
            pc_low: bool):
        if self._sensor_csv_writer is None or self._sensor_csv_file is None:
            return

        row = {
            'trial_id': self._trial_id,
            'wall_time_iso': datetime.now().astimezone().isoformat(
                timespec='milliseconds'
            ),
            'elapsed_s': _fmt_float(time.monotonic() - self._trial_start_time),
            'ros_time_s': _fmt_float(
                self.get_clock().now().nanoseconds / 1_000_000_000.0
            ),
            'sensor_set': self._sensor_set_label,
            'oak_control': _fmt_bool(self._control_uses_oak),
            'lidar_status': lidar_status,
            'lidar_front_m': _fmt_float(lidar_front),
            'lidar_left_m': _fmt_float(lidar_left),
            'lidar_right_m': _fmt_float(lidar_right),
            'lidar_rear_m': _fmt_float(lidar_rear),
            'lidar_age_s': _fmt_float(lidar_age),
            'tof_status': tof_status,
            'tof_distance_m': _fmt_float(tof_dist),
            'tof_emergency': _fmt_bool(tof_emergency),
            'tof_age_s': _fmt_float(tof_age),
            'depth_status': depth_status,
            'depth_topic': self._depth_topic,
            'depth_front_m': _fmt_float(self._depth_front),
            'depth_samples': self._depth_samples,
            'depth_age_s': _fmt_float(depth_age),
            'pointcloud_status': pointcloud_status,
            'pointcloud_topic': self._pointcloud_topic,
            'pointcloud_front_m': _fmt_float(self._pointcloud_front),
            'pointcloud_low_front_m': _fmt_float(self._pointcloud_low_front),
            'pointcloud_low_obstacle': _fmt_bool(pc_low),
            'pointcloud_low_count': self._pointcloud_low_count,
            'pointcloud_points_sampled': self._pointcloud_points_sampled,
            'pointcloud_age_s': _fmt_float(pointcloud_age),
        }
        self._sensor_csv_writer.writerow(row)
        self._sensor_csv_file.flush()

    def _report_age(self, key: str, now: float) -> float | None:
        if self._last_report_time is None:
            return None
        ages = self._latest_report.get('ages', {}) if isinstance(self._latest_report, dict) else {}
        age = ages.get(key)
        if age is None:
            return None
        try:
            return max(0.0, float(age)) + max(0.0, now - self._last_report_time)
        except (TypeError, ValueError):
            return None

    def _report_status(self, section: dict, age: float | None) -> str:
        if self._last_report_time is None or age is None:
            return 'missing'
        if age > self._stale_sec:
            return 'stale'
        if not bool(section.get('available', False)):
            return 'missing'
        return 'ok'

    @staticmethod
    def _topic_age(stamp: float | None, now: float) -> float | None:
        if stamp is None:
            return None
        return max(0.0, now - stamp)

    def _topic_status(self, enabled: bool, stamp: float | None, now: float) -> str:
        if not enabled:
            return 'disabled'
        if stamp is None:
            return 'missing'
        if now - stamp > self._stale_sec:
            return 'stale'
        return 'ok'

    def _summary(self) -> dict:
        elapsed_sec = self._elapsed()
        elapsed_min = max(elapsed_sec / 60.0, 1e-6)
        recovery_rate = (
            self._recovery_success_count / self._dead_end_count
            if self._dead_end_count
            else None
        )
        mean_latency = (
            sum(self._latencies) / len(self._latencies)
            if self._latencies
            else None
        )
        return {
            'trial_id': self._trial_id,
            'trial_label': self._trial_label,
            'elapsed_sec': elapsed_sec,
            'collision_count': self._collision_count,
            'collision_rate_per_min': self._collision_count / elapsed_min,
            'dead_end_count': self._dead_end_count,
            'recovery_success_count': self._recovery_success_count,
            'recovery_success_rate': recovery_rate,
            'dynamic_response_count': len(self._latencies),
            'mean_dynamic_latency_s': mean_latency,
            'path_length_m': self._path_length,
            'coverage_area_m2': self._coverage_area(),
            'trial_dir': self._trial_dir,
            'events_csv': self._events_path,
            'odom_csv': self._odom_path,
            'summary_csv': self._summary_path,
            'obstacles_csv': (
                self._obstacles_path if self._log_obstacle_samples else ''
            ),
            'sensor_fallback_csv': (
                self._sensor_fallback_path if self._sensor_csv_enabled else ''
            ),
        }

    def _coverage_area(self) -> float:
        if None in (self._min_x, self._max_x, self._min_y, self._max_y):
            return 0.0
        return max(0.0, (self._max_x - self._min_x) * (self._max_y - self._min_y))

    def _write_event(
            self,
            event_type: str,
            detail: str = '',
            dynamic_latency=None):
        row = {
            **self._common_row(),
            'event_type': event_type,
            'state': self._state,
            'detail': detail,
            **self._obstacle_row(),
            **self._odom_snapshot_row(prefix='odom_'),
            'path_length_m': _fmt_float(self._path_length),
            'coverage_area_m2': _fmt_float(self._coverage_area()),
            'collision_count': self._collision_count,
            'dead_end_count': self._dead_end_count,
            'recovery_success_count': self._recovery_success_count,
            'dynamic_latency_s': _fmt_float(dynamic_latency),
        }
        self._events_writer.writerow(row)
        self._events_file.flush()

    def _write_odom_sample(self):
        row = {
            **self._common_row(),
            'state': self._state,
            'x_m': _fmt_float(self._latest_odom.get('x')),
            'y_m': _fmt_float(self._latest_odom.get('y')),
            'yaw_rad': _fmt_float(self._latest_odom.get('yaw')),
            'linear_mps': _fmt_float(self._latest_odom.get('linear')),
            'angular_radps': _fmt_float(self._latest_odom.get('angular')),
            'path_length_m': _fmt_float(self._path_length),
            'coverage_area_m2': _fmt_float(self._coverage_area()),
        }
        self._odom_writer.writerow(row)
        self._odom_file.flush()

    def _write_obstacle_sample(self):
        if self._obstacles_writer is None or self._obstacles_file is None:
            return

        row = {
            **self._common_row(),
            'state': self._state,
            **self._obstacle_row(),
        }
        self._obstacles_writer.writerow(row)
        self._obstacles_file.flush()

    def _write_summary(self, summary_type: str):
        summary = self._summary()
        row = {
            **self._common_row(),
            'summary_type': summary_type,
            'trial_label': self._trial_label,
            'collision_count': self._collision_count,
            'collision_rate_per_min': _fmt_float(
                summary['collision_rate_per_min']
            ),
            'dead_end_count': self._dead_end_count,
            'recovery_success_count': self._recovery_success_count,
            'recovery_success_rate': _fmt_float(
                summary['recovery_success_rate']
            ),
            'dynamic_response_count': summary['dynamic_response_count'],
            'mean_dynamic_latency_s': _fmt_float(
                summary['mean_dynamic_latency_s']
            ),
            'path_length_m': _fmt_float(self._path_length),
            'coverage_area_m2': _fmt_float(self._coverage_area()),
            'events_csv': self._events_path,
            'odom_csv': self._odom_path,
            'obstacles_csv': (
                self._obstacles_path if self._log_obstacle_samples else ''
            ),
            'sensor_fallback_csv': (
                self._sensor_fallback_path if self._sensor_csv_enabled else ''
            ),
        }
        self._summary_writer.writerow(row)
        self._summary_file.flush()

    def _common_row(self) -> dict:
        return {
            'trial_id': self._trial_id,
            'wall_time_iso': datetime.now().astimezone().isoformat(
                timespec='milliseconds'
            ),
            'elapsed_s': _fmt_float(self._elapsed()),
            'ros_time_s': _fmt_float(
                self.get_clock().now().nanoseconds / 1_000_000_000.0
            ),
        }

    def _obstacle_row(self) -> dict:
        fused = self._latest_fused
        return {
            'front_m': _fmt_float(fused.get('front_distance')),
            'left_m': _fmt_float(fused.get('left_distance')),
            'right_m': _fmt_float(fused.get('right_distance')),
            'rear_m': _fmt_float(fused.get('rear_distance')),
            'best_gap_angle_deg': _fmt_float(fused.get('best_gap_angle')),
            'dynamic_obstacle': _fmt_bool(fused.get('dynamic_obstacle', False)),
            'emergency': _fmt_bool(fused.get('emergency', False)),
            'dead_end': _fmt_bool(fused.get('dead_end', False)),
            'source': '+'.join(str(item) for item in self._latest_source if item),
        }

    def _odom_snapshot_row(self, prefix: str = '') -> dict:
        return {
            f'{prefix}x_m': _fmt_float(self._latest_odom.get('x')),
            f'{prefix}y_m': _fmt_float(self._latest_odom.get('y')),
            f'{prefix}yaw_rad': _fmt_float(self._latest_odom.get('yaw')),
            f'{prefix}linear_mps': _fmt_float(self._latest_odom.get('linear')),
            f'{prefix}angular_radps': _fmt_float(self._latest_odom.get('angular')),
        }

    def _elapsed(self) -> float:
        return time.monotonic() - self._trial_start_time

    @staticmethod
    def _should_log(now: float, previous: float, period: float) -> bool:
        return previous <= 0.0 or period <= 0.0 or now - previous >= period

    def destroy_node(self):
        try:
            if not self._closed:
                self._closed = True
                self._write_event('TRIAL_END')
                self._write_summary('final')
        finally:
            for csv_file in self._csv_files:
                csv_file.close()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTrialLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
