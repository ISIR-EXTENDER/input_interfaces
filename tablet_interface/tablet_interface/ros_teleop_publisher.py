from __future__ import annotations

import json
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from extender_msgs.msg import TeleopCommand

from tablet_interface.petanque_measurements import (
    CameraIntrinsics,
    CircleDetectionConfig,
    MeasurementConfig,
    PetanqueMeasurements,
)
from tablet_interface.teleop_mapping import map_and_scale, normalize_mapping

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore


class TabletInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tablet_interface_node")

        self.declare_parameter("teleop_cmd_topic", "/teleop_cmd")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("linear_scale", 0.2)
        self.declare_parameter("angular_scale", 0.5)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("linear_axes", [0, 1, 2])
        self.declare_parameter("linear_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("angular_axes", [0, 1, 2])
        self.declare_parameter("angular_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("default_mode", 0)
        self.declare_parameter("accept_mode_from_client", True)
        self.declare_parameter("state_publish_hz", 5.0)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8765)
        self.declare_parameter("ws_path", "/ws/control")
        self.declare_parameter(
            "state_machine_topic", "/petanque_state_machine/change_state"
        )
        self.declare_parameter("gripper_topic", "/gripper_controller/commands")
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", 1.05)
        self.declare_parameter("hub_digital_output_topic", "/hub/digital_output")
        self.declare_parameter("hub_electromagnet_channel", 2.0)
        self.declare_parameter("petanque_param_service", "/petanque_throw/set_parameters")
        self.declare_parameter("petanque_total_duration_param", "total_duration")
        self.declare_parameter(
            "petanque_angle_between_start_and_finish_param",
            "angle_between_start_and_finish",
        )
        self.declare_parameter("param_call_timeout_sec", 1.5)
        self.declare_parameter("petanque_measurements_enabled", False)
        self.declare_parameter(
            "petanque_measurement_request_image_topic",
            "/petanque/measure/request_image/compressed",
        )
        self.declare_parameter("petanque_measurement_points_topic", "/petanque_measurements/points")
        self.declare_parameter(
            "petanque_measurement_result_image_topic",
            "/petanque/measure/result_image/compressed",
        )
        self.declare_parameter(
            "petanque_measurement_result_vectors_topic",
            "/petanque/measure/result_vectors",
        )
        self.declare_parameter("petanque_sphere_diameter_m", 0.073)
        self.declare_parameter("petanque_click_to_circle_threshold_px", 525.0)
        self.declare_parameter("petanque_click_search_margin_px", 180.0)
        self.declare_parameter("petanque_max_candidate_spheres", 20)
        self.declare_parameter("petanque_intrinsics_mode", "image")
        self.declare_parameter("petanque_assumed_hfov_deg", 69.0)
        self.declare_parameter("petanque_camera_fx", 600.0)
        self.declare_parameter("petanque_camera_fy", 600.0)
        self.declare_parameter("petanque_camera_cx", 320.0)
        self.declare_parameter("petanque_camera_cy", 240.0)
        self.declare_parameter("petanque_blur_kernel_size", 5)
        self.declare_parameter("petanque_hough_dp", 1.)
        self.declare_parameter("petanque_hough_min_dist_px", 150.0)
        self.declare_parameter("petanque_hough_param1", 80.0)
        self.declare_parameter("petanque_hough_param2", 40.0)
        self.declare_parameter("petanque_min_radius_px", 30)
        self.declare_parameter("petanque_max_radius_px", 200)

        self.teleop_cmd_topic = self.get_parameter("teleop_cmd_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.swap_xy = bool(self.get_parameter("swap_xy").value)
        linear_axes_param = list(self.get_parameter("linear_axes").value)
        linear_signs_param = list(self.get_parameter("linear_signs").value)
        angular_axes_param = list(self.get_parameter("angular_axes").value)
        angular_signs_param = list(self.get_parameter("angular_signs").value)
        self.default_mode = int(self.get_parameter("default_mode").value)
        self.accept_mode_from_client = bool(self.get_parameter("accept_mode_from_client").value)
        self.state_publish_hz = float(self.get_parameter("state_publish_hz").value)
        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.ws_path = str(self.get_parameter("ws_path").value)
        self.state_machine_topic = str(self.get_parameter("state_machine_topic").value)
        self.gripper_topic = str(self.get_parameter("gripper_topic").value)
        self.gripper_open_position = float(
            self.get_parameter("gripper_open_position").value
        )
        self.gripper_close_position = float(
            self.get_parameter("gripper_close_position").value
        )
        self.hub_digital_output_topic = str(
            self.get_parameter("hub_digital_output_topic").value
        )
        self.hub_electromagnet_channel = float(
            self.get_parameter("hub_electromagnet_channel").value
        )
        self.petanque_param_service = str(self.get_parameter("petanque_param_service").value)
        self.petanque_total_duration_param = str(
            self.get_parameter("petanque_total_duration_param").value
        )
        self.petanque_angle_between_start_and_finish_param = str(
            self.get_parameter("petanque_angle_between_start_and_finish_param").value
        )
        self.param_call_timeout_sec = float(self.get_parameter("param_call_timeout_sec").value)
        self.petanque_measurements_enabled = bool(
            self.get_parameter("petanque_measurements_enabled").value
        )
        self.petanque_measurement_request_image_topic = str(
            self.get_parameter("petanque_measurement_request_image_topic").value
        )
        self.petanque_measurement_points_topic = str(
            self.get_parameter("petanque_measurement_points_topic").value
        )
        self.petanque_measurement_result_image_topic = str(
            self.get_parameter("petanque_measurement_result_image_topic").value
        )
        self.petanque_measurement_result_vectors_topic = str(
            self.get_parameter("petanque_measurement_result_vectors_topic").value
        )
        self.petanque_sphere_diameter_m = float(
            self.get_parameter("petanque_sphere_diameter_m").value
        )
        self.petanque_click_to_circle_threshold_px = float(
            self.get_parameter("petanque_click_to_circle_threshold_px").value
        )
        self.petanque_click_search_margin_px = float(
            self.get_parameter("petanque_click_search_margin_px").value
        )
        self.petanque_max_candidate_spheres = int(
            self.get_parameter("petanque_max_candidate_spheres").value
        )
        self.petanque_intrinsics_mode = str(
            self.get_parameter("petanque_intrinsics_mode").value
        ).strip().lower()
        if self.petanque_intrinsics_mode not in {"image", "fixed"}:
            self.get_logger().warning(
                f"Invalid petanque_intrinsics_mode={self.petanque_intrinsics_mode}, using image"
            )
            self.petanque_intrinsics_mode = "image"
        self.petanque_assumed_hfov_deg = float(
            self.get_parameter("petanque_assumed_hfov_deg").value
        )
        self.petanque_camera_fx = float(self.get_parameter("petanque_camera_fx").value)
        self.petanque_camera_fy = float(self.get_parameter("petanque_camera_fy").value)
        self.petanque_camera_cx = float(self.get_parameter("petanque_camera_cx").value)
        self.petanque_camera_cy = float(self.get_parameter("petanque_camera_cy").value)
        self.petanque_blur_kernel_size = int(
            self.get_parameter("petanque_blur_kernel_size").value
        )
        self.petanque_hough_dp = float(self.get_parameter("petanque_hough_dp").value)
        self.petanque_hough_min_dist_px = float(
            self.get_parameter("petanque_hough_min_dist_px").value
        )
        self.petanque_hough_param1 = float(
            self.get_parameter("petanque_hough_param1").value
        )
        self.petanque_hough_param2 = float(
            self.get_parameter("petanque_hough_param2").value
        )
        self.petanque_min_radius_px = int(
            self.get_parameter("petanque_min_radius_px").value
        )
        self.petanque_max_radius_px = int(
            self.get_parameter("petanque_max_radius_px").value
        )
        try:
            self.linear_axes, self.linear_signs = normalize_mapping(
                axes=linear_axes_param,
                signs=linear_signs_param,
            )
            self.angular_axes, self.angular_signs = normalize_mapping(
                axes=angular_axes_param,
                signs=angular_signs_param,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f"Invalid axis mapping parameters, using identity mapping: {exc}"
            )
            self.linear_axes = (0, 1, 2)
            self.linear_signs = (1.0, 1.0, 1.0)
            self.angular_axes = (0, 1, 2)
            self.angular_signs = (1.0, 1.0, 1.0)

        self._lock = threading.Lock()
        self._latest_twist = Twist()
        self._current_mode: int = self.default_mode
        self._last_cmd_received_ms: Optional[int] = None
        self._last_seq: int = 0
        self._connected: bool = False
        self._last_events: List[str] = []
        self._gripper_state: str = "unknown"
        self._petanque_processor: Optional[PetanqueMeasurements] = None
        self._petanque_latest_image_msg: Optional[CompressedImage] = None
        self._petanque_latest_points: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None

        self._publisher = self.create_publisher(TeleopCommand, self.teleop_cmd_topic, 10)
        self._state_cmd_publisher = self.create_publisher(
            String, self.state_machine_topic, 10
        )
        self._gripper_publisher = self.create_publisher(
            Float64MultiArray, self.gripper_topic, 10
        )
        self._hub_digital_output_publisher = self.create_publisher(
            Float32MultiArray, self.hub_digital_output_topic, 10
        )
        self._petanque_overlay_publisher = self.create_publisher(
            CompressedImage, self.petanque_measurement_result_image_topic, 10
        )
        self._petanque_vectors_publisher = self.create_publisher(
            String, self.petanque_measurement_result_vectors_topic, 10
        )
        self._gripper_subscription = self.create_subscription(
            Float64MultiArray, self.gripper_topic, self._on_gripper_command, 10
        )
        self._petanque_image_subscription = self.create_subscription(
            CompressedImage,
            self.petanque_measurement_request_image_topic,
            self._on_petanque_image,
            10,
        )
        self._petanque_points_subscription = self.create_subscription(
            Float32MultiArray,
            self.petanque_measurement_points_topic,
            self._on_petanque_points,
            10,
        )
        self._petanque_param_client = self.create_client(
            SetParameters, self.petanque_param_service
        )
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        if self.petanque_measurements_enabled:
            self._petanque_processor = PetanqueMeasurements(
                intrinsics=CameraIntrinsics(
                    fx=self.petanque_camera_fx,
                    fy=self.petanque_camera_fy,
                    cx=self.petanque_camera_cx,
                    cy=self.petanque_camera_cy,
                ),
                detection_config=CircleDetectionConfig(
                    blur_kernel_size=self.petanque_blur_kernel_size,
                    hough_dp=self.petanque_hough_dp,
                    hough_min_dist_px=self.petanque_hough_min_dist_px,
                    hough_param1=self.petanque_hough_param1,
                    hough_param2=self.petanque_hough_param2,
                    min_radius_px=self.petanque_min_radius_px,
                    max_radius_px=self.petanque_max_radius_px,
                ),
                measurement_config=MeasurementConfig(
                    sphere_diameter_m=self.petanque_sphere_diameter_m,
                    click_to_circle_threshold_px=self.petanque_click_to_circle_threshold_px,
                    click_search_margin_px=self.petanque_click_search_margin_px,
                    max_candidate_spheres=self.petanque_max_candidate_spheres,
                ),
            )

        self.get_logger().info("Tablet interface node initialized")
        self.get_logger().info("SafetyGate disabled for debug: raw mapped command forwarding")
        self.get_logger().info(
            "WS params: bind_host={0} bind_port={1} ws_path={2} state_publish_hz={3:.1f}".format(
                self.bind_host,
                self.bind_port,
                self.ws_path,
                self.state_publish_hz,
            )
        )
        self.get_logger().info(
            "Scale params: linear_scale={0:.3f} angular_scale={1:.3f}".format(
                self.linear_scale,
                self.angular_scale,
            )
        )
        self.get_logger().info(
            "Mapping params: linear_axes={0} linear_signs={1} "
            "angular_axes={2} angular_signs={3} swap_xy={4}".format(
                self.linear_axes,
                self.linear_signs,
                self.angular_axes,
                self.angular_signs,
                str(self.swap_xy).lower(),
            )
        )
        self.get_logger().info(
            "Teleop params: topic={0} publish_rate_hz={1:.1f} accept_mode_from_client={2}".format(
                self.teleop_cmd_topic,
                self.publish_rate_hz,
                str(self.accept_mode_from_client).lower(),
            )
        )
        self.get_logger().info(
            "Petanque bridge: state_machine_topic={0} param_service={1} "
            "duration_param={2} angle_param={3}".format(
                self.state_machine_topic,
                self.petanque_param_service,
                self.petanque_total_duration_param,
                self.petanque_angle_between_start_and_finish_param,
            )
        )
        self.get_logger().info(
            "Gripper bridge: topic={0} open={1:.3f} close={2:.3f}".format(
                self.gripper_topic,
                self.gripper_open_position,
                self.gripper_close_position,
            )
        )
        self.get_logger().info(
            "Hub bridge: digital_output_topic={0} electromagnet_channel={1:.1f}".format(
                self.hub_digital_output_topic,
                self.hub_electromagnet_channel,
            )
        )
        self.get_logger().info(
            "Petanque measurements: enabled={0} image_topic={1} points_topic={2} "
            "result_image_topic={3} result_vectors_topic={4} sphere_diameter_m={5:.3f} "
            "click_threshold_px={6:.1f} search_margin_px={7:.1f} max_candidates={8}".format(
                str(self.petanque_measurements_enabled).lower(),
                self.petanque_measurement_request_image_topic,
                self.petanque_measurement_points_topic,
                self.petanque_measurement_result_image_topic,
                self.petanque_measurement_result_vectors_topic,
                self.petanque_sphere_diameter_m,
                self.petanque_click_to_circle_threshold_px,
                self.petanque_click_search_margin_px,
                self.petanque_max_candidate_spheres,
            )
        )

    def map_and_scale_cmd(
        self,
        *,
        linear_values: Tuple[float, float, float],
        angular_values: Tuple[float, float, float],
    ) -> Twist:
        linear, angular = map_and_scale(
            linear_values=linear_values,
            angular_values=angular_values,
            linear_axes=self.linear_axes,
            linear_signs=self.linear_signs,
            angular_axes=self.angular_axes,
            angular_signs=self.angular_signs,
            linear_scale=self.linear_scale,
            angular_scale=self.angular_scale,
            swap_xy=self.swap_xy,
        )
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        return twist

    def update_latest_cmd(
        self,
        *,
        twist: Twist,
        mode: int,
        seq: int,
        received_ms: Optional[int] = None,
    ) -> None:
        now_ms = self._now_ms()
        if received_ms is None:
            received_ms = now_ms

        if not self.accept_mode_from_client:
            mode = self.default_mode

        with self._lock:
            self._latest_twist = self._copy_twist(twist)
            self._current_mode = int(mode)
            self._last_cmd_received_ms = int(received_ms)
            self._last_seq = int(seq)

    def send_state_command(self, command: str) -> bool:
        normalized = command.strip().lower()
        if normalized not in {
            "teleop",
            "activate_throw",
            "go_to_start",
            "throw",
            "pick_up",
            "stop",
        }:
            self.get_logger().warning(f"Invalid state machine command: {command}")
            return False

        msg = String()
        msg.data = normalized
        self._state_cmd_publisher.publish(msg)
        self.get_logger().info(f"Published state machine command: {normalized}")
        return True

    def set_gripper(self, action: str) -> bool:
        normalized = action.strip().lower()
        if normalized not in {"open", "close"}:
            self.get_logger().warning(f"Invalid gripper action: {action}")
            return False

        position = (
            self.gripper_open_position
            if normalized == "open"
            else self.gripper_close_position
        )
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._gripper_publisher.publish(msg)
        with self._lock:
            self._gripper_state = normalized
        self.get_logger().info(
            "Published gripper command: action={0} topic={1} value={2:.3f}".format(
                normalized,
                self.gripper_topic,
                position,
            )
        )
        return True

    def set_electromagnet(self, enabled: bool) -> bool:
        msg = Float32MultiArray()
        # Hardware wiring for the electromagnet is active-low:
        # 0.0 => magnet ON, 1.0 => magnet OFF.
        msg.data = [
            float(self.hub_electromagnet_channel),
            0.0 if enabled else 1.0,
        ]
        self._hub_digital_output_publisher.publish(msg)
        self.get_logger().info(
            "Published hub digital output: channel={0:.1f} value={1:.1f}".format(
                self.hub_electromagnet_channel,
                msg.data[1],
            )
        )
        return True

    def _on_gripper_command(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        self._set_gripper_state_from_position(float(msg.data[0]))

    def _on_petanque_image(self, msg: CompressedImage) -> None:
        if not self.petanque_measurements_enabled or self._petanque_processor is None:
            return
        with self._lock:
            self._petanque_latest_image_msg = msg
        self._run_petanque_measurement_if_ready()

    def _on_petanque_points(self, msg: Float32MultiArray) -> None:
        if not self.petanque_measurements_enabled or self._petanque_processor is None:
            return
        if len(msg.data) < 4:
            self.get_logger().warning(
                "Petanque points message must contain at least 4 values [x1, y1, x2, y2]"
            )
            return
        points = (
            (float(msg.data[0]), float(msg.data[1])),
            (float(msg.data[2]), float(msg.data[3])),
        )
        with self._lock:
            self._petanque_latest_points = points
        self._run_petanque_measurement_if_ready()

    def _run_petanque_measurement_if_ready(self) -> None:
        if self._petanque_processor is None:
            return

        with self._lock:
            image_msg = self._petanque_latest_image_msg
            points = self._petanque_latest_points

        if image_msg is None or points is None:
            return

        image_bgr = self._compressed_image_msg_to_bgr(image_msg)
        if image_bgr is None:
            return

        self._update_petanque_intrinsics_from_frame(image_bgr)

        result = self._petanque_processor.process(
            image_bgr=image_bgr,
            point_a_px=points[0],
            point_b_px=points[1],
        )

        overlay_msg = self._bgr_to_compressed_image_msg(result.overlay_bgr, image_msg)
        self._petanque_overlay_publisher.publish(overlay_msg)

        vectors_payload: Dict[str, object] = {
            "valid": bool(result.valid),
            "message": str(result.message),
            "point_a_px": [float(points[0][0]), float(points[0][1])],
            "point_b_px": [float(points[1][0]), float(points[1][1])],
            "distance_m": float(result.distance_m) if result.distance_m is not None else None,
        }
        if result.point_a_3d_m is not None:
            vectors_payload["point_a_3d_m"] = [
                float(result.point_a_3d_m[0]),
                float(result.point_a_3d_m[1]),
                float(result.point_a_3d_m[2]),
            ]
        if result.point_b_3d_m is not None:
            vectors_payload["point_b_3d_m"] = [
                float(result.point_b_3d_m[0]),
                float(result.point_b_3d_m[1]),
                float(result.point_b_3d_m[2]),
            ]

        vectors_msg = String()
        vectors_msg.data = json.dumps(vectors_payload)
        self._petanque_vectors_publisher.publish(vectors_msg)

        if not result.valid:
            self.get_logger().debug(f"Petanque measurement skipped: {result.message}")

    def _set_gripper_state_from_position(self, position: float) -> None:
        open_distance = abs(position - float(self.gripper_open_position))
        close_distance = abs(position - float(self.gripper_close_position))
        state = "open" if open_distance <= close_distance else "close"
        with self._lock:
            self._gripper_state = state

    def set_petanque_total_duration(self, total_duration: float) -> bool:
        if total_duration <= 0.0:
            self.get_logger().warning(
                f"Invalid total_duration={total_duration:.3f}; expected > 0"
            )
            return False

        return self._set_petanque_double_parameter(
            parameter_name=self.petanque_total_duration_param,
            value=float(total_duration),
        )

    def set_petanque_angle_between_start_and_finish(self, angle: float) -> bool:
        return self._set_petanque_double_parameter(
            parameter_name=self.petanque_angle_between_start_and_finish_param,
            value=float(angle),
        )

    def _set_petanque_double_parameter(self, *, parameter_name: str, value: float) -> bool:
        if not parameter_name:
            self.get_logger().warning("Petanque parameter name is empty")
            return False

        if not self._petanque_param_client.wait_for_service(
            timeout_sec=self.param_call_timeout_sec
        ):
            self.get_logger().warning(
                f"Service unavailable: {self.petanque_param_service}"
            )
            return False

        param = Parameter(
            name=parameter_name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )
        req = SetParameters.Request(parameters=[param])
        future = self._petanque_param_client.call_async(req)

        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.param_call_timeout_sec):
            self.get_logger().warning(
                f"Timeout while setting parameter {parameter_name}"
            )
            return False

        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"SetParameters call failed: {exc}")
            return False

        if not result or not result.results:
            self.get_logger().warning("SetParameters returned empty result")
            return False

        if not result.results[0].successful:
            reason = result.results[0].reason or "unknown error"
            self.get_logger().warning(
                f"Failed to set {parameter_name}: {reason}"
            )
            return False

        self.get_logger().info(
            f"Updated {parameter_name}={value:.3f}"
        )
        return True

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = bool(connected)

    def get_state(self) -> Dict[str, object]:
        with self._lock:
            now_ms = self._now_ms()
            cmd_age_ms = None
            if self._last_cmd_received_ms is not None:
                cmd_age_ms = int(now_ms) - int(self._last_cmd_received_ms)

            return {
                "connected": self._connected,
                "cmd_age_ms": cmd_age_ms,
                "watchdog_timeout_ms": 0,
                "last_seq": self._last_seq,
                "publishing_rate_hz": float(self.publish_rate_hz),
                "current_mode": int(self._current_mode),
                "gripper_state": self._gripper_state,
                "events": list(self._last_events),
            }

    def _on_timer(self) -> None:
        with self._lock:
            twist = self._copy_twist(self._latest_twist)
            mode = int(self._current_mode)
            self._last_events = []

        msg = TeleopCommand()
        msg.twist = twist
        msg.mode = int(mode)
        self._publisher.publish(msg)

    @staticmethod
    def _copy_twist(twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = float(twist.linear.x)
        out.linear.y = float(twist.linear.y)
        out.linear.z = float(twist.linear.z)
        out.angular.x = float(twist.angular.x)
        out.angular.y = float(twist.angular.y)
        out.angular.z = float(twist.angular.z)
        return out

    def _now_ms(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000_000)

    def _compressed_image_msg_to_bgr(self, msg: CompressedImage) -> Optional[np.ndarray]:
        if cv2 is None:
            self.get_logger().warning("opencv not available for compressed image decode")
            return None

        if not msg.data:
            self.get_logger().warning(
                "Petanque compressed image payload is empty"
            )
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        decoded = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if decoded is None:
            self.get_logger().warning("Failed to decode petanque compressed image")
            return None
        return decoded

    def _update_petanque_intrinsics_from_frame(self, image_bgr: np.ndarray) -> None:
        if self._petanque_processor is None:
            return
        if self.petanque_intrinsics_mode == "fixed":
            self._petanque_processor.set_intrinsics(
                CameraIntrinsics(
                    fx=self.petanque_camera_fx,
                    fy=self.petanque_camera_fy,
                    cx=self.petanque_camera_cx,
                    cy=self.petanque_camera_cy,
                )
            )
            return

        intrinsics = PetanqueMeasurements.estimate_intrinsics_from_image(
            image_width_px=int(image_bgr.shape[1]),
            image_height_px=int(image_bgr.shape[0]),
            assumed_hfov_deg=self.petanque_assumed_hfov_deg,
        )
        self._petanque_processor.set_intrinsics(intrinsics)

    @staticmethod
    def _bgr_to_compressed_image_msg(
        image_bgr: np.ndarray,
        source_msg: CompressedImage,
    ) -> CompressedImage:
        if cv2 is None:
            out = CompressedImage()
            out.header = source_msg.header
            out.format = "jpeg"
            out.data = b""
            return out

        ok, encoded = cv2.imencode(".jpg", image_bgr)
        out = CompressedImage()
        out.header = source_msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes() if ok else b""
        return out


__all__ = ["TabletInterfaceNode"]
