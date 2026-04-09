#!/usr/bin/env python3

from __future__ import annotations

import math
import csv
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float32MultiArray

try:
    from gazebo_msgs.msg import ModelState, ModelStates
    from gazebo_msgs.srv import SetModelState
    _GAZEBO_MSGS_AVAILABLE = True
except Exception:
    ModelState = None
    ModelStates = None
    SetModelState = None
    _GAZEBO_MSGS_AVAILABLE = False


@dataclass
class CommandState:
    roll: float = 0.0
    pitch: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0
    x: float = 0.0     # used here for "sway"
    y: float = 0.0


class StewartCommandNode(Node):
    """ROS 2 controller that publishes pose commands (sinusoidal or from CSV)
    and a 140-step window of true heave values from the CSV.
    """

    def __init__(self) -> None:
        super().__init__("stewart_command_sinusoidal")

        # Main pose publisher
        self.publisher_ = self.create_publisher(
            Twist,
            "/stewart/platform_pose",
            10,
        )

        # New: heave prediction publisher (ground-truth window from CSV)
        self.heave_pred_pub = self.create_publisher(
            Float32MultiArray,
            "/heave_predicted_true",
            10,
        )

        # Mode parameters
        self.declare_parameter("use_csv", False)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("csv_heave_scale", 0.4)
        self.declare_parameter("zero_command", False)

        # Controller parameters (configurable via ROS parameters)
        self.declare_parameter("kp", 0.1)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("roll_amplitude", 0.0)
        self.declare_parameter("pitch_amplitude", 0.0)
        self.declare_parameter("heave_amplitude", 0.6)
        self.declare_parameter("x_amplitude", 1.0)
        self.declare_parameter("y_amplitude", 1.2)
        self.declare_parameter("frequency", 0.5)
        self.declare_parameter("x_frequency", 0.2)
        self.declare_parameter("y_frequency", 0.1)
        self.declare_parameter("x_phase", 0.0)
        self.declare_parameter("y_phase", math.pi / 2.0)
        self.declare_parameter("move_whole_platform_xy", False)
        self.declare_parameter("model_name", "stewart")
        self.declare_parameter("base_link_name", "base_link")
        self.declare_parameter("world_x_origin", 3.0)
        self.declare_parameter("world_y_origin", 0.0)
        self.declare_parameter("world_z", 0.0)
        self.declare_parameter("hold_initial_world_z", True)
        self.declare_parameter("hold_initial_world_orientation", True)
        self.declare_parameter("filter_alpha", 0.5)
        # 20 Hz -> 0.05 s between samples
        self.declare_parameter("command_rate", 20.0)

        self.use_csv: bool = self.get_parameter("use_csv").value
        self.csv_path: str = self.get_parameter("csv_path").value
        self.csv_heave_scale: float = float(self.get_parameter("csv_heave_scale").value)
        self.csv_data: List[CommandState] = []
        self.csv_index: int = 0
        self.move_whole_platform_xy: bool = bool(
            self.get_parameter("move_whole_platform_xy").value
        )
        self.model_name: str = str(self.get_parameter("model_name").value)
        self.base_link_name: str = str(self.get_parameter("base_link_name").value)
        self.world_x_origin: float = float(self.get_parameter("world_x_origin").value)
        self.world_y_origin: float = float(self.get_parameter("world_y_origin").value)
        self.world_z: float = float(self.get_parameter("world_z").value)
        self.hold_initial_world_z: bool = bool(
            self.get_parameter("hold_initial_world_z").value
        )
        self.hold_initial_world_orientation: bool = bool(
            self.get_parameter("hold_initial_world_orientation").value
        )

        self._set_model_state_client = None
        self._set_model_state_client_alt = None
        self._set_model_state_pub = None
        self._set_model_state_pub_alt = None
        self._base_pose_pub = None
        self._model_states_sub = None
        self._have_initial_pose = False
        self._initial_world_z = None
        self._initial_world_orientation = None

        if self.use_csv:
            self._load_csv()
        if self.move_whole_platform_xy:
            self._init_world_xy_client()

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_error = CommandState()
        self.last_command = CommandState()

        command_period = 1.0 / self.get_parameter("command_rate").value
        self.timer = self.create_timer(command_period, self._timer_callback)

        zero_command = self.get_parameter("zero_command").value
        mode_str = "zero" if zero_command else (
            "CSV" if self.use_csv and self.csv_data else "sinusoidal"
        )
        self.get_logger().info(
            f"Stewart command node ready (rate={command_period:.3f} Hz, mode={mode_str})"
        )

    def _init_world_xy_client(self) -> None:
        if not _GAZEBO_MSGS_AVAILABLE:
            self.get_logger().error(
                "move_whole_platform_xy=True but gazebo_msgs is unavailable. "
                "Falling back to IK x/y (top-platform motion)."
            )
            self.move_whole_platform_xy = False
            return

        self._set_model_state_client = self.create_client(
            SetModelState, "/gazebo/set_model_state"
        )
        self._set_model_state_client_alt = self.create_client(
            SetModelState, "/set_model_state"
        )
        self._set_model_state_pub = self.create_publisher(
            ModelState, "/gazebo/set_model_state", 10
        )
        self._set_model_state_pub_alt = self.create_publisher(
            ModelState, "/set_model_state", 10
        )
        self._base_pose_pub = self.create_publisher(
            Pose, "/stewart/base_pose_cmd", 10
        )
        self._model_states_sub = self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._model_states_callback,
            10,
        )
        self.get_logger().info(
            "Whole-platform XY mode enabled via set_model_state "
            f"(model='{self.model_name}', origin=({self.world_x_origin:.3f}, "
            f"{self.world_y_origin:.3f}), z={self.world_z:.3f})"
        )

    def _model_states_callback(self, msg: ModelStates) -> None:
        if self._have_initial_pose:
            return

        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return

        self._initial_world_z = float(msg.pose[idx].position.z)
        self._initial_world_orientation = msg.pose[idx].orientation
        self._have_initial_pose = True
        self.get_logger().info(
            f"Captured initial model pose for hold: z={self._initial_world_z:.4f}"
        )

    def _publish_world_xy(self, x_cmd: float, y_cmd: float) -> None:
        if not self.move_whole_platform_xy:
            return

        z_cmd = self.world_z
        if self.hold_initial_world_z and self._have_initial_pose:
            z_cmd = float(self._initial_world_z)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = self.world_x_origin + x_cmd
        state.pose.position.y = self.world_y_origin + y_cmd
        state.pose.position.z = z_cmd

        if self.hold_initial_world_orientation and self._have_initial_pose:
            state.pose.orientation = self._initial_world_orientation
        else:
            state.pose.orientation.w = 1.0
        state.twist = Twist()

        if self._set_model_state_pub is not None:
            self._set_model_state_pub.publish(state)
        if self._set_model_state_pub_alt is not None:
            self._set_model_state_pub_alt.publish(state)

        if self._set_model_state_client is not None and self._set_model_state_client.service_is_ready():
            req = SetModelState.Request()
            req.model_state = state
            self._set_model_state_client.call_async(req)
        elif (
            self._set_model_state_client_alt is not None
            and self._set_model_state_client_alt.service_is_ready()
        ):
            req = SetModelState.Request()
            req.model_state = state
            self._set_model_state_client_alt.call_async(req)

        if self._base_pose_pub is not None:
            base_pose = Pose()
            base_pose.position = state.pose.position
            base_pose.orientation = state.pose.orientation
            self._base_pose_pub.publish(base_pose)

    def _load_csv(self) -> None:
        """Load roll, pitch, heave, sway from a 4-column CSV (no header)."""
        if not self.csv_path:
            self.get_logger().error("use_csv=True but csv_path is empty; falling back to sinusoidal.")
            self.use_csv = False
            return

        try:
            with open(self.csv_path, "r", newline="") as f:
                reader = csv.reader(f)
                count = 0
                for idx, row in enumerate(reader):
                    if len(row) < 4:
                        self.get_logger().warn(
                            f"CSV row {idx} has {len(row)} columns (<4), skipping."
                        )
                        continue
                    try:
                        roll = float(row[0])
                        pitch = float(row[1])
                        heave = float(row[2]) * self.csv_heave_scale
                        sway = float(row[3])
                    except ValueError:
                        self.get_logger().warn(
                            f"CSV row {idx} contains non-numeric data, skipping."
                        )
                        continue

                    # sway is mapped to x translation (linear.x)
                    self.csv_data.append(
                        CommandState(
                            roll=roll,
                            pitch=pitch,
                            heave=heave,
                            yaw=0.0,
                            x=sway,
                            y=0.0,
                        )
                    )
                    count += 1

            if count == 0:
                self.get_logger().error(
                    f"No valid rows found in CSV '{self.csv_path}'. Falling back to sinusoidal."
                )
                self.use_csv = False
            else:
                self.get_logger().info(
                    f"Loaded {count} samples from CSV '{self.csv_path}' "
                    "(dt assumed 0.05 s at 20 Hz)."
                )

        except Exception as e:
            self.get_logger().error(
                f"Failed to read CSV '{self.csv_path}': {e}. Falling back to sinusoidal."
            )
            self.use_csv = False

    def _timer_callback(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        if self.get_parameter("zero_command").value:
            self.prev_error = CommandState()
            self.last_command = CommandState()

            twist = Twist()
            self.publisher_.publish(twist)
            self.get_logger().debug("cmd zero (zero_command=True)")
            return

        # Choose desired command: CSV or sinusoidal
        if self.use_csv and self.csv_data:
            # Current index for this tick
            if self.csv_index < len(self.csv_data):
                idx_curr = self.csv_index
                desired = self.csv_data[idx_curr]
                self.csv_index += 1
            else:
                idx_curr = len(self.csv_data) - 1
                desired = self.csv_data[idx_curr]

            # Build 120-step heave window starting from idx_curr
            window_len = 120
            heave_msg = Float32MultiArray()
            heave_msg.data = []

            for k in range(window_len):
                idx = idx_curr + k
                if idx < len(self.csv_data):
                    heave_val = float(self.csv_data[idx].heave)
                else:
                    # Past end: hold last available value
                    heave_val = float(self.csv_data[-1].heave)
                heave_msg.data.append(heave_val)

            self.heave_pred_pub.publish(heave_msg)

        else:
            # Sinusoidal mode: no CSV window to publish (heave_predicted_true
            # will remain unused / empty in this mode).
            freq = self.get_parameter("frequency").value
            x_freq = self.get_parameter("x_frequency").value
            y_freq = self.get_parameter("y_frequency").value
            x_phase = self.get_parameter("x_phase").value
            y_phase = self.get_parameter("y_phase").value
            desired = CommandState(
                roll=self.get_parameter("roll_amplitude").value * math.sin(freq * elapsed),
                pitch=self.get_parameter("pitch_amplitude").value * math.sin(
                    freq * elapsed + math.pi / 2.0
                ),
                heave=self.get_parameter("heave_amplitude").value * math.sin(freq * elapsed),
                yaw=0.0,
                x=self.get_parameter("x_amplitude").value * math.sin(
                    x_freq * elapsed + x_phase
                ),
                y=self.get_parameter("y_amplitude").value * math.sin(
                    y_freq * elapsed + y_phase
                ),
            )

        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value

        errors = CommandState(
            roll=desired.roll - self.last_command.roll,
            pitch=desired.pitch - self.last_command.pitch,
            heave=desired.heave - self.last_command.heave,
            yaw=desired.yaw - self.last_command.yaw,
            x=desired.x - self.last_command.x,
            y=desired.y - self.last_command.y,
        )

        command = CommandState(
            roll=self._pd(desired.roll, errors.roll, self.prev_error.roll, kp, kd),
            pitch=self._pd(desired.pitch, errors.pitch, self.prev_error.pitch, kp, kd),
            heave=self._pd(desired.heave, errors.heave, self.prev_error.heave, kp, kd),
            yaw=self._pd(desired.yaw, errors.yaw, self.prev_error.yaw, kp, kd),
            x=self._pd(desired.x, errors.x, self.prev_error.x, kp, kd),
            y=self._pd(desired.y, errors.y, self.prev_error.y, kp, kd),
        )

        self.prev_error = errors

        alpha = self.get_parameter("filter_alpha").value
        smoothed = CommandState(
            roll=self._smooth(command.roll, self.last_command.roll, alpha),
            pitch=self._smooth(command.pitch, self.last_command.pitch, alpha),
            heave=self._smooth(command.heave, self.last_command.heave, alpha),
            yaw=self._smooth(command.yaw, self.last_command.yaw, alpha),
            x=self._smooth(command.x, self.last_command.x, alpha),
            y=self._smooth(command.y, self.last_command.y, alpha),
        )
        self.last_command = smoothed

        ik_x = smoothed.x
        ik_y = smoothed.y
        if self.move_whole_platform_xy:
            self._publish_world_xy(smoothed.x, smoothed.y)
            # Keep IK x/y zero so only whole-model XY is applied.
            ik_x = 0.0
            ik_y = 0.0

        twist = Twist()
        twist.angular.x = smoothed.roll
        twist.angular.y = smoothed.pitch
        twist.angular.z = smoothed.yaw
        twist.linear.x = ik_x       # sway mapped to x
        twist.linear.y = ik_y
        twist.linear.z = smoothed.heave

        self.publisher_.publish(twist)
        self.get_logger().debug(
            f"cmd roll={smoothed.roll:.3f} pitch={smoothed.pitch:.3f} "
            f"heave={smoothed.heave:.3f} x={smoothed.x:.3f} y={smoothed.y:.3f} "
            f"(whole_xy={'on' if self.move_whole_platform_xy else 'off'})"
        )

    @staticmethod
    def _pd(desired: float, error: float, prev_error: float, kp: float, kd: float) -> float:
        return desired + kp * error + kd * (error - prev_error)

    @staticmethod
    def _smooth(value: float, previous: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        return alpha * value + (1.0 - alpha) * previous


def main() -> None:
    rclpy.init()
    node = StewartCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
