#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


@dataclass
class CommandState:
    roll: float = 0.0
    pitch: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0


class StewartCommandNode(Node):
    """Publish Stewart pose commands from sinusoidal motion or CSV."""

    def __init__(self) -> None:
        super().__init__("stewart_command_sinusoidal")

        self.publisher_ = self.create_publisher(Twist, "/stewart/platform_pose", 10)
        self.heave_pred_pub = self.create_publisher(
            Float32MultiArray, "/heave_predicted_true", 10
        )

        self.declare_parameter("use_csv", False)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("csv_heave_scale", 0.4)
        self.declare_parameter("zero_command", False)

        self.declare_parameter("kp", 0.1)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("roll_amplitude", 0.0)
        self.declare_parameter("pitch_amplitude", 0.0)
        self.declare_parameter("heave_amplitude", 0.6)
        self.declare_parameter("frequency", 0.5)
        self.declare_parameter("filter_alpha", 0.5)
        self.declare_parameter("command_rate", 20.0)

        self.use_csv: bool = bool(self.get_parameter("use_csv").value)
        self.csv_path: str = str(self.get_parameter("csv_path").value)
        self.csv_heave_scale: float = float(self.get_parameter("csv_heave_scale").value)
        self.csv_data: List[CommandState] = []
        self.csv_index = 0

        if self.use_csv:
            self._load_csv()

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_error = CommandState()
        self.last_command = CommandState()

        self.command_period = 1.0 / float(self.get_parameter("command_rate").value)
        self.timer = self.create_timer(self.command_period, self._timer_callback)

        zero_command = bool(self.get_parameter("zero_command").value)
        mode_str = "zero" if zero_command else (
            "CSV" if self.use_csv and self.csv_data else "sinusoidal"
        )
        self.get_logger().info(
            f"Stewart command node ready (rate={self.command_period:.3f} Hz, mode={mode_str})"
        )

    def _load_csv(self) -> None:
        if not self.csv_path:
            self.get_logger().error(
                "use_csv=True but csv_path is empty; falling back to sinusoidal."
            )
            self.use_csv = False
            return

        try:
            with open(self.csv_path, "r", newline="") as handle:
                reader = csv.reader(handle)
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

                    self.csv_data.append(
                        CommandState(
                            roll=roll,
                            pitch=pitch,
                            heave=heave,
                            yaw=0.0,
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
        except Exception as exc:
            self.get_logger().error(
                f"Failed to read CSV '{self.csv_path}': {exc}. Falling back to sinusoidal."
            )
            self.use_csv = False

    def _timer_callback(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        if bool(self.get_parameter("zero_command").value):
            self.prev_error = CommandState()
            self.last_command = CommandState()
            self.publisher_.publish(Twist())
            return

        if self.use_csv and self.csv_data:
            if self.csv_index < len(self.csv_data):
                idx_curr = self.csv_index
                desired = self.csv_data[idx_curr]
                self.csv_index += 1
            else:
                idx_curr = len(self.csv_data) - 1
                desired = self.csv_data[idx_curr]

            window_len = 120
            heave_msg = Float32MultiArray()
            heave_msg.data = []
            for k in range(window_len):
                idx = idx_curr + k
                if idx < len(self.csv_data):
                    heave_val = float(self.csv_data[idx].heave)
                else:
                    heave_val = float(self.csv_data[-1].heave)
                heave_msg.data.append(heave_val)
            self.heave_pred_pub.publish(heave_msg)
        else:
            freq = float(self.get_parameter("frequency").value)
            desired = CommandState(
                roll=float(self.get_parameter("roll_amplitude").value)
                * math.sin(freq * elapsed),
                pitch=float(self.get_parameter("pitch_amplitude").value)
                * math.sin(freq * elapsed + math.pi / 2.0),
                heave=float(self.get_parameter("heave_amplitude").value)
                * math.sin(freq * elapsed),
                yaw=0.0,
            )

        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)

        errors = CommandState(
            roll=desired.roll - self.last_command.roll,
            pitch=desired.pitch - self.last_command.pitch,
            heave=desired.heave - self.last_command.heave,
            yaw=desired.yaw - self.last_command.yaw,
        )

        command = CommandState(
            roll=self._pd(desired.roll, errors.roll, self.prev_error.roll, kp, kd),
            pitch=self._pd(desired.pitch, errors.pitch, self.prev_error.pitch, kp, kd),
            heave=self._pd(desired.heave, errors.heave, self.prev_error.heave, kp, kd),
            yaw=self._pd(desired.yaw, errors.yaw, self.prev_error.yaw, kp, kd),
        )
        self.prev_error = errors

        alpha = float(self.get_parameter("filter_alpha").value)
        smoothed = CommandState(
            roll=self._smooth(command.roll, self.last_command.roll, alpha),
            pitch=self._smooth(command.pitch, self.last_command.pitch, alpha),
            heave=self._smooth(command.heave, self.last_command.heave, alpha),
            yaw=self._smooth(command.yaw, self.last_command.yaw, alpha),
        )
        self.last_command = smoothed

        twist = Twist()
        twist.angular.x = smoothed.roll
        twist.angular.y = smoothed.pitch
        twist.angular.z = smoothed.yaw
        twist.linear.z = smoothed.heave
        self.publisher_.publish(twist)

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
