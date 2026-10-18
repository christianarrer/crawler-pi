#!/usr/bin/env python3

import csv
import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from datetime import datetime


class CrawlerOdomCalNode(Node):
    def __init__(self):
        super().__init__("crawler_odom_cal_node")
        
        self.start_sample_captured = False
        self.odom_x_start = None
        self.wall_dist_start = None

        self.declare_parameter("speed", 0.05)
        self.declare_parameter("start_delay_s", 8.0)
        self.declare_parameter("cmd_hz", 10.0)
        self.declare_parameter("log_hz", 10.0)
        self.declare_parameter("log_file", "/home/ros/odom_cal.csv")
        self.declare_parameter("log_file_xy", "/home/ros/odom_cal_xy.csv")
        self.declare_parameter("log_file_summary", "/home/ros/odom_cal_summary.txt")
        
        self.declare_parameter("front_angle_rad", math.pi)
        self.declare_parameter("search_half_width_rad", 0.65)
        self.declare_parameter("min_points", 15)
        self.declare_parameter("max_std_x", 0.06)
        self.declare_parameter("min_width_m", 0.40)

        self.speed = float(self.get_parameter("speed").value)
        self.speed = max(0.01, min(0.15, self.speed))

        self.start_delay_s = float(self.get_parameter("start_delay_s").value)
        self.cmd_hz = float(self.get_parameter("cmd_hz").value)
        self.log_hz = float(self.get_parameter("log_hz").value)
        self.log_file = str(self.get_parameter("log_file").value)
        self.log_file_xy = str(self.get_parameter("log_file_xy").value)
        self.log_file_summary = str(self.get_parameter("log_file_summary").value)
        
        self.xy_file = open(self.log_file_xy, "w", newline="")
        self.xy_writer = csv.writer(self.xy_file)
        self.xy_writer.writerow(["t_sec", "odom_x_m", "lidar_x_m"])
        self.xy_file.flush()

        self.front_angle_rad = float(self.get_parameter("front_angle_rad").value)
        self.search_half_width_rad = float(self.get_parameter("search_half_width_rad").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.max_std_x = float(self.get_parameter("max_std_x").value)
        self.min_width_m = float(self.get_parameter("min_width_m").value)

        self.t0 = time.monotonic()
        self.is_driving = False
        self.latest_cmd_x = 0.0
        
        self.samples = []

        self.latest_odom_x: Optional[float] = None
        self.latest_odom_y: Optional[float] = None
        self.latest_odom_stamp_s: Optional[float] = None

        self.latest_wall_dist: Optional[float] = None
        self.latest_wall_width: Optional[float] = None
        self.latest_wall_std_x: Optional[float] = None
        self.latest_wall_points: int = 0
        self.latest_wall_stamp_s: Optional[float] = None
        self.wall_valid = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            20,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        self.csv_file = open(self.log_file, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "ros_time_s",
            "elapsed_s",
            "is_driving",
            "cmd_linear_x",
            "odom_x",
            "odom_y",
            "odom_stamp_s",
            "wall_valid",
            "wall_dist_m",
            "wall_width_m",
            "wall_std_x_m",
            "wall_points",
            "wall_stamp_s",
        ])
        self.csv_file.flush()

        self.cmd_timer = self.create_timer(1.0 / self.cmd_hz, self.cmd_timer_callback)
        self.log_timer = self.create_timer(1.0 / self.log_hz, self.log_timer_callback)

        self.get_logger().info(
            f"Calibration node started. speed={self.speed:.3f} m/s, "
            f"start_delay_s={self.start_delay_s:.1f}, log_file={self.log_file}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom_x = msg.pose.pose.position.x
        self.latest_odom_y = msg.pose.pose.position.y
        self.latest_odom_stamp_s = self.stamp_to_s(msg.header.stamp)

    def scan_callback(self, msg: LaserScan):
        points = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            diff = self.angle_diff(angle, self.front_angle_rad)

            if abs(diff) > self.search_half_width_rad:
                continue

            # Coordinates with x pointing toward the selected wall direction.
            # For LDS-01 on your setup front seems to be around pi rad.
            x = r * math.cos(diff)
            y = r * math.sin(diff)
            points.append((x, y, angle))

        if len(points) < self.min_points:
            self.wall_valid = False
            self.latest_wall_points = len(points)
            self.latest_wall_stamp_s = self.stamp_to_s(msg.header.stamp)
            return

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        median_x = self.median(xs)

        # Keep points close to median wall distance.
        filtered = [
            p for p in points
            if abs(p[0] - median_x) <= self.max_std_x * 2.0
        ]

        if len(filtered) < self.min_points:
            self.wall_valid = False
            self.latest_wall_points = len(filtered)
            self.latest_wall_stamp_s = self.stamp_to_s(msg.header.stamp)
            return

        xs = [p[0] for p in filtered]
        ys = [p[1] for p in filtered]
        angles = [p[2] for p in filtered]

        mean_x = sum(xs) / len(xs)
        var_x = sum((x - mean_x) ** 2 for x in xs) / len(xs)
        std_x = math.sqrt(var_x)
        width = max(ys) - min(ys)

        if std_x > self.max_std_x or width < self.min_width_m:
            self.wall_valid = False
        else:
            self.wall_valid = True

        self.latest_wall_dist = mean_x
        self.latest_wall_width = width
        self.latest_wall_std_x = std_x
        self.latest_wall_points = len(filtered)
        self.latest_wall_stamp_s = self.stamp_to_s(msg.header.stamp)

    def cmd_timer_callback(self):
        elapsed = time.monotonic() - self.t0

        msg = Twist()

        if elapsed >= self.start_delay_s:
            msg.linear.x = self.speed
            self.is_driving = True
            if not self.start_sample_captured:
                if self.latest_odom_x is not None and self.latest_wall_dist is not None:
                    self.odom_x_start = self.latest_odom_x
                    self.wall_dist_start = self.latest_wall_dist
                    self.start_sample_captured = True
                    self.get_logger().info(
                        f"Start captured: odom_x={self.odom_x_start:.3f}, "
                        f"wall_dist={self.wall_dist_start:.3f}"
                    )
        else:
            msg.linear.x = 0.0
            self.is_driving = False

        self.latest_cmd_x = msg.linear.x
        self.cmd_pub.publish(msg)

    def log_timer_callback(self):
        ros_time_s = self.get_clock().now().nanoseconds / 1e9
        elapsed = time.monotonic() - self.t0

        self.csv_writer.writerow([
            f"{ros_time_s:.6f}",
            f"{elapsed:.6f}",
            int(self.is_driving),
            f"{self.latest_cmd_x:.6f}",
            self.format_optional(self.latest_odom_x),
            self.format_optional(self.latest_odom_y),
            self.format_optional(self.latest_odom_stamp_s),
            int(self.wall_valid),
            self.format_optional(self.latest_wall_dist),
            self.format_optional(self.latest_wall_width),
            self.format_optional(self.latest_wall_std_x),
            self.latest_wall_points,
            self.format_optional(self.latest_wall_stamp_s),
        ])
        self.csv_file.flush()
        
        if (
            self.is_driving
            and self.start_sample_captured
            and self.wall_valid
            and self.latest_odom_x is not None
            and self.latest_wall_dist is not None
        ):
            self.add_sample(
                odom_x_m=abs(self.latest_odom_x - self.odom_x_start),
                lidar_x_m=abs(self.latest_wall_dist - self.wall_dist_start),
            )

    def stop_robot(self):
        print("Stopping robot...")
        msg = Twist()

        for _ in range(10):
            try:
                if not rclpy.ok():
                    break
                self.cmd_pub.publish(msg)
                time.sleep(0.05)
            except Exception as exc:
                print(f"WARNING: Could not publish stop command: {exc}")
                break

    def close_log(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f"Log written: {self.log_file}")
        except Exception:
            pass

        try:
            self.xy_file.flush()
            self.xy_file.close()
            self.get_logger().info(f"XY log written: {self.log_file_xy}")
        except Exception:
            pass

    @staticmethod
    def stamp_to_s(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def angle_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def median(values):
        values = sorted(values)
        n = len(values)
        mid = n // 2
        if n % 2 == 1:
            return values[mid]
        return 0.5 * (values[mid - 1] + values[mid])

    @staticmethod
    def format_optional(value):
        if value is None:
            return ""
        return f"{value:.6f}"
        
    def add_sample(self, odom_x_m: float, lidar_x_m: float):
        if not math.isfinite(odom_x_m) or not math.isfinite(lidar_x_m):
            return

        if abs(odom_x_m) < 0.001:
            return

        t_sec = self.get_clock().now().nanoseconds * 1e-9

        self.samples.append((t_sec, odom_x_m, lidar_x_m))

        self.xy_writer.writerow([
            f"{t_sec:.6f}",
            f"{odom_x_m:.6f}",
            f"{lidar_x_m:.6f}",
        ])
        self.xy_file.flush()
            
    def finish_calibration(self):
        if len(self.samples) < 2:
            self.get_logger().warn("Not enough samples for odom calibration.")
            return

        odom = [s[1] for s in self.samples]
        lidar = [s[2] for s in self.samples]

        denom = sum(x * x for x in odom)

        if denom <= 0.0:
            self.get_logger().warn("Cannot compute scale factor: odom denominator is zero.")
            return

        scale = sum(x * y for x, y in zip(odom, lidar)) / denom

        errors = [y - scale * x for x, y in zip(odom, lidar)]
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))

        last_odom = odom[-1]
        last_lidar = lidar[-1]
        last_ratio = last_lidar / last_odom if abs(last_odom) > 0.001 else float("nan")

        text = (
            f"odom calibration summary\n"
            f"time: {datetime.now().isoformat(timespec='seconds')}\n"
            f"samples: {len(self.samples)}\n"
            f"scale_lidar_per_odom: {scale:.6f}\n"
            f"last_ratio: {last_ratio:.6f}\n"
            f"rmse_m: {rmse:.6f}\n"
            f"last_odom_x_m: {last_odom:.6f}\n"
            f"last_lidar_x_m: {last_lidar:.6f}\n"
            f"csv_xy: {self.log_file_xy}\n"
        )

        self.get_logger().info("\n" + text)

        with open(self.log_file_summary, "w") as f:
            f.write(text)

def main(args=None):
    rclpy.init(args=args)
    node = CrawlerOdomCalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
        finally:
            try:
                node.finish_calibration()
            finally:
                node.close_log()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    main()