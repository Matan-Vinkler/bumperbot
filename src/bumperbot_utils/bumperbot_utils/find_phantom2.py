#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import math
import os


class PhantomScanFinder(Node):
    def __init__(self):
        super().__init__("find_phantom2")

        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("num_scans", 30)
        self.declare_parameter("search_min_range", 1.8)
        self.declare_parameter("search_max_range", 2.7)
        self.declare_parameter("presence_ratio", 0.8)
        self.declare_parameter("max_candidates", 40)

        self.scan_topic_ = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.num_scans_ = self.get_parameter("num_scans").get_parameter_value().integer_value
        self.search_min_range_ = self.get_parameter("search_min_range").get_parameter_value().double_value
        self.search_max_range_ = self.get_parameter("search_max_range").get_parameter_value().double_value
        self.presence_ratio_ = self.get_parameter("presence_ratio").get_parameter_value().double_value
        self.max_candidates_ = self.get_parameter("max_candidates").get_parameter_value().integer_value

        self.count_ = 0
        self.samples_ = {}

        self.sub_ = self.create_subscription(LaserScan, self.scan_topic_, self.scan_callback, 10)

    def scan_callback(self, msg):
        self.count_ += 1
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            self.samples_.setdefault(i, []).append(r)
        self.get_logger().info(f"scan {self.count_}/{self.num_scans_} received")

        if self.count_ < self.num_scans_:
            return

        self.get_logger().info(
            f"--- after {self.count_} scans, looking for low-variance returns "
            f"between {self.search_min_range_}m and {self.search_max_range_}m ---"
        )
        candidates = []
        for i, vals in self.samples_.items():
            if len(vals) < self.count_ * self.presence_ratio_:
                continue
            mean = sum(vals) / len(vals)
            if not (self.search_min_range_ <= mean <= self.search_max_range_):
                continue
            variance = sum((v - mean) ** 2 for v in vals) / len(vals)
            candidates.append((i, mean, math.sqrt(variance), len(vals)))
        candidates.sort(key=lambda c: c[2])

        if not candidates:
            self.get_logger().info(
                f"No low-variance candidates found between {self.search_min_range_}m and {self.search_max_range_}m"
            )
        for i, mean, std, n in candidates[: self.max_candidates_]:
            angle = msg.angle_min + i * msg.angle_increment
            self.get_logger().info(
                f"index={i} angle={angle:.3f}rad ({math.degrees(angle):.1f}deg) "
                f"mean_range={mean:.3f}m std={std:.4f} n={n}/{self.count_}"
            )

        # rclpy.shutdown() from inside a subscription callback hangs the executor;
        # os._exit is the reliable way to stop this one-shot diagnostic.
        os._exit(0)


def main():
    rclpy.init()
    node = PhantomScanFinder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
