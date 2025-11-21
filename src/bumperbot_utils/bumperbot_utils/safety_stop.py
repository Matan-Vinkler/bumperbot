#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from visualization_msgs.msg import Marker, MarkerArray

from enum import Enum
import math

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop")

        self.declare_parameter("danger_distance", 0.2)
        self.declare_parameter("warning_distance", 0.6)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")

        self.danger_distance_ = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance_ = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic_ = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic_ = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.laser_sub_ = self.create_subscription(LaserScan, self.scan_topic_, self.laserCallback, 10)
        self.safety_stop_pub_ = self.create_publisher(Bool, self.safety_stop_topic_, 10)
        self.zones_pub_ = self.create_publisher(MarkerArray, "zones", 10)

        self.decrease_speed_client_ = ActionClient(self, JoyTurbo, "joy_turbo_decrease")
        self.increase_speed_client_ = ActionClient(self, JoyTurbo, "joy_turbo_increase")

        timeout_sec = 5.0

        while not self.decrease_speed_client_.wait_for_server(timeout_sec=timeout_sec) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_decrease not available! Waiting...")

        while not self.increase_speed_client_.wait_for_server(timeout_sec=timeout_sec) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_increase not available! Waiting...")

        self.state_ = State.FREE
        self.prev_state_ = State.FREE
        self.is_first_msg_ = True

        self.zones_ = MarkerArray()

        warning_zone = Marker()
        warning_zone.id = 0
        warning_zone.action = Marker.ADD
        warning_zone.type = Marker.CYLINDER
        warning_zone.scale.z = 0.001
        warning_zone.scale.x = self.warning_distance_ * 2
        warning_zone.scale.y = self.warning_distance_ * 2
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5

        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.action = Marker.ADD
        danger_zone.type = Marker.CYLINDER
        danger_zone.scale.z = 0.001
        danger_zone.scale.x = self.danger_distance_ * 2
        danger_zone.scale.y = self.danger_distance_ * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5
        danger_zone.pose.position.z = 0.01

        self.zones_.markers = [warning_zone, danger_zone]

    def laserCallback(self, msg: LaserScan):
        self.state_ = State.FREE

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.warning_distance_:
                self.state_ = State.WARNING
                if range_value <= self.danger_distance_:
                    self.state_ = State.DANGER
                    break

        if self.state_ != self.prev_state_:
            is_safety_stop = Bool()
            if self.state_ == State.WARNING:
                is_safety_stop.data = False
                self.zones_.markers[0].color.a = 1.0
                self.zones_.markers[1].color.a = 0.5
                self.decrease_speed_client_.send_goal_async(JoyTurbo.Goal())
            elif self.state_ == State.DANGER:
                is_safety_stop.data = True
                self.zones_.markers[0].color.a = 1.0
                self.zones_.markers[1].color.a = 1.0
            elif self.state_ == State.FREE:
                is_safety_stop.data = False
                self.zones_.markers[0].color.a = 0.5
                self.zones_.markers[1].color.a = 0.5
                self.increase_speed_client_.send_goal_async(JoyTurbo.Goal())
            
            self.safety_stop_pub_.publish(is_safety_stop)
            self.prev_state_ = self.state_

        if self.is_first_msg_:
            for zone in self.zones_.markers:
                zone.header.frame_id = msg.header.frame_id
            self.is_first_msg_ = False

        self.zones_pub_.publish(self.zones_)

def main():
    rclpy.init()

    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()