#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion

import math

PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self, px: float = 0.0, py: float = 0.0):
        self.x_ = px
        self.y_ = py

def coordinatesToPose(px: float, py: float, map_info: MapMetaData) -> Pose:
        pose = Pose()
        pose.x_ = float(round((px - map_info.origin.position.x) / map_info.resolution))
        pose.y_ = float(round((py - map_info.origin.position.y) / map_info.resolution))

        return pose

def poseOnMap(pose: Pose, map_info: MapMetaData) -> bool:
    return pose.x_ < map_info.width and pose.x_ >= 0 and pose.y_ < map_info.height and pose.y_ >= 0

def poseToCell(pose: Pose, map_info: MapMetaData) -> int:
    return map_info.width * pose.y_ + pose.x_

def bresenham(start: Pose, end: Pose) -> list[Pose]:
    line = []

    dx = end.x_ - start.x_
    dy = end.y_ - start.y_

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x_ + i * xx + y * yx, start.y_ + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(robot_p: Pose, beam_p: Pose) -> list[tuple[Pose, int]]:
    occ_values = []
    line = bresenham(robot_p, beam_p)
    
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))
    occ_values.append((line[-1], OCC_PROB))

    return occ_values

def prob2logodds(p: float) -> float:
    return math.log(p / (1 - p))

def logodds2prob(l: float) -> float:
    try:
        return 1 - 1 / (1 + math.exp(l))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

class MappingWithKnownPoses(Node):
    def __init__(self):
        super().__init__("mapping_with_known_poses")

        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").get_parameter_value().double_value()
        height = self.get_parameter("height").get_parameter_value().double_value()
        resolution = self.get_parameter("resolution").get_parameter_value().double_value()

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.prob_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        self.map_pub_ = self.create_publisher(OccupancyGrid, "map", 1)
        self.scan_sub_ = self.create_subscription(LaserScan, "scan", self.scanCallback, 10)
        self.timer_ = self.create_timer(1.0, self.timerCallback)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_)

    def scanCallback(self, scan_msg: LaserScan):
        try:
            t = self.tf_buffer_.lookup_transform(self.map_.header.frame_id, scan_msg.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().error("Unable to transform between /odom and /base_footprint")
            return
        
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().error("The robot is not on the map!")
            return
        
        _, _, yaw = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])

        for i in range(len(scan_msg.ranges)):
            if math.isinf(scan_msg.ranges[i]):
                continue
            
            angle = scan_msg.angle_min + (scan_msg.angle_increment * i) + yaw

            px = scan_msg.ranges[i] * math.cos(angle) + t.transform.translation.x
            py = scan_msg.ranges[i] * math.sin(angle) + t.transform.translation.y

            beam_p = coordinatesToPose(px, py, self.map_.info)
            if not poseOnMap(beam_p, self.map_.info):
                continue

            poses = inverseSensorModel(robot_p, beam_p)
            for pose, value in poses:
                cell = poseToCell(pose, self.map_.info)
                self.prob_map_[cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB)

    def timerCallback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.data = [int(logodds2prob(value) * 100) for value in self.prob_map_]
        self.map_pub_.publish(self.map_)

def main():
    rclpy.init()

    node = MappingWithKnownPoses()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()