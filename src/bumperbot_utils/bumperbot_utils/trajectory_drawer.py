#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryDrawer(Node):
    def __init__(self):
        super().__init__("trajectory_drawer")

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        self.traj_pub_ = self.create_publisher(Path, "bumperbot_controller/trajectory", 10)

        self.traj_msg_ = Path()
        self.traj_msg_.header.frame_id = "odom"

    def odomCallback(self, odom_msg: Odometry):
        pose = odom_msg.pose.pose

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "odom"

        self.traj_msg_.header.stamp = self.get_clock().now().to_msg()
        self.traj_msg_.poses.append(pose_stamped)

        self.traj_pub_.publish(self.traj_msg_)

def main():
    rclpy.init()

    trajectory_drawer = TrajectoryDrawer()
    rclpy.spin(trajectory_drawer)
    trajectory_drawer.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()