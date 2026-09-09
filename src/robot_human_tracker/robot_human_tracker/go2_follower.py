#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool


class HumanFollower(Node):
    def __init__(self):
        super().__init__("human_follower")

        self.human_target_topic = "/human_tracker/target"
        self.cmd_vel_topic = "/cmd_vel_follow"
        self.cmd_follow_topic = "/human_tracker/follow_flag"
        # TARGET
        self.follow_flag = False

        # Khoảng cách robot muốn giữ với người
        self.TARGET_DISTANCE = 1.5  # m
        # self.TARGET_DISTANCE = 1.8  # m
        # Vùng chết theo khoảng cách
        self.DISTANCE_TOL = 0.10  # m

        # Vùng chết lateral
        self.LATERAL_TOL = 0.08  # m
        # SPEED

        self.K_DISTANCE = 0.5
        self.K_LATERAL = 0.3

        self.MAX_VX = 0.20
        self.MAX_VY = 0.20

        # TRACKER TIMEOUT

        self.TRACKER_TIMEOUT = 0.5

        self.last_target_time = time.monotonic()

        self.target_locked = False
        self.target_distance = 0.0
        self.target_lateral = 0.0

        # ROS

        self.sub_target = self.create_subscription(
            Vector3,
            self.human_target_topic,
            self.target_callback,
            10,
        )
        self.sub_follow = self.create_subscription(
            Bool,
            self.cmd_follow_topic,
            self.follow_callback,
            10,
        )
        self.pub_cmd = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        # 50 Hz
        self.control_timer = self.create_timer(
            0.02,
            self.control_loop,
        )

        self.get_logger().info("Human Follower started")

        self.get_logger().info(f"Target distance: {self.TARGET_DISTANCE:.2f} m")

    # TARGET CALLBACK

    def target_callback(self, msg):

        self.last_target_time = time.monotonic()
        if not self.follow_flag:
            self.stop_robot()
            return
        if msg.z > 0.5:
            self.target_locked = True

            # x = distance
            self.target_distance = float(msg.x)

            # y = lateral
            self.target_lateral = float(msg.y)

        else:
            self.target_locked = False

            self.target_distance = 0.0
            self.target_lateral = 0.0

    # CONTROL

    def control_loop(self):
        if not self.follow_flag:
            self.stop_robot()
            return
        now = time.monotonic()

        # Tracker timeout

        if now - self.last_target_time > self.TRACKER_TIMEOUT:
            self.stop_robot()

            return

        # No person

        if not self.target_locked:
            self.stop_robot()

            return

        distance = self.target_distance
        lateral = self.target_lateral

        # DISTANCE CONTROL

        distance_error = distance - self.TARGET_DISTANCE

        if abs(distance_error) <= self.DISTANCE_TOL:
            vx = 0.0

        else:
            vx = self.K_DISTANCE * distance_error

        # LATERAL CONTROL

        if abs(lateral) <= self.LATERAL_TOL:
            vy = 0.0

        else:
            # Nếu lateral > 0 nghĩa người bên trái
            # robot chạy sang trái
            vy = self.K_LATERAL * lateral

        # LIMIT SPEED

        vx = max(-self.MAX_VX, min(self.MAX_VX, vx))

        vy = max(-self.MAX_VY, min(self.MAX_VY, vy))
        # PUBLISH TWIST

        cmd = Twist()

        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)

        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)
        # DEBUG
        self.get_logger().info(
            f"Person: "
            f"D={distance:.2f}m "
            f"L={lateral:.2f}m | "
            f"Error={distance_error:.2f}m | "
            f"CMD "
            f"Vx={vx:.2f} "
            f"Vy={vy:.2f}"
        )

    # STOP

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)

    # DESTROY
    def destroy_node(self):

        self.stop_robot()

        super().destroy_node()

    def follow_callback(self, msg):
        self.follow_flag = msg.data
        self.get_logger().info(f"FOLLOW {'Enable' if self.follow_flag else 'Disnable'}")


def main(args=None):

    rclpy.init(args=args)

    node = HumanFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
