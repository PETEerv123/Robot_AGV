#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class HumanFollower(Node):
    def __init__(self):
        super().__init__("human_follower")

        # ============================================================
        # TOPICS
        # ============================================================

        self.human_target_topic = "/human_tracker/target"
        self.cmd_follow_topic = "/human_tracker/follow_flag"
        self.NavigateToPoseTopic = "/navigate_to_pose"

        self.follow_flag = False

        # ============================================================
        # FOLLOW PARAMETERS
        # ============================================================

        self.TARGET_DISTANCE = 1.2

        self.DISTANCE_TOL = 0.10
        self.LATERAL_TOL = 0.08

        # Nếu tracker mất target quá thời gian này -> cancel goal
        self.TRACKER_TIMEOUT = 2.0
        self.last_target_time = time.monotonic()

        # ============================================================
        # TARGET
        # ============================================================

        self.target_locked = False

        self.target_distance = 0.0
        self.target_lateral = 0.0

        # ============================================================
        # NAV2
        # ============================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.NavigateToPoseTopic,
        )

        # Goal hiện tại
        self.goal_handle = None
        self.current_goal = None

        # Không gửi goal liên tục 10 Hz
        self.last_goal_time = 0.0

        # Tối thiểu bao lâu mới cập nhật goal
        self.GOAL_UPDATE_INTERVAL = 0.5

        # Nếu target mới thay đổi quá ít thì không cần gửi goal mới
        self.GOAL_UPDATE_THRESHOLD = 0.20

        # Lưu target local trước đó
        self.last_local_goal = None

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

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

        # ============================================================
        # CONTROL LOOP
        # ============================================================

        self.control_timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        self.get_logger().info("Human Follower + Nav2 started")

    # ================================================================
    # TARGET CALLBACK
    # ================================================================

    def target_callback(self, msg):

        self.last_target_time = time.monotonic()

        if not self.follow_flag:
            return

        # msg.z > 0.5 = target hợp lệ
        if msg.z > 0.5:
            self.target_locked = True

            # Tracker:
            # x = distance
            # y = lateral

            self.target_distance = float(msg.x)
            self.target_lateral = float(msg.y)

        else:
            self.target_locked = False

    # ================================================================
    # FOLLOW FLAG
    # ================================================================

    def follow_callback(self, msg):

        self.follow_flag = msg.data

        self.get_logger().info(f"FOLLOW {'Enable' if self.follow_flag else 'Disable'}")

        if not self.follow_flag:
            self.target_locked = False

            self.cancel_nav_goal()

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # FOLLOW OFF
        # ------------------------------------------------------------

        if not self.follow_flag:
            return

        now = time.monotonic()

        # ------------------------------------------------------------
        # TRACKER TIMEOUT
        # ------------------------------------------------------------

        if now - self.last_target_time > self.TRACKER_TIMEOUT:
            self.get_logger().warn("Tracker timeout -> cancel Nav2 goal")

            self.cancel_nav_goal()

            return

        # ------------------------------------------------------------
        # NO TARGET
        # ------------------------------------------------------------

        if not self.target_locked:
            self.cancel_nav_goal()

            return

        # ------------------------------------------------------------
        # TARGET DATA
        # ------------------------------------------------------------

        distance = self.target_distance
        lateral = self.target_lateral

        # ============================================================
        # TÍNH ĐIỂM ROBOT CẦN ĐI ĐẾN
        #
        # Tracker:
        #
        #   x = khoảng cách phía trước
        #   y = lệch trái/phải
        #
        # Robot cần đứng cách người:
        #
        #   TARGET_DISTANCE = 1.5 m
        #
        # Ví dụ:
        #
        #   distance = 2.5
        #
        #   goal_x = 2.5 - 1.5
        #           = 1.0 m
        #
        # => robot tiến 1 m
        #
        # ============================================================

        goal_x = distance - self.TARGET_DISTANCE
        goal_y = lateral

        # ------------------------------------------------------------
        # ĐÃ ĐẾN VỊ TRÍ MONG MUỐN
        # ------------------------------------------------------------

        if abs(goal_x) <= self.DISTANCE_TOL and abs(goal_y) <= self.LATERAL_TOL:
            self.cancel_nav_goal()

            self.last_local_goal = None

            return

        # ------------------------------------------------------------
        # KHÔNG GỬI GOAL LIÊN TỤC
        # ------------------------------------------------------------

        if now - self.last_goal_time < self.GOAL_UPDATE_INTERVAL:
            return

        # ------------------------------------------------------------
        # TARGET CHƯA THAY ĐỔI ĐỦ NHIỀU
        # ------------------------------------------------------------

        if self.last_local_goal is not None:
            old_x, old_y = self.last_local_goal

            delta = math.sqrt((goal_x - old_x) ** 2 + (goal_y - old_y) ** 2)

            if delta < self.GOAL_UPDATE_THRESHOLD:
                return

        # ------------------------------------------------------------
        # SEND
        # ------------------------------------------------------------

        self.send_follow_goal(
            goal_x,
            goal_y,
        )

    # ================================================================
    # SEND NAV2 GOAL
    # ================================================================

    def send_follow_goal(self, x, y):

        # ------------------------------------------------------------
        # NAV2 CHECK
        # ------------------------------------------------------------

        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready")

            return

        # ------------------------------------------------------------
        # TF MAP -> BASE_FOOTPRINT
        # ------------------------------------------------------------

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
            )

        except Exception as e:
            self.get_logger().warn(f"TF map -> base_footprint unavailable: {e}")

            return

        # ============================================================
        # LOCAL GOAL
        #
        # Goal này đang nằm trong base_footprint
        #
        # x = tiến/lùi
        # y = trái/phải
        # ============================================================

        local_pose = PoseStamped()

        local_pose.header.frame_id = "base_footprint"

        local_pose.header.stamp = self.get_clock().now().to_msg()

        local_pose.pose.position.x = x
        local_pose.pose.position.y = y
        local_pose.pose.position.z = 0.0

        # ------------------------------------------------------------
        # ROBOT HƯỚNG VỀ TARGET
        # ------------------------------------------------------------

        yaw = math.atan2(y, x)

        local_pose.pose.orientation.x = 0.0
        local_pose.pose.orientation.y = 0.0

        local_pose.pose.orientation.z = math.sin(yaw / 2.0)

        local_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # ============================================================
        # TRANSFORM
        #
        # QUAN TRỌNG:
        #
        # do_transform_pose() trên ROS2 Humble cần Pose,
        # KHÔNG truyền PoseStamped trực tiếp.
        #
        # ============================================================

        global_pose_data = do_transform_pose(
            local_pose.pose,
            transform,
        )

        # ------------------------------------------------------------
        # TẠO PoseStamped MAP
        # ------------------------------------------------------------

        global_pose = PoseStamped()

        global_pose.header.frame_id = "map"

        global_pose.header.stamp = self.get_clock().now().to_msg()

        global_pose.pose = global_pose_data

        # ============================================================
        # NAV2 GOAL
        # ============================================================

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = global_pose

        # ------------------------------------------------------------
        # LƯU GOAL
        # ------------------------------------------------------------

        self.current_goal = (
            global_pose.pose.position.x,
            global_pose.pose.position.y,
        )

        self.last_local_goal = (
            x,
            y,
        )

        self.last_goal_time = time.monotonic()

        # ------------------------------------------------------------
        # GỬI NAV2
        # ------------------------------------------------------------

        future = self.nav_client.send_goal_async(goal_msg)

        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(
            "Nav2 follow goal sent: "
            f"local=({x:.2f}, {y:.2f}) "
            f"map=("
            f"{global_pose.pose.position.x:.2f}, "
            f"{global_pose.pose.position.y:.2f})"
        )

    # ================================================================
    # NAV2 RESPONSE
    # ================================================================

    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()

        except Exception as e:
            self.get_logger().error(f"Nav2 goal failed: {e}")

            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")

            return

        self.goal_handle = goal_handle

        self.get_logger().info("Nav2 follow goal accepted")

    # ================================================================
    # CANCEL NAV2 GOAL
    # ================================================================

    def cancel_nav_goal(self):

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()

            except Exception:
                pass

            self.goal_handle = None

        self.current_goal = None

    # ================================================================
    # DESTROY
    # ================================================================

    def destroy_node(self):

        self.cancel_nav_goal()

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================


def main(args=None):

    rclpy.init(args=args)

    node = HumanFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
