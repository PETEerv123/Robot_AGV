#!/usr/bin/env python3

import tkinter as tk

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ============================================================
# TOPICS
# ============================================================


CAMERA_TOPIC = "/camera/camera/color/image_raw"
OUTPUT_TOPIC = "/human_tracker/output"
LOCK_ID_TOPIC = "/human_tracker/lock_id"
MODE_TOPIC = "/human_tracker/mode"
FOLLOW_ENABLE_TOPIC = "/human_tracker/follow_flag"


class ObjectTrackTab(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)

        self.controller = controller
        self.bridge = CvBridge()

        # ====================================================
        # STATE
        # ====================================================

        self.locked_id = None
        self.follow_enable = False
        self.mode = "IDLE"

        # Camera RAW
        self.raw_frame = None

        # Tracker OUTPUT
        self.output_frame = None

        # Tkinter image references
        self.raw_photo = None
        self.output_photo = None
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # ====================================================
        # ROS PUBLISHERS
        # ====================================================

        self.lock_id_pub = controller.create_publisher(Int32, LOCK_ID_TOPIC, 10)

        self.mode_pub = controller.create_publisher(String, MODE_TOPIC, 10)

        self.follow_pub = controller.create_publisher(Bool, FOLLOW_ENABLE_TOPIC, 10)

        # ---------------- RAW CAMERA ----------------

        self.camera_sub = controller.create_subscription(
            Image, CAMERA_TOPIC, self.camera_callback, image_qos
        )

        # ---------------- TRACKER OUTPUT ----------------

        self.output_sub = controller.create_subscription(
            Image, OUTPUT_TOPIC, self.output_callback, image_qos
        )

        self.create_gui()

    # ========================================================
    # CREATE GUI
    # ========================================================

    def create_gui(self):

        # ====================================================
        # CONTROL FRAME
        # ====================================================

        control_frame = tk.Frame(self)

        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        # ----------------------------------------------------
        # TARGET ID
        # ----------------------------------------------------

        tk.Label(control_frame, text="Target ID:").pack(side=tk.LEFT, padx=(0, 5))

        self.id_entry = tk.Entry(control_frame, width=8)

        self.id_entry.pack(side=tk.LEFT, padx=(0, 10))

        # ----------------------------------------------------
        # SEARCHING
        # ----------------------------------------------------

        self.search_btn = tk.Button(
            control_frame, text="SEARCHING", width=12, command=self.on_searching
        )

        self.search_btn.pack(side=tk.LEFT, padx=3)

        # ----------------------------------------------------
        # SEND ID
        # ----------------------------------------------------

        self.send_id_btn = tk.Button(
            control_frame, text="LOCK ID", width=10, command=self.on_send_id
        )

        self.send_id_btn.pack(side=tk.LEFT, padx=3)

        # ----------------------------------------------------
        # STOP
        # ----------------------------------------------------

        self.stop_btn = tk.Button(
            control_frame, text="STOP", width=10, command=self.on_stop
        )

        self.stop_btn.pack(side=tk.LEFT, padx=3)

        # ----------------------------------------------------
        # XÓA ID
        # ----------------------------------------------------

        self.delete_btn = tk.Button(
            control_frame, text="DELETE ID", width=10, command=self.on_delete_id
        )

        self.delete_btn.pack(side=tk.LEFT, padx=3)

        # ----------------------------------------------------
        # TRACK
        # ----------------------------------------------------

        self.track_btn = tk.Button(
            control_frame, text="TRACK", width=10, command=self.on_track
        )

        self.track_btn.pack(side=tk.LEFT, padx=3)

        # ====================================================
        # STATUS
        # ====================================================

        status_frame = tk.Frame(self)

        status_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 10))

        self.mode_label = tk.Label(status_frame, text="MODE: IDLE")

        self.mode_label.pack(side=tk.LEFT, padx=10)

        self.id_label = tk.Label(status_frame, text="ID: None")

        self.id_label.pack(side=tk.LEFT, padx=10)

        self.follow_label = tk.Label(status_frame, text="FOLLOW: OFF")

        self.follow_label.pack(side=tk.LEFT, padx=10)

        # ====================================================
        # IMAGE AREA
        # ====================================================

        image_frame = tk.Frame(self)

        image_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ----------------------------------------------------
        # RAW CAMERA
        # ----------------------------------------------------

        raw_frame = tk.Frame(image_frame)

        raw_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        tk.Label(raw_frame, text="CAMERA RAW").pack(side=tk.TOP, pady=5)

        self.raw_image_label = tk.Label(raw_frame, text="Waiting for camera...")

        self.raw_image_label.pack(fill=tk.BOTH, expand=True)

        # ----------------------------------------------------
        # TRACKER OUTPUT
        # ----------------------------------------------------

        output_frame = tk.Frame(image_frame)

        output_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        tk.Label(output_frame, text="HUMAN TRACKER").pack(side=tk.TOP, pady=5)

        self.output_image_label = tk.Label(
            output_frame, text="Waiting for tracker output..."
        )

        self.output_image_label.pack(fill=tk.BOTH, expand=True)

    # ========================================================
    # SEARCHING
    # ========================================================

    def on_searching(self):

        mode_msg = String()
        mode_msg.data = "SEARCHING"
        self.mode = "SEARCHING"

        self.mode_pub.publish(mode_msg)

        self.publish_follow(False)
        self.update_status()

        self.controller.get_logger().info("GUI: SEARCHING")

    # ========================================================
    # SEND ID
    # ========================================================

    def on_send_id(self):

        text = self.id_entry.get().strip()

        if not text:
            self.controller.get_logger().warn("Target ID is empty")
            return

        try:
            target_id = int(text)

        except ValueError:
            self.controller.get_logger().warn(f"Invalid Target ID: {text}")
            return

        if target_id <= 0:
            self.controller.get_logger().warn("Target ID must be > 0")
            return

        msg = Int32()
        msg.data = target_id

        self.lock_id_pub.publish(msg)

        self.locked_id = target_id

        self.update_status()

        self.controller.get_logger().info(f"GUI: LOCK ID = {target_id}")

    # ========================================================
    # STOP
    # ========================================================

    def on_stop(self):

        # MODE -> IDLE
        mode_msg = String()
        mode_msg.data = "IDLE"
        self.mode = "IDLE"
        self.mode_pub.publish(mode_msg)

        self.publish_follow(False)

        # Local state
        self.locked_id = None
        self.follow_enable = False

        self.id_entry.delete(0, tk.END)

        self.update_status()

        self.controller.get_logger().info("GUI: STOP -> IDLE")

    # ========================================================
    # DELETE ID
    # ========================================================

    def on_delete_id(self):

        # -1 = delete current ID
        msg = Int32()
        msg.data = -1

        self.lock_id_pub.publish(msg)

        self.locked_id = None

        # Delete ID -> SEARCHING
        mode_msg = String()
        mode_msg.data = "SEARCHING"
        self.mode = "SEARCHING"
        self.mode_pub.publish(mode_msg)

        self.id_entry.delete(0, tk.END)
        self.publish_follow(False)
        # Không thay đổi follow_enable
        self.update_status()

        self.controller.get_logger().info("GUI: DELETE ID -> SEARCHING")

    # ========================================================
    # TRACK
    # ========================================================

    def on_track(self):

        # TRACK chỉ gửi follow_flag = TRUE

        self.publish_follow(True)

        self.update_status()

        self.controller.get_logger().info("GUI: TRACK -> follow_flag = TRUE")

    # ========================================================
    # CAMERA RAW CALLBACK
    # ========================================================

    def camera_callback(self, msg):

        try:
            self.raw_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Đưa việc cập nhật Tkinter về main thread
            self.after(0, self.update_raw_camera)

        except Exception as e:
            self.controller.get_logger().error(f"RAW camera conversion error: {e}")

    # ========================================================
    # TRACKER OUTPUT CALLBACK
    # ========================================================

    def output_callback(self, msg):

        try:
            self.output_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.after(0, self.update_tracker_output)

        except Exception as e:
            self.controller.get_logger().error(f"Tracker output conversion error: {e}")

    # ========================================================
    # UPDATE RAW CAMERA
    # ========================================================

    def update_raw_camera(self):

        if self.raw_frame is None:
            return

        self.display_frame(self.raw_frame, self.raw_image_label, "raw")

    # ========================================================
    # UPDATE TRACKER OUTPUT
    # ========================================================

    def update_tracker_output(self):

        if self.output_frame is None:
            return

        self.display_frame(self.output_frame, self.output_image_label, "output")

    # ========================================================
    # DISPLAY FRAME
    # ========================================================

    def display_frame(self, frame, label, image_type):

        try:
            from PIL import Image
            from PIL import ImageTk

            frame = frame.copy()

            height, width = frame.shape[:2]

            # Kích thước tối đa mỗi panel
            max_width = 640
            max_height = 480

            scale = min(max_width / width, max_height / height, 1.0)

            if scale < 1.0:
                frame = cv2.resize(
                    frame,
                    (int(width * scale), int(height * scale)),
                    interpolation=cv2.INTER_AREA,
                )

            # BGR -> RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            image = Image.fromarray(frame)

            photo = ImageTk.PhotoImage(image=image)

            label.configure(image=photo, text="")

            # Phải giữ reference
            if image_type == "raw":
                self.raw_photo = photo

            else:
                self.output_photo = photo

        except Exception as e:
            self.controller.get_logger().error(f"Display image error: {e}")

    # ========================================================
    # UPDATE STATUS
    # ========================================================

    def update_status(self):

        if self.locked_id is None:
            self.id_label.configure(text="ID: None")

        else:
            self.id_label.configure(text=f"ID: {self.locked_id}")

        if self.follow_enable:
            self.follow_label.configure(text="FOLLOW: ON")

        else:
            self.follow_label.configure(text="FOLLOW: OFF")

        self.mode_label.configure(text=f"MODE: {self.mode}")

    # ========================================================
    # COMPATIBILITY
    # ========================================================

    def set_enabled(self, enabled):
        """
        Giữ lại để tương thích với robot_gui.py.
        Không dùng active để chặn camera callback.
        """

        if enabled:
            self.raw_image_label.configure(text="Waiting for camera...")

            self.output_image_label.configure(text="Waiting for tracker output...")

        else:
            self.raw_frame = None
            self.output_frame = None

            self.raw_photo = None
            self.output_photo = None

            self.raw_image_label.configure(image="", text="Object Track inactive")

            self.output_image_label.configure(image="", text="Object Track inactive")

    def publish_follow(self, enable):
        msg = Bool()
        msg.data = bool(enable)

        self.follow_pub.publish(msg)
        self.follow_enable = bool(enable)

        self.controller.get_logger().info(f"GUI: follow_flag = {self.follow_enable}")

        self.update_status()

    def update(self):
        """
        robot_gui.py đang gọi update().
        """

        self.update_status()
