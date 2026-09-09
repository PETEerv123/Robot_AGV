#!/usr/bin/env python3

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid

from .manual import ManualTab
from .object_track import ObjectTrackTab

# ============================================================
# ROS2 CONTROLLER
# ============================================================


class RobotController(Node):
    def __init__(self):

        super().__init__("robot_gui")

        # ====================================================
        # MANUAL COMMAND
        # ====================================================

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_manual", 10)

        # ====================================================
        # ODOMETRY
        # ====================================================

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )

        # ====================================================
        # MAP
        # ====================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # ====================================================
        # COMMAND STATE
        # ====================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # ====================================================
        # REAL ROBOT VELOCITY
        # ====================================================

        self.feedback_vx = 0.0
        self.feedback_vy = 0.0
        self.feedback_wz = 0.0

        # ====================================================
        # ROBOT POSE
        # ====================================================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # ====================================================
        # MAP DATA
        # ====================================================

        self.map_data = None

        self.map_width = 0
        self.map_height = 0

        self.map_resolution = 0.05

        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.get_logger().info("Robot GUI ROS2 Node started")

    # ========================================================
    # SEND COMMAND
    # ========================================================

    def send_cmd(self, vx, vy, wz):

        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)

        self.cmd_pub.publish(msg)

        # Save command
        self.cmd_vx = float(vx)
        self.cmd_vy = float(vy)
        self.cmd_wz = float(wz)

    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        self.send_cmd(0.0, 0.0, 0.0)

    # ========================================================
    # ODOM CALLBACK
    # ========================================================

    def odom_callback(self, msg):

        # ----------------------------------------------------
        # REAL VELOCITY
        # ----------------------------------------------------

        self.feedback_vx = msg.twist.twist.linear.x

        self.feedback_vy = msg.twist.twist.linear.y

        self.feedback_wz = msg.twist.twist.angular.z

        # ----------------------------------------------------
        # POSITION
        # ----------------------------------------------------

        self.robot_x = msg.pose.pose.position.x

        self.robot_y = msg.pose.pose.position.y

        # ----------------------------------------------------
        # QUATERNION -> YAW
        # ----------------------------------------------------

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)

        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.robot_yaw = math.atan2(sin_yaw, cos_yaw)

    # ========================================================
    # MAP CALLBACK
    # ========================================================

    def map_callback(self, msg):

        self.map_data = msg.data

        self.map_width = msg.info.width

        self.map_height = msg.info.height

        self.map_resolution = msg.info.resolution

        self.map_origin_x = msg.info.origin.position.x

        self.map_origin_y = msg.info.origin.position.y


# ============================================================
# ROBOT GUI
# ============================================================


class RobotGUI:
    def __init__(self, root, controller):

        self.root = root

        self.controller = controller

        # ====================================================
        # WINDOW
        # ====================================================

        self.root.title("Robot AGV")

        self.root.geometry("1200x750")

        self.root.minsize(1000, 650)

        self.root.configure(bg="#111827")

        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # ====================================================
        # HEADER
        # ====================================================

        self.header = tk.Frame(self.root, bg="#1f2937", height=70)

        self.header.pack(fill="x")

        self.header.pack_propagate(False)

        # ----------------------------------------------------
        # TITLE
        # ----------------------------------------------------

        tk.Label(
            self.header,
            text="ROBOT AGV",
            bg="#1f2937",
            fg="white",
            font=("Arial", 22, "bold"),
        ).pack(side="left", padx=25)

        # ----------------------------------------------------
        # ROS STATUS
        # ----------------------------------------------------

        self.connection = tk.Label(
            self.header,
            text="● ROS2 CONNECTED",
            bg="#1f2937",
            fg="#22c55e",
            font=("Arial", 12, "bold"),
        )

        self.connection.pack(side="right", padx=25)

        # ====================================================
        # MODE BAR
        # ====================================================

        mode_bar = tk.Frame(self.root, bg="#111827", height=55)

        mode_bar.pack(fill="x")

        mode_bar.pack_propagate(False)

        tk.Label(
            mode_bar,
            text="MODE",
            bg="#111827",
            fg="#d1d5db",
            font=("Arial", 11, "bold"),
        ).pack(side="left", padx=(20, 10))

        # ----------------------------------------------------
        # MODE COMBOBOX
        # ----------------------------------------------------

        self.mode = tk.StringVar(value="IDLE")

        self.mode_combo = ttk.Combobox(
            mode_bar,
            textvariable=self.mode,
            values=["IDLE", "MANUAL", "OBJECT TRACK"],
            state="readonly",
            width=15,
        )

        self.mode_combo.pack(side="left")

        self.mode_combo.bind("<<ComboboxSelected>>", self.mode_changed)

        # ====================================================
        # CONTENT
        # ====================================================

        self.content = tk.Frame(self.root, bg="#111827")

        self.content.pack(fill="both", expand=True, padx=15, pady=(5, 15))

        # ====================================================
        # DASHBOARD
        # ====================================================

        self.dashboard_frame = tk.Frame(self.content, bg="#111827")

        # ====================================================
        # MANUAL
        # ====================================================

        self.manual = ManualTab(self.content, self.controller)
        # ====================================================
        # Object Track
        # ====================================================
        self.object_track = ObjectTrackTab(self.content, self.controller)
        # ====================================================
        # START IN IDLE
        # ====================================================

        self.show_dashboard()

        self.manual.set_enabled(False)
        self.object_track.set_enabled(False)
        self.controller.stop()

        # ====================================================
        # UPDATE
        # ====================================================

        self.update_gui()

    # ========================================================
    # MODE CHANGED
    # ========================================================

    def mode_changed(self, event=None):

        mode = self.mode.get()

        # Ẩn TẤT CẢ tab trước
        self.dashboard_frame.pack_forget()
        self.manual.pack_forget()
        self.object_track.pack_forget()

        # ====================================================
        # IDLE
        # ====================================================

        if mode == "IDLE":
            self.controller.stop()

            self.manual.set_enabled(False)

            self.object_track.set_enabled(False)

            self.dashboard_frame.pack(fill="both", expand=True)

            self.build_dashboard()

        # ====================================================
        # MANUAL
        # ====================================================

        elif mode == "MANUAL":
            self.manual.pack(fill="both", expand=True)

            self.manual.set_enabled(True)

        # ====================================================
        # OBJECT TRACK
        # ====================================================

        elif mode == "OBJECT TRACK":
            self.object_track.pack(fill="both", expand=True)

    def show_dashboard(self):

        # Ẩn tất cả
        self.dashboard_frame.pack_forget()
        self.manual.pack_forget()
        self.object_track.pack_forget()

        # Hiện Dashboard
        self.dashboard_frame.pack(fill="both", expand=True)

        self.build_dashboard()

    # ========================================================
    # SHOW MANUAL
    # ========================================================

    def show_manual(self):

        self.dashboard_frame.pack_forget()
        self.manual.pack_forget()
        self.object_track.pack_forget()

        self.manual.pack(fill="both", expand=True)

    # ========================================================
    # BUILD DASHBOARD
    # ========================================================

    def build_dashboard(self):

        # Clear dashboard
        for widget in self.dashboard_frame.winfo_children():
            widget.destroy()

        # ----------------------------------------------------
        # TITLE
        # ----------------------------------------------------

        tk.Label(
            self.dashboard_frame,
            text="DASHBOARD",
            bg="#111827",
            fg="white",
            font=("Arial", 24, "bold"),
        ).pack(pady=(15, 25))

        # ----------------------------------------------------
        # STATE AREA
        # ----------------------------------------------------

        state = tk.Frame(self.dashboard_frame, bg="#1f2937", padx=25, pady=20)

        state.pack(fill="x", padx=20)

        tk.Label(
            state,
            text="ROBOT STATE",
            bg="#1f2937",
            fg="white",
            font=("Arial", 15, "bold"),
        ).pack(anchor="w", pady=(0, 15))

        values = tk.Frame(state, bg="#1f2937")

        values.pack(fill="x")

        self.dashboard_labels = {}

        fields = [
            ("X", "m"),
            ("Y", "m"),
            ("Yaw", "deg"),
            ("Vx", "m/s"),
            ("Vy", "m/s"),
            ("Wz", "rad/s"),
        ]

        for i, (name, unit) in enumerate(fields):
            box = tk.Frame(values, bg="#111827", padx=15, pady=10)

            box.grid(row=0, column=i, padx=5, sticky="ew")

            values.grid_columnconfigure(i, weight=1)

            tk.Label(
                box, text=name, bg="#111827", fg="#9ca3af", font=("Arial", 10, "bold")
            ).pack()

            value = tk.Label(
                box,
                text=f"0.00 {unit}",
                bg="#111827",
                fg="white",
                font=("Arial", 13, "bold"),
            )

            value.pack(pady=(5, 0))

            self.dashboard_labels[name] = value

        # ----------------------------------------------------
        # INFO
        # ----------------------------------------------------

        info = tk.Frame(self.dashboard_frame, bg="#1f2937", padx=25, pady=20)

        info.pack(fill="x", padx=20, pady=20)

        tk.Label(
            info, text="ROS2", bg="#1f2937", fg="#9ca3af", font=("Arial", 10, "bold")
        ).grid(row=0, column=0, sticky="w")

        tk.Label(
            info,
            text="Connected",
            bg="#1f2937",
            fg="#22c55e",
            font=("Arial", 11, "bold"),
        ).grid(row=1, column=0, sticky="w", pady=(3, 10))

        tk.Label(
            info, text="Map", bg="#1f2937", fg="#9ca3af", font=("Arial", 10, "bold")
        ).grid(row=0, column=1, padx=80, sticky="w")

        self.dashboard_map_status = tk.Label(
            info,
            text="Waiting for /map",
            bg="#1f2937",
            fg="#f59e0b",
            font=("Arial", 11, "bold"),
        )

        self.dashboard_map_status.grid(row=1, column=1, padx=80, sticky="w")

        tk.Label(
            info,
            text="Current Mode",
            bg="#1f2937",
            fg="#9ca3af",
            font=("Arial", 10, "bold"),
        ).grid(row=0, column=2, sticky="w")

        self.dashboard_mode_status = tk.Label(
            info, text="IDLE", bg="#1f2937", fg="white", font=("Arial", 11, "bold")
        )

        self.dashboard_mode_status.grid(row=1, column=2, sticky="w")

    # ========================================================
    # UPDATE DASHBOARD
    # ========================================================

    def update_dashboard(self):

        if not hasattr(self, "dashboard_labels"):
            return

        c = self.controller

        self.dashboard_labels["X"].config(text=f"{c.robot_x:.3f} m")

        self.dashboard_labels["Y"].config(text=f"{c.robot_y:.3f} m")

        self.dashboard_labels["Yaw"].config(text=f"{math.degrees(c.robot_yaw):.1f}°")

        self.dashboard_labels["Vx"].config(text=f"{c.feedback_vx:.3f} m/s")

        self.dashboard_labels["Vy"].config(text=f"{c.feedback_vy:.3f} m/s")

        self.dashboard_labels["Wz"].config(text=f"{c.feedback_wz:.3f} rad/s")

        if c.map_data is None:
            self.dashboard_map_status.config(text="Waiting for /map", fg="#f59e0b")

        else:
            self.dashboard_map_status.config(
                text=(f"{c.map_width} x {c.map_height} | {c.map_resolution:.3f} m"),
                fg="#22c55e",
            )

        self.dashboard_mode_status.config(text=self.mode.get())

    # ========================================================
    # UPDATE GUI
    # ========================================================

    def update_gui(self):

        if not rclpy.ok():
            return

        self.connection.config(text="● ROS2 CONNECTED", fg="#22c55e")

        self.update_dashboard()

        # Manual update only when displayed
        if self.mode.get() == "MANUAL":
            self.manual.update()

        self.root.after(50, self.update_gui)

    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        try:
            self.controller.stop()

            self.controller.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

        except Exception as e:
            print("Shutdown error:", e)

        self.root.destroy()


# ============================================================
# MAIN
# ============================================================


def main(args=None):

    rclpy.init(args=args)

    controller = RobotController()

    root = tk.Tk()

    gui = RobotGUI(root, controller)
    print("Robot GUI INIT")
    # ========================================================
    # ROS SPIN
    # ========================================================

    def ros_spin():

        if not rclpy.ok():
            return

        try:
            rclpy.spin_once(controller, timeout_sec=0)

        except Exception as e:
            print("ROS spin error:", e)

            return

        root.after(10, ros_spin)

    root.after(10, ros_spin)

    root.mainloop()


if __name__ == "__main__":
    main()
