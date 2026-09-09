#!/usr/bin/env python3

import math
import tkinter as tk


class ManualTab(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="#111827")

        self.controller = controller

        # Manual velocity limits
        self.max_vx = 0.15
        self.max_vy = 0.15
        self.max_wz = 3.0

        self.enabled = True

        # ----------------------------------------------------
        # ROOT LAYOUT
        # ----------------------------------------------------

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)

        # ====================================================
        # LEFT PANEL
        # ====================================================

        left = tk.Frame(self, bg="#111827", width=360)
        left.grid(row=0, column=0, sticky="ns", padx=(15, 10), pady=15)
        left.grid_propagate(False)

        # ----------------------------------------------------
        # VELOCITY LIMIT
        # ----------------------------------------------------

        velocity_frame = tk.Frame(left, bg="#1f2937", padx=15, pady=12)
        velocity_frame.pack(fill="x", pady=(0, 15))

        tk.Label(
            velocity_frame,
            text="VELOCITY LIMIT",
            bg="#1f2937",
            fg="white",
            font=("Arial", 14, "bold"),
        ).pack(anchor="w", pady=(0, 8))

        self.vx_var = tk.DoubleVar(value=self.max_vx)
        self.vy_var = tk.DoubleVar(value=self.max_vy)
        self.wz_var = tk.DoubleVar(value=self.max_wz)

        self.vx_scale, self.vx_value_label = self._create_scale(
            velocity_frame, "Vx", self.vx_var, 1.0, 0.05, "m/s", self.vx_changed
        )

        self.vy_scale, self.vy_value_label = self._create_scale(
            velocity_frame, "Vy", self.vy_var, 1.0, 0.05, "m/s", self.vy_changed
        )

        self.wz_scale, self.wz_value_label = self._create_scale(
            velocity_frame, "Wz", self.wz_var, 5.0, 0.05, "rad/s", self.wz_changed
        )

        # ----------------------------------------------------
        # CONTROL
        # ----------------------------------------------------

        control_frame = tk.Frame(left, bg="#111827")
        control_frame.pack(fill="both", expand=True)

        tk.Label(
            control_frame,
            text="MANUAL",
            bg="#111827",
            fg="white",
            font=("Arial", 14, "bold"),
        ).pack(pady=(0, 10))

        buttons = tk.Frame(control_frame, bg="#111827")
        buttons.pack(expand=True)

        button_font = ("Arial", 18, "bold")

        self.forward_button = tk.Button(
            buttons,
            text="↑",
            width=7,
            height=2,
            font=button_font,
            command=lambda: self.send_cmd(self.max_vx, 0.0, 0.0),
        )
        self.forward_button.grid(row=0, column=1, padx=5, pady=5)

        self.left_button = tk.Button(
            buttons,
            text="←",
            width=7,
            height=2,
            font=button_font,
            command=lambda: self.send_cmd(0.0, self.max_vy, 0.0),
        )
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.stop_button = tk.Button(
            buttons,
            text="STOP",
            width=7,
            height=2,
            bg="#dc2626",
            fg="white",
            activebackground="#b91c1c",
            activeforeground="white",
            font=("Arial", 13, "bold"),
            command=self.stop,
        )
        self.stop_button.grid(row=1, column=1, padx=5, pady=5)

        self.right_button = tk.Button(
            buttons,
            text="→",
            width=7,
            height=2,
            font=button_font,
            command=lambda: self.send_cmd(0.0, -self.max_vy, 0.0),
        )
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        self.backward_button = tk.Button(
            buttons,
            text="↓",
            width=7,
            height=2,
            font=button_font,
            command=lambda: self.send_cmd(-self.max_vx, 0.0, 0.0),
        )
        self.backward_button.grid(row=2, column=1, padx=5, pady=5)

        self.rotate_left_button = tk.Button(
            buttons,
            text="↺",
            width=7,
            height=2,
            font=("Arial", 16, "bold"),
            command=lambda: self.send_cmd(0.0, 0.0, self.max_wz),
        )
        self.rotate_left_button.grid(row=3, column=0, padx=5, pady=(15, 5))

        self.rotate_right_button = tk.Button(
            buttons,
            text="↻",
            width=7,
            height=2,
            font=("Arial", 16, "bold"),
            command=lambda: self.send_cmd(0.0, 0.0, -self.max_wz),
        )
        self.rotate_right_button.grid(row=3, column=2, padx=5, pady=(15, 5))

        # ====================================================
        # RIGHT PANEL
        # ====================================================

        right = tk.Frame(self, bg="#111827")
        right.grid(row=0, column=1, sticky="nsew", padx=(10, 15), pady=15)

        right.grid_rowconfigure(1, weight=1)
        right.grid_columnconfigure(0, weight=1)

        # ----------------------------------------------------
        # ROBOT STATE
        # ----------------------------------------------------

        state_frame = tk.Frame(right, bg="#1f2937", padx=15, pady=10)
        state_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        tk.Label(
            state_frame,
            text="ROBOT STATE",
            bg="#1f2937",
            fg="white",
            font=("Arial", 14, "bold"),
        ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 8))

        self.state_labels = {}

        fields = [
            ("X", "m"),
            ("Y", "m"),
            ("Yaw", "deg"),
            ("Vx", "m/s"),
            ("Vy", "m/s"),
            ("Wz", "rad/s"),
        ]

        for i, (name, unit) in enumerate(fields):
            col = i

            tk.Label(
                state_frame,
                text=name,
                bg="#1f2937",
                fg="#9ca3af",
                font=("Arial", 10, "bold"),
            ).grid(row=1, column=col, padx=8, sticky="w")

            value = tk.Label(
                state_frame,
                text=f"0.00 {unit}",
                bg="#1f2937",
                fg="white",
                font=("Arial", 11, "bold"),
            )
            value.grid(row=2, column=col, padx=8, sticky="w")

            self.state_labels[name] = value

        # ----------------------------------------------------
        # MAP
        # ----------------------------------------------------

        map_frame = tk.Frame(right, bg="#1f2937", bd=1, relief="solid")
        map_frame.grid(row=1, column=0, sticky="nsew")

        map_frame.grid_rowconfigure(0, weight=1)
        map_frame.grid_columnconfigure(0, weight=1)

        self.map_canvas = tk.Canvas(map_frame, bg="#0b0f14", highlightthickness=0)
        self.map_canvas.grid(row=0, column=0, sticky="nsew")

        self.map_canvas.bind("<Configure>", lambda event: self.draw_map())

        # ----------------------------------------------------
        # MAP STATUS
        # ----------------------------------------------------

        self.map_status = tk.Label(
            map_frame,
            text="MAP: waiting for /map",
            bg="#1f2937",
            fg="#9ca3af",
            font=("Arial", 9),
        )
        self.map_status.grid(row=1, column=0, sticky="w", padx=8, pady=5)

        self._last_map_signature = None

        # ====================================================
        # KEYBOARD
        # ====================================================

        self.bind_all("<KeyPress>", self.key_press)
        self.bind_all("<KeyRelease>", self.key_release)

    # ========================================================
    # SCALE
    # ========================================================

    def _create_scale(
        self, parent, name, variable, maximum, resolution, unit, callback
    ):
        frame = tk.Frame(parent, bg="#1f2937")
        frame.pack(fill="x", pady=3)

        tk.Label(
            frame,
            text=name,
            width=4,
            anchor="w",
            bg="#1f2937",
            fg="#d1d5db",
            font=("Arial", 10, "bold"),
        ).pack(side="left")

        scale = tk.Scale(
            frame,
            from_=0.0,
            to=maximum,
            resolution=resolution,
            variable=variable,
            orient="horizontal",
            length=180,
            showvalue=False,
            bg="#1f2937",
            fg="white",
            troughcolor="#374151",
            highlightthickness=0,
            command=callback,
        )
        scale.pack(side="left", fill="x", expand=True)

        value_label = tk.Label(
            frame,
            text=f"{variable.get():.2f} {unit}",
            width=11,
            anchor="e",
            bg="#1f2937",
            fg="#9ca3af",
            font=("Arial", 9),
        )
        value_label.pack(side="left", padx=(5, 0))

        return scale, value_label

    # ========================================================
    # VELOCITY CALLBACKS
    # ========================================================

    def vx_changed(self, value):
        self.max_vx = float(value)
        self.vx_value_label.config(text=f"{self.max_vx:.2f} m/s")

    def vy_changed(self, value):
        self.max_vy = float(value)
        self.vy_value_label.config(text=f"{self.max_vy:.2f} m/s")

    def wz_changed(self, value):
        self.max_wz = float(value)
        self.wz_value_label.config(text=f"{self.max_wz:.2f} rad/s")

    # ========================================================
    # COMMAND
    # ========================================================

    def send_cmd(self, vx, vy, wz):
        if not self.enabled:
            return

        self.controller.send_cmd(vx, vy, wz)

    def stop(self):
        self.controller.stop()

    # ========================================================
    # KEYBOARD
    # ========================================================

    def key_press(self, event):
        if not self.enabled:
            return

        key = event.keysym.lower()

        if key == "w":
            self.send_cmd(self.max_vx, 0.0, 0.0)
        elif key == "s":
            self.send_cmd(-self.max_vx, 0.0, 0.0)
        elif key == "a":
            self.send_cmd(0.0, self.max_vy, 0.0)
        elif key == "d":
            self.send_cmd(0.0, -self.max_vy, 0.0)
        elif key == "q":
            self.send_cmd(0.0, 0.0, self.max_wz)
        elif key == "e":
            self.send_cmd(0.0, 0.0, -self.max_wz)
        elif key == "space":
            self.stop()

    def key_release(self, event):
        if event.keysym.lower() in ("w", "a", "s", "d", "q", "e"):
            self.stop()

    # ========================================================
    # ENABLE
    # ========================================================

    def set_enabled(self, enabled):
        self.enabled = enabled

        state = "normal" if enabled else "disabled"

        for button in (
            self.forward_button,
            self.backward_button,
            self.left_button,
            self.right_button,
            self.stop_button,
            self.rotate_left_button,
            self.rotate_right_button,
        ):
            button.config(state=state)

    # ========================================================
    # ROBOT STATE
    # ========================================================

    def update_state(self):
        c = self.controller

        self.state_labels["X"].config(text=f"{c.robot_x:.3f} m")

        self.state_labels["Y"].config(text=f"{c.robot_y:.3f} m")

        self.state_labels["Yaw"].config(text=f"{math.degrees(c.robot_yaw):.1f}°")

        self.state_labels["Vx"].config(text=f"{c.feedback_vx:.3f} m/s")

        self.state_labels["Vy"].config(text=f"{c.feedback_vy:.3f} m/s")

        self.state_labels["Wz"].config(text=f"{c.feedback_wz:.3f} rad/s")

    # ========================================================
    # MAP
    # ========================================================

    def draw_map(self):
        self.map_canvas.delete("all")

        c = self.controller

        # RobotController must provide /map data.
        data = getattr(c, "map_data", None)

        if data is None:
            self.map_canvas.create_text(
                self.map_canvas.winfo_width() // 2,
                self.map_canvas.winfo_height() // 2,
                text="Waiting for /map ...",
                fill="#9ca3af",
                font=("Arial", 13),
            )
            return

        width = getattr(c, "map_width", 0)
        height = getattr(c, "map_height", 0)
        resolution = getattr(c, "map_resolution", 0.05)

        if width <= 0 or height <= 0:
            return

        canvas_w = max(self.map_canvas.winfo_width(), 1)
        canvas_h = max(self.map_canvas.winfo_height(), 1)

        # Fit entire map while preserving aspect ratio.
        scale = min(canvas_w / (width * resolution), canvas_h / (height * resolution))

        # Pixel size of each map cell.
        cell = max(scale * resolution, 1.0)

        map_w = width * cell
        map_h = height * cell

        offset_x = (canvas_w - map_w) / 2.0
        offset_y = (canvas_h - map_h) / 2.0

        # ----------------------------------------------------
        # OCCUPANCY GRID
        #
        # -1 = unknown
        #  0 = free
        # 100 = occupied
        # ----------------------------------------------------

        # Draw cells as rectangles.
        # Skip empty/free cells only when the map is too large.
        for my in range(height):
            row_start = my * width

            for mx in range(width):
                value = data[row_start + mx]

                if value < 0:
                    fill = "#374151"
                elif value >= 65:
                    fill = "#111111"
                else:
                    fill = "#e5e7eb"

                x1 = offset_x + mx * cell
                y1 = offset_y + (height - 1 - my) * cell
                x2 = x1 + cell
                y2 = y1 + cell

                self.map_canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="")

        # ----------------------------------------------------
        # ROBOT POSITION
        # ----------------------------------------------------

        origin_x = getattr(c, "map_origin_x", 0.0)
        origin_y = getattr(c, "map_origin_y", 0.0)

        robot_x = c.robot_x
        robot_y = c.robot_y
        robot_yaw = c.robot_yaw

        rx = offset_x + (robot_x - origin_x) / resolution * cell
        ry = offset_y + (height - (robot_y - origin_y) / resolution) * cell

        robot_radius = max(min(cell * 3.0, 14.0), 5.0)

        self.map_canvas.create_oval(
            rx - robot_radius,
            ry - robot_radius,
            rx + robot_radius,
            ry + robot_radius,
            fill="#2563eb",
            outline="white",
            width=2,
        )

        # Heading arrow.
        arrow_length = max(robot_radius * 2.5, 15.0)

        hx = rx + math.cos(robot_yaw) * arrow_length
        hy = ry - math.sin(robot_yaw) * arrow_length

        self.map_canvas.create_line(
            rx, ry, hx, hy, fill="white", width=3, arrow=tk.LAST
        )

        self.map_canvas.create_text(
            rx,
            ry + robot_radius + 10,
            text=f"({robot_x:.2f}, {robot_y:.2f})",
            fill="white",
            font=("Arial", 9, "bold"),
        )

        self.map_status.config(
            text=f"MAP: {width} x {height} | resolution: {resolution:.3f} m"
        )

    # ========================================================
    # UPDATE
    # ========================================================

    def update(self):
        self.update_state()

        c = self.controller

        signature = (
            getattr(c, "map_width", 0),
            getattr(c, "map_height", 0),
            getattr(c, "map_resolution", 0.05),
            id(getattr(c, "map_data", None)),
        )

        # Redraw when a new map arrives.
        if signature != self._last_map_signature:
            self._last_map_signature = signature
            self.draw_map()
        else:
            # Robot moves even when map itself does not change.
            self.draw_map()

    # ========================================================
    # DESTROY
    # ========================================================

    def destroy(self):
        self.unbind_all("<KeyPress>")
        self.unbind_all("<KeyRelease>")
        super().destroy()
