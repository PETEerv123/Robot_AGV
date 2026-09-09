import tkinter as tk


class NavigationTab:
    def __init__(self, parent, controller, gui):

        self.parent = parent
        self.controller = controller
        self.gui = gui

        parent.configure(bg="#111827")

        # ====================================================
        # TITLE
        # ====================================================

        tk.Label(
            parent,
            text="NAVIGATION",
            bg="#111827",
            fg="white",
            font=("Arial", 22, "bold"),
        ).pack(pady=20)

        # ====================================================
        # MAIN
        # ====================================================

        main = tk.Frame(parent, bg="#111827")

        main.pack(fill="both", expand=True, padx=25)

        # ====================================================
        # MAP
        # ====================================================

        map_frame = tk.Frame(main, bg="#050505")

        map_frame.pack(side="left", fill="both", expand=True, padx=(0, 20))

        self.map_label = tk.Label(
            map_frame,
            text="NAV2 MAP\n\nMAP VIEW",
            bg="#050505",
            fg="#6b7280",
            font=("Arial", 20),
        )

        self.map_label.pack(fill="both", expand=True)

        # ====================================================
        # INFO
        # ====================================================

        info = tk.Frame(main, bg="#1f2937", width=300)

        info.pack(side="right", fill="y")

        info.pack_propagate(False)

        tk.Label(
            info,
            text="ROBOT POSE",
            bg="#1f2937",
            fg="white",
            font=("Arial", 18, "bold"),
        ).pack(pady=25)

        self.x_label = tk.Label(
            info, text="X: 0.00 m", bg="#1f2937", fg="white", font=("Arial", 15)
        )

        self.x_label.pack(pady=10)

        self.y_label = tk.Label(
            info, text="Y: 0.00 m", bg="#1f2937", fg="white", font=("Arial", 15)
        )

        self.y_label.pack(pady=10)

        self.yaw_label = tk.Label(
            info, text="Yaw: 0.00 rad", bg="#1f2937", fg="white", font=("Arial", 15)
        )

        self.yaw_label.pack(pady=10)

        # ====================================================
        # NAV STATUS
        # ====================================================

        self.status = tk.Label(
            info, text="READY", bg="#1f2937", fg="#22c55e", font=("Arial", 15, "bold")
        )

        self.status.pack(pady=30)

        # ====================================================
        # CANCEL
        # ====================================================

        self.cancel_button = tk.Button(
            info,
            text="CANCEL GOAL",
            bg="#374151",
            fg="white",
            font=("Arial", 13, "bold"),
            height=2,
            command=self.cancel_goal,
        )

        self.cancel_button.pack(fill="x", padx=25, pady=10)

    # ========================================================
    # UPDATE
    # ========================================================

    def update(self):

        c = self.controller

        self.x_label.config(text=f"X: {c.robot_x:.2f} m")

        self.y_label.config(text=f"Y: {c.robot_y:.2f} m")

        self.yaw_label.config(text=f"Yaw: {c.robot_yaw:.2f} rad")

    # ========================================================
    # CANCEL
    # ========================================================

    def cancel_goal(self):

        self.controller.stop()

        self.status.config(text="GOAL CANCELLED", fg="#f59e0b")
