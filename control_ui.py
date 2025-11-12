#!/usr/bin/env python3
"""Tk-based control panel for TrailBotPro/Husky teleoperation."""

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ControlUI(Node):
    """ROS 2 node that exposes a small Tk control surface."""

    def __init__(self) -> None:
        super().__init__("control_ui")

        self.win = tk.Tk()
        self.win.title("TrailBot Control Console")
        self.win.geometry("420x420")
        self.win.protocol("WM_DELETE_WINDOW", self.on_close)

        self.cmd_vel_pub = self.create_publisher(Twist, "/husky/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/system/status", 10)

        self.lin_var = tk.DoubleVar(value=0.6)
        self.ang_var = tk.DoubleVar(value=0.8)
        self.status_var = tk.StringVar(value="Status: Ready")
        self.autonomous = False

        self.key_state: dict[str, bool] = {}
        self.drive_timer: int | None = None
        self.drive_rate_ms = 80  # ~12.5 Hz manual drive loop

        self._build_ui()
        self._bind_keys()

        self.get_logger().info("Control UI node constructed")

    # ---------- UI ----------
    def _build_ui(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(8, 4))

        root = ttk.Frame(self.win, padding=16)
        root.pack(fill="both", expand=True)

        status_frame = ttk.LabelFrame(root, text="System")
        status_frame.pack(fill="x", pady=(0, 12))

        ttk.Label(status_frame, textvariable=self.status_var, font=("Helvetica", 12, "bold")).pack(
            anchor="w"
        )

        btn_frame = ttk.LabelFrame(root, text="Core Controls")
        btn_frame.pack(fill="x", pady=(0, 12))

        controls = [
            ("Start", self.start_system),
            ("Stop", self.stop_system),
            ("Autonomous", self.toggle_autonomous),
        ]
        for idx, (label, callback) in enumerate(controls):
            ttk.Button(btn_frame, text=label, command=callback).grid(
                row=0, column=idx, padx=4, pady=4, sticky="nsew"
            )
            btn_frame.columnconfigure(idx, weight=1)

        drive_frame = ttk.LabelFrame(root, text="Directional Drive")
        drive_frame.pack(fill="x", pady=(0, 12))

        ttk.Button(drive_frame, text="▲ Forward", command=lambda: self.move("forward")).grid(
            row=0, column=1, pady=4
        )
        ttk.Button(drive_frame, text="◄ Left", command=lambda: self.turn("left")).grid(
            row=1, column=0, padx=4
        )
        ttk.Button(drive_frame, text="Stop", command=self.stop_system).grid(row=1, column=1, padx=4)
        ttk.Button(drive_frame, text="Right ►", command=lambda: self.turn("right")).grid(
            row=1, column=2, padx=4
        )
        ttk.Button(drive_frame, text="▼ Back", command=lambda: self.move("backward")).grid(
            row=2, column=1, pady=4
        )

        slider_frame = ttk.LabelFrame(root, text="Speed Presets")
        slider_frame.pack(fill="x", pady=(0, 12))

        self._add_slider(
            slider_frame,
            label="Linear (m/s)",
            var=self.lin_var,
            row=0,
            from_=0.0,
            to=1.5,
        )
        self._add_slider(
            slider_frame,
            label="Angular (rad/s)",
            var=self.ang_var,
            row=1,
            from_=0.0,
            to=2.0,
        )

        ttk.Label(
            root,
            text="Manual keys: W/A/S/D (hold) · Q/E reserved",
            foreground="#555",
        ).pack(anchor="w", pady=(4, 0))

    def _add_slider(
        self, parent: ttk.LabelFrame, *, label: str, var: tk.DoubleVar, row: int, from_: float, to: float
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        scale = ttk.Scale(parent, variable=var, from_=from_, to=to, orient="horizontal")
        scale.grid(row=row, column=1, sticky="ew", padx=6, pady=4)
        parent.columnconfigure(1, weight=1)
        value_lbl = ttk.Label(parent, text=f"{var.get():.2f}")
        value_lbl.grid(row=row, column=2, padx=(6, 0))
        var.trace_add("write", lambda *_: value_lbl.config(text=f"{var.get():.2f}"))

    # ---------- Key handling ----------
    def _bind_keys(self) -> None:
        for key in ("w", "a", "s", "d", "q", "e"):
            self.win.bind(f"<KeyPress-{key}>", self._on_key_press)
            self.win.bind(f"<KeyRelease-{key}>", self._on_key_release)

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        self.key_state[key] = True
        if self.drive_timer is None:
            self._log("Manual drive engaged (WASD)")
            self.drive_timer = self.win.after(self.drive_rate_ms, self._drive_tick)

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        self.key_state[key] = False
        if not any(self.key_state.values()):
            self._cancel_drive_loop()

    def _cancel_drive_loop(self) -> None:
        if self.drive_timer is not None:
            self.win.after_cancel(self.drive_timer)
            self.drive_timer = None
            self.publish_twist(0.0, 0.0)

    def _drive_tick(self) -> None:
        lin = 0.0
        ang = 0.0
        if self.key_state.get("w"):
            lin += 1.0
        if self.key_state.get("s"):
            lin -= 1.0
        if self.key_state.get("a"):
            ang += 1.0
        if self.key_state.get("d"):
            ang -= 1.0

        if lin or ang:
            self.publish_twist(
                lin * float(self.lin_var.get()),
                ang * float(self.ang_var.get()),
            )
            self.status_var.set("Status: Manual Drive (WASD)")

        self.drive_timer = self.win.after(self.drive_rate_ms, self._drive_tick)

    # ---------- Commands ----------
    def start_system(self) -> None:
        self.publish_status("System started")

    def stop_system(self) -> None:
        self.publish_status("System stopped")
        self.publish_twist(0.0, 0.0)
        self._cancel_drive_loop()

    def move(self, direction: str) -> None:
        lin = float(self.lin_var.get())
        if direction == "backward":
            lin = -lin
        self.publish_twist(lin, 0.0)
        self.status_var.set(f"Status: Moving {direction}")

    def turn(self, direction: str) -> None:
        ang = float(self.ang_var.get())
        if direction == "right":
            ang = -ang
        self.publish_twist(0.0, ang)
        self.status_var.set(f"Status: Turning {direction}")

    def toggle_autonomous(self) -> None:
        self.autonomous = not self.autonomous
        mode = "enabled" if self.autonomous else "disabled"
        self.publish_status(f"Autonomous mode {mode}")

    # ---------- Helpers ----------
    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.status_var.set(f"Status: {text}")
        self._log(text)

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def _set_status(self, text: str) -> None:
        self.status_var.set(f"Status: {text}")

    def _log(self, text: str) -> None:
        self.get_logger().info(text)

    def on_close(self) -> None:
        try:
            self.publish_twist(0.0, 0.0)
        except Exception:
            pass
        self._log("Closing UI window")
        self._cancel_drive_loop()
        self.win.destroy()


def main() -> None:
    rclpy.init()
    node = ControlUI()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def ros_pump() -> None:
        try:
            executor.spin_once(timeout_sec=0.0)
        except Exception as exc:
            node.get_logger().error(f"Executor error: {exc}")
        node.win.after(20, ros_pump)

    node.get_logger().info("Control UI started")
    node.win.after(20, ros_pump)
    node.win.mainloop()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
