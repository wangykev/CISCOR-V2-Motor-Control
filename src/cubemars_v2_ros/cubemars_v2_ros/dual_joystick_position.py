#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy


def wrap_to_pi(x: float) -> float:
    while x <= -math.pi:
        x += 2.0 * math.pi
    while x > math.pi:
        x -= 2.0 * math.pi
    return x


class DualJoystickPosition(Node):
    """
    Dual-joystick -> two continuous position commands (turntable style).
    - Left stick controls motor1, right stick controls motor2 (indices configurable)
    - Unit circle angle from stick (right=0, up=+pi/2)
    - Accumulate angle deltas across wrap (infinite turns)
    - Hysteresis deadzone to avoid jitter around center
    - Optional low-pass on commanded position for smooth motion
    Publishes to:
      /<motor1_ns>/mit_cmd and /<motor1_ns>/special
      /<motor2_ns>/mit_cmd and /<motor2_ns>/special
    """

    def __init__(self):
        super().__init__("dual_joystick_position")

        # ---- General control rate ----
        self.declare_parameter("control_hz", 200.0)
        self.hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / self.hz

        # ---- Motor namespaces ----
        self.declare_parameter("motor1_ns", "ak70")
        self.declare_parameter("motor2_ns", "ak80")
        self.ns1 = str(self.get_parameter("motor1_ns").value)
        self.ns2 = str(self.get_parameter("motor2_ns").value)

        # ---- Auto-start (send "start" once) ----
        self.declare_parameter("auto_start", True)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        # ---- Gains (per motor) ----
        self.declare_parameter("kp1", 400.0)
        self.declare_parameter("kd1", 2.5)
        self.declare_parameter("kp2", 500.0)
        self.declare_parameter("kd2", 3.0)
        self.kp1 = float(self.get_parameter("kp1").value)
        self.kd1 = float(self.get_parameter("kd1").value)
        self.kp2 = float(self.get_parameter("kp2").value)
        self.kd2 = float(self.get_parameter("kd2").value)

        # ---- Stick -> position scaling (per revolution) ----
        # Full 2π stick rotation equals gain_per_rev radians on the motor.
        # For "one full stick circle = one motor revolution", set 6.283.
        self.declare_parameter("gain_per_rev1", 6.283)
        self.declare_parameter("gain_per_rev2", 6.283)
        self.gain1 = float(self.get_parameter("gain_per_rev1").value)
        self.gain2 = float(self.get_parameter("gain_per_rev2").value)

        # ---- Axes mapping (left/right stick) ----
        # Logitech Dual Action typically: left=(0,1), right=(3,4) or (2,3). Confirm via /joy.
        self.declare_parameter("axis1_x", 0)
        self.declare_parameter("axis1_y", 1)
        self.declare_parameter("axis2_x", 3)
        self.declare_parameter("axis2_y", 4)
        self.ax1x = int(self.get_parameter("axis1_x").value)
        self.ax1y = int(self.get_parameter("axis1_y").value)
        self.ax2x = int(self.get_parameter("axis2_x").value)
        self.ax2y = int(self.get_parameter("axis2_y").value)

        # ---- Optional inversions and direction flips ----
        self.declare_parameter("invert1_x", 1.0)
        self.declare_parameter("invert1_y", -1.0)
        self.declare_parameter("invert2_x", 1.0)
        self.declare_parameter("invert2_y", -1.0)
        self.inv1x = float(self.get_parameter("invert1_x").value)
        self.inv1y = float(self.get_parameter("invert1_y").value)
        self.inv2x = float(self.get_parameter("invert2_x").value)
        self.inv2y = float(self.get_parameter("invert2_y").value)

        # Direction multipliers: +1.0 (same as stick), -1.0 (invert)
        self.declare_parameter("direction1", -1.0)
        self.declare_parameter("direction2", -1.0)
        self.dir1 = float(self.get_parameter("direction1").value)
        self.dir2 = float(self.get_parameter("direction2").value)

        # ---- Deadzone / hysteresis ----
        self.declare_parameter("deadzone_enter", 0.15)
        self.declare_parameter("deadzone_exit", 0.10)
        self.dz_enter = float(self.get_parameter("deadzone_enter").value)
        self.dz_exit = float(self.get_parameter("deadzone_exit").value)

        # ---- Low-pass filter cutoff (Hz). 0 => no smoothing (alpha=1) ----
        self.declare_parameter("pos_lpf_hz", 10.0)
        self.alpha_pos = self._alpha_from_fc(float(self.get_parameter("pos_lpf_hz").value), self.dt)

        # ---- Optional position clamps (disabled by default for infinite turns) ----
        self.declare_parameter("enable_pos_clamp1", False)
        self.declare_parameter("enable_pos_clamp2", False)
        self.declare_parameter("p1_min", -50.0)
        self.declare_parameter("p1_max", 50.0)
        self.declare_parameter("p2_min", -50.0)
        self.declare_parameter("p2_max", 50.0)
        self.clamp1 = bool(self.get_parameter("enable_pos_clamp1").value)
        self.clamp2 = bool(self.get_parameter("enable_pos_clamp2").value)
        self.p1_min = float(self.get_parameter("p1_min").value)
        self.p1_max = float(self.get_parameter("p1_max").value)
        self.p2_min = float(self.get_parameter("p2_min").value)
        self.p2_max = float(self.get_parameter("p2_max").value)

        # ---- Publishers / Subscribers ----
        self.pub1_cmd = self.create_publisher(Float64MultiArray, f"/{self.ns1}/mit_cmd", 10)
        self.pub1_sp = self.create_publisher(String, f"/{self.ns1}/special", 10)
        self.pub2_cmd = self.create_publisher(Float64MultiArray, f"/{self.ns2}/mit_cmd", 10)
        self.pub2_sp = self.create_publisher(String, f"/{self.ns2}/special", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)

        # ---- State (per motor) ----
        self.axes = []
        # Motor 1
        self.m1_active = False
        self.m1_last_th = 0.0
        self.m1_turns = 0.0         # accumulated stick angle (radians)
        self.m1_pos_cmd = 0.0       # filtered output (radians)
        # Motor 2
        self.m2_active = False
        self.m2_last_th = 0.0
        self.m2_turns = 0.0
        self.m2_pos_cmd = 0.0

        # Timer loop
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            "dual_joystick_position: left->%s, right->%s | axes1=(%d,%d) axes2=(%d,%d)"
            % (self.ns1, self.ns2, self.ax1x, self.ax1y, self.ax2x, self.ax2y)
        )

        # Auto-start
        if self.auto_start:
            self._special(self.pub1_sp, "start")
            self._special(self.pub2_sp, "start")

    # --------------- Helpers ---------------
    @staticmethod
    def _alpha_from_fc(fc: float, dt: float) -> float:
        if fc <= 0.0:
            return 1.0
        tau = 1.0 / (2.0 * math.pi * fc)
        return dt / (dt + tau)

    @staticmethod
    def _stick_polar(x: float, y: float, invert_x: float, invert_y: float):
        # Normalize if int16-style
        if abs(x) > 1.5 or abs(y) > 1.5:
            x /= 32767.0
            y /= 32767.0
        x *= invert_x
        y *= invert_y
        r = math.hypot(x, y)
        th = math.atan2(y, x)  # (-pi, pi]
        return th, r

    @staticmethod
    def _publish_cmd(pub, pos: float, kp: float, kd: float):
        msg = Float64MultiArray()
        msg.data = [float(pos), 0.0, float(kp), float(kd), 0.0]
        pub.publish(msg)

    @staticmethod
    def _special(pub, s: str):
        m = String()
        m.data = s
        pub.publish(m)

    # --------------- ROS callbacks ---------------
    def on_joy(self, msg: Joy):
        self.axes = list(msg.axes) if msg.axes is not None else []

    def tick(self):
        # Read axes safely
        def get_axis(i, default=0.0):
            try:
                return float(self.axes[i])
            except Exception:
                return default

        # ---- Motor 1: left stick ----
        x1 = get_axis(self.ax1x)
        y1 = get_axis(self.ax1y)
        th1, r1 = self._stick_polar(x1, y1, self.inv1x, self.inv1y)

        if not self.m1_active and r1 >= self.dz_enter:
            self.m1_active = True
            self.m1_last_th = th1
        elif self.m1_active and r1 < self.dz_exit:
            self.m1_active = False

        if self.m1_active:
            dth1 = wrap_to_pi(th1 - self.m1_last_th)
            self.m1_last_th = th1
            self.m1_turns += self.dir1 * dth1
        # target is accumulated stick angle, scaled to motor radians
        m1_target = (self.m1_turns / (2.0 * math.pi)) * self.gain1
        if self.clamp1:
            m1_target = max(self.p1_min, min(self.p1_max, m1_target))
        self.m1_pos_cmd += self.alpha_pos * (m1_target - self.m1_pos_cmd)
        self._publish_cmd(self.pub1_cmd, self.m1_pos_cmd, self.kp1, self.kd1)

        # ---- Motor 2: right stick ----
        x2 = get_axis(self.ax2x)
        y2 = get_axis(self.ax2y)
        th2, r2 = self._stick_polar(x2, y2, self.inv2x, self.inv2y)

        if not self.m2_active and r2 >= self.dz_enter:
            self.m2_active = True
            self.m2_last_th = th2
        elif self.m2_active and r2 < self.dz_exit:
            self.m2_active = False

        if self.m2_active:
            dth2 = wrap_to_pi(th2 - self.m2_last_th)
            self.m2_last_th = th2
            self.m2_turns += self.dir2 * dth2
        m2_target = (self.m2_turns / (2.0 * math.pi)) * self.gain2
        if self.clamp2:
            m2_target = max(self.p2_min, min(self.p2_max, m2_target))
        self.m2_pos_cmd += self.alpha_pos * (m2_target - self.m2_pos_cmd)
        self._publish_cmd(self.pub2_cmd, self.m2_pos_cmd, self.kp2, self.kd2)

    # --------------- Main ---------------
def main(args=None):
    rclpy.init(args=args)
    node = DualJoystickPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
