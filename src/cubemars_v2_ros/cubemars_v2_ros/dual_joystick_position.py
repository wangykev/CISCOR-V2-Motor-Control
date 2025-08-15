#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy, JointState

def clamp(x, lo, hi): 
    return lo if x < lo else hi if x > hi else x

def wrap_pi(a):
    while a <= -math.pi: a += 2.0 * math.pi
    while a >   math.pi: a -= 2.0 * math.pi
    return a

class DualJoystickPosition(Node):
    """
    Turntable mapping: integrate stick Δθ (wrapped) into joint target.
    Per-motor gain, direction, and optional stiction torque feedforward.
    """
    def __init__(self):
        super().__init__('dual_joystick_position')

        # --- Params ---
        self.declare_parameter('motor1_ns', 'ak70')
        self.declare_parameter('motor2_ns', 'ak80')
        self.declare_parameter('control_hz', 100.0)

        # Per-motor MIT gains (AK80 often needs softer Kp, higher D)
        self.declare_parameter('kp1', 12.0)
        self.declare_parameter('kd1', 0.8)
        self.declare_parameter('kp2', 8.0)
        self.declare_parameter('kd2', 1.2)

        # Motor radians per full 2π stick turn (AK80 needs more due to higher reduction)
        self.declare_parameter('gain_per_rev1', 2.0)   # AK70-10 default
        self.declare_parameter('gain_per_rev2', 6.0)   # AK80-64 default (bigger)

        # Direction (+1 or -1): make clockwise stick match clockwise joint
        self.declare_parameter('dir1', +1.0)
        self.declare_parameter('dir2', -1.0)

        # Optional stiction torque (Nm). Applied only when stick moves.
        self.declare_parameter('tau_ff1', 0.00)
        self.declare_parameter('tau_ff2', 0.30)   # give AK80 a little push

        # Scale torque feedforward with stick radius (0..1)
        self.declare_parameter('scale_ff_with_radius', True)

        # Deadzone on stick magnitude
        self.declare_parameter('deadzone', 0.15)

        # Clamp commanded position to physical range
        self.declare_parameter('clamp_p_min', -12.5)
        self.declare_parameter('clamp_p_max',  12.5)

        # Axis mapping (Xbox-like defaults)
        self.declare_parameter('axis_left_x', 0)
        self.declare_parameter('axis_left_y', 1)
        self.declare_parameter('axis_right_x', 3)
        self.declare_parameter('axis_right_y', 4)

        # Inversions so up is +Y (most pads report up=-1)
        self.declare_parameter('invert_left_x',  1.0)
        self.declare_parameter('invert_left_y', -1.0)
        self.declare_parameter('invert_right_x',  1.0)
        self.declare_parameter('invert_right_y', -1.0)

        # Buttons
        self.declare_parameter('btn_start', 7)   # Start/Menu
        self.declare_parameter('btn_exit',  6)   # Back/Select
        self.declare_parameter('btn_zero',  2)   # X / Square

        # Auto-start motors once
        self.declare_parameter('auto_start', True)

        # --- Read params ---
        self.ns1 = self.get_parameter('motor1_ns').value
        self.ns2 = self.get_parameter('motor2_ns').value
        self.dt = 1.0 / float(self.get_parameter('control_hz').value)

        self.kp1 = float(self.get_parameter('kp1').value)
        self.kd1 = float(self.get_parameter('kd1').value)
        self.kp2 = float(self.get_parameter('kp2').value)
        self.kd2 = float(self.get_parameter('kd2').value)

        self.gpr1 = float(self.get_parameter('gain_per_rev1').value)
        self.gpr2 = float(self.get_parameter('gain_per_rev2').value)
        self.gprad1 = self.gpr1 / (2.0 * math.pi)  # motor rad per stick rad
        self.gprad2 = self.gpr2 / (2.0 * math.pi)

        self.dir1 = float(self.get_parameter('dir1').value)
        self.dir2 = float(self.get_parameter('dir2').value)

        self.tau_ff1 = float(self.get_parameter('tau_ff1').value)
        self.tau_ff2 = float(self.get_parameter('tau_ff2').value)
        self.scale_ff = bool(self.get_parameter('scale_ff_with_radius').value)

        self.deadzone = float(self.get_parameter('deadzone').value)
        self.pmin = float(self.get_parameter('clamp_p_min').value)
        self.pmax = float(self.get_parameter('clamp_p_max').value)

        self.LX = int(self.get_parameter('axis_left_x').value)
        self.LY = int(self.get_parameter('axis_left_y').value)
        self.RX = int(self.get_parameter('axis_right_x').value)
        self.RY = int(self.get_parameter('axis_right_y').value)

        self.iLX = float(self.get_parameter('invert_left_x').value)
        self.iLY = float(self.get_parameter('invert_left_y').value)
        self.iRX = float(self.get_parameter('invert_right_x').value)
        self.iRY = float(self.get_parameter('invert_right_y').value)

        self.btn_start = int(self.get_parameter('btn_start').value)
        self.btn_exit  = int(self.get_parameter('btn_exit').value)
        self.btn_zero  = int(self.get_parameter('btn_zero').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        # --- Pubs/Subs ---
        self.pub1_cmd = self.create_publisher(Float64MultiArray, f'/{self.ns1}/mit_cmd', 10)
        self.pub2_cmd = self.create_publisher(Float64MultiArray, f'/{self.ns2}/mit_cmd', 10)
        self.pub1_sp  = self.create_publisher(String,             f'/{self.ns1}/special', 10)
        self.pub2_sp  = self.create_publisher(String,             f'/{self.ns2}/special', 10)

        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(JointState, f'/{self.ns1}/joint_state', self.on_js1, 10)
        self.create_subscription(JointState, f'/{self.ns2}/joint_state', self.on_js2, 10)

        # --- State ---
        self.axes = []; self.buttons = []; self.last_buttons = []
        self.started = False

        # Current (for init only) + targets
        self.have1 = False; self.cur_p1 = 0.0; self.tgt_p1 = None
        self.have2 = False; self.cur_p2 = 0.0; self.tgt_p2 = None

        # Stick angle trackers to compute continuous delta
        self.active1 = False; self.last_th1 = 0.0
        self.active2 = False; self.last_th2 = 0.0

        # Timer loop
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info("Turntable joystick control running (per-motor gains & stiction FF)")

    # ---- Callbacks ----
    def on_joy(self, msg: Joy):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        if not self.last_buttons:
            self.last_buttons = [0]*len(self.buttons)

        def rising(i):
            try: return self.buttons[i]==1 and self.last_buttons[i]==0
            except IndexError: return False

        if rising(self.btn_start): self._special('start')
        if rising(self.btn_exit):  self._special('exit')
        if rising(self.btn_zero):
            self._special('zero')
            # Optionally reset targets after zero:
            # self.tgt_p1 = 0.0; self.tgt_p2 = 0.0

        self.last_buttons = self.buttons.copy()

    def on_js1(self, msg: JointState):
        if msg.position:
            self.cur_p1 = float(msg.position[0])
            if not self.have1:
                self.have1 = True
                if self.tgt_p1 is None: self.tgt_p1 = self.cur_p1

    def on_js2(self, msg: JointState):
        if msg.position:
            self.cur_p2 = float(msg.position[0])
            if not self.have2:
                self.have2 = True
                if self.tgt_p2 is None: self.tgt_p2 = self.cur_p2

    # ---- Control tick ----
    def tick(self):
        if self.auto_start and not self.started:
            self._special('start'); self.started = True

        # Motor 1 from left stick
        self.tgt_p1, self.active1, self.last_th1, sgn1, r1 = self._update_from_stick(
            self.tgt_p1, self.active1, self.last_th1,
            ax_x=self.LX, ax_y=self.LY, inv_x=self.iLX, inv_y=self.iLY,
            gain_per_rad=self.gprad1, direction=self.dir1, pmin=self.pmin, pmax=self.pmax
        )
        t1 = (self.tau_ff1 * sgn1 * (r1 if self.scale_ff else 1.0)) if self.active1 else 0.0
        if self.tgt_p1 is not None:
            self._send_cmd(self.pub1_cmd, self.tgt_p1, self.kp1, self.kd1, t1)

        # Motor 2 from right stick
        self.tgt_p2, self.active2, self.last_th2, sgn2, r2 = self._update_from_stick(
            self.tgt_p2, self.active2, self.last_th2,
            ax_x=self.RX, ax_y=self.RY, inv_x=self.iRX, inv_y=self.iRY,
            gain_per_rad=self.gprad2, direction=self.dir2, pmin=self.pmin, pmax=self.pmax
        )
        t2 = (self.tau_ff2 * sgn2 * (r2 if self.scale_ff else 1.0)) if self.active2 else 0.0
        if self.tgt_p2 is not None:
            self._send_cmd(self.pub2_cmd, self.tgt_p2, self.kp2, self.kd2, t2)

    # ---- Helpers ----
    def _update_from_stick(self, tgt_p, active, last_th, ax_x, ax_y, inv_x, inv_y,
                           gain_per_rad, direction, pmin, pmax):
        # Read stick; compute angle and magnitude
        try:
            x = float(self.axes[ax_x]) * float(inv_x)
            y = float(self.axes[ax_y]) * float(inv_y)
        except (IndexError, TypeError):
            return tgt_p, False, last_th, 0.0, 0.0

        r = (x*x + y*y) ** 0.5
        if r < self.deadzone:
            return tgt_p, False, last_th, 0.0, r

        th = math.atan2(y, x)  # (-pi, pi]
        if not active:
            # First frame after entering active region: latch angle
            return tgt_p, True, th, 0.0, r

        # Continuous delta with wrap handling
        dth = wrap_pi(th - last_th)  # (-pi, pi]
        dp = direction * gain_per_rad * dth  # motor radians to add
        if tgt_p is None:
            tgt_p = 0.0
        tgt_p = clamp(tgt_p + dp, pmin, pmax)

        # Sign for torque FF in commanded direction
        sgn = 1.0 if dp > 0.0 else (-1.0 if dp < 0.0 else 0.0)
        return tgt_p, True, th, sgn, r

    def _send_cmd(self, pub, p, kp, kd, t_ff):
        msg = Float64MultiArray()
        msg.data = [float(p), 0.0, float(kp), float(kd), float(t_ff)]
        pub.publish(msg)

    def _special(self, s):
        ss = String(); ss.data = s
        self.pub1_sp.publish(ss); self.pub2_sp.publish(ss)
        if s == 'start': self.started = True
        if s == 'exit':  self.started = False

def main(args=None):
    rclpy.init(args=args)
    node = DualJoystickPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
