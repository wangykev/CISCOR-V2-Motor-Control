#!/usr/bin/env python3
import threading, can, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Int32, UInt8
from sensor_msgs.msg import JointState

# Motor limits (from your working scripts)
LIMITS = {
    "AK70-10": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-50.0, V_MAX=50.0,
                    T_MIN=-25.0, T_MAX=25.0, KP_MIN=0.0, KP_MAX=500.0, KD_MIN=0.0, KD_MAX=5.0),
    "AK80-64": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-8.00, V_MAX=8.00,
                    T_MIN=-144.0, T_MAX=144.0, KP_MIN=0.0, KP_MAX=500.0, KD_MIN=0.0, KD_MAX=5.0),
}

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x
def f2u(x, lo, hi, bits):
    x = clamp(x, lo, hi); return int((x - lo) * (((1 << bits) - 1) / (hi - lo)))
def u2f(u, lo, hi, bits):
    u = max(0, min(u, (1 << bits) - 1)); return lo + (u / ((1 << bits) - 1)) * (hi - lo)

def pack_mit(p, v, kp, kd, t, R):
    p_i  = f2u(p,  R["P_MIN"],  R["P_MAX"],  16)
    v_i  = f2u(v,  R["V_MIN"],  R["V_MAX"],  12)
    kp_i = f2u(kp, R["KP_MIN"], R["KP_MAX"], 12)
    kd_i = f2u(kd, R["KD_MIN"], R["KD_MAX"], 12)
    t_i  = f2u(t,  R["T_MIN"],  R["T_MAX"],  12)
    return bytes([
        (p_i >> 8) & 0xFF, p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
    ])

def parse_reply(b, R):
    if len(b) != 8: return None
    drv   = b[0]
    p_int = (b[1] << 8) | b[2]
    v_int = (b[3] << 4) | (b[4] >> 4)
    i_int = ((b[4] & 0x0F) << 8) | b[5]
    temp  = b[6]
    err   = b[7]
    p = u2f(p_int, R["P_MIN"], R["P_MAX"], 16)
    v = u2f(v_int, R["V_MIN"], R["V_MAX"], 12)
    tau = u2f(i_int, R["T_MIN"], R["T_MAX"], 12)
    return drv, p, v, tau, temp, err

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        # ---- params ----
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id', 1)
        self.declare_parameter('motor_type', 'AK70-10')  # 'AK70-10' or 'AK80-64'
        self.declare_parameter('control_hz', 20.0)  
        self.declare_parameter('poll_state', False)      # keep false; no periodic specials
        self.declare_parameter('joint_name', 'joint')
        self.declare_parameter('auto_start', False)      # default off — avoid accidental START spam

        self.iface = self.get_parameter('can_interface').value
        self.can_id = int(self.get_parameter('can_id').value)
        self.motor_type = self.get_parameter('motor_type').value
        self.R = LIMITS[self.motor_type]
        self.joint_name = self.get_parameter('joint_name').value
        self.control_dt = 1.0 / float(self.get_parameter('control_hz').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        # ---- CAN ----
        self.arb = self.can_id & 0x7FF  # standard frame
        self.bus = can.interface.Bus(bustype="socketcan", channel=self.iface)
        try: self.bus.set_filters([{"can_id": self.arb, "can_mask": 0x7FF}])
        except Exception: pass

        # ---- ROS I/O ----
        self.pub_js = self.create_publisher(JointState, 'joint_state', 10)
        self.pub_temp = self.create_publisher(Int32, 'temperature', 10)
        self.pub_err = self.create_publisher(UInt8, 'error_code', 10)
        self.pub_line = self.create_publisher(String, 'state_line', 10)
        self.create_subscription(Float64MultiArray, 'mit_cmd', self.on_cmd, 10)
        self.create_subscription(String, 'special', self.on_special, 10)

        # ---- command / state ----
        self._lock = threading.Lock()
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]    # p, v, kp, kd, t
        self._started = False                   # track START state ourselves

        # Absolute (unwrapped) position state
        self._last_p = None
        self._p_abs = 0.0
        self._span = self.R["P_MAX"] - self.R["P_MIN"]

        # ---- timers ----
        self.create_timer(self.control_dt, self._tick_control)
        # NOTE: no state polling timer; we rely on replies to MIT frames or explicit specials

        # ---- RX ----
        self._stop = False
        self._rx = threading.Thread(target=self._rx_loop, daemon=True); self._rx.start()

        # Optional one-time auto start (single frame)
        if self.auto_start and not self._started:
            self._send_special(0xFC)
            self._started = True

    # ---- callbacks ----
    def on_cmd(self, msg):
        if len(msg.data) != 5:
            self.get_logger().warn("mit_cmd expects [p, v, kp, kd, t]")
            return
        with self._lock:
            self.cmd = list(map(float, msg.data))
        # Lazy-start exactly once on first command
        if not self._started:
            self._send_special(0xFC)
            self._started = True

    def on_special(self, msg):
        m = msg.data.strip().lower()
        if   m == "start":
            if not self._started:
                self._send_special(0xFC)
                self._started = True
        elif m == "exit":
            if self._started:
                self._send_special(0xFD)
                self._started = False
        elif m == "zero":
            self._send_mit_once(0,0,0,0,0)
            self._send_special(0xFE)
        else:
            self.get_logger().warn("special: start|exit|zero")

    # ---- timers ----
    def _tick_control(self):
        with self._lock:
            p, v, kp, kd, t = self.cmd
        data = pack_mit(p, v, kp, kd, t, self.R)
        try:
            self.bus.send(can.Message(arbitration_id=self.arb, data=data, is_extended_id=False))
        except can.CanError:
            pass

    # ---- helpers ----
    def _send_special(self, code):
        d = b"\xFF"*7 + bytes([code & 0xFF])
        try: self.bus.send(can.Message(arbitration_id=self.arb, data=d, is_extended_id=False))
        except can.CanError: pass

    def _send_mit_once(self, p, v, kp, kd, t):
        d = pack_mit(p, v, kp, kd, t, self.R)
        try: self.bus.send(can.Message(arbitration_id=self.arb, data=d, is_extended_id=False))
        except can.CanError: pass

    def _rx_loop(self):
        while not self._stop:
            rx = self.bus.recv(timeout=0.1)
            if not rx or rx.arbitration_id != self.arb or len(rx.data) != 8:
                continue
            s = parse_reply(rx.data, self.R)
            if not s:
                continue
            drv, p, v, tau, temp, err = s
            if drv != (self.arb & 0xFF):
                continue

            # Unwrap absolute position
            if self._last_p is None:
                self._p_abs = p
            else:
                dp = p - self._last_p
                if dp >  0.5 * self._span: dp -= self._span
                if dp < -0.5 * self._span: dp += self._span
                self._p_abs += dp
            self._last_p = p

            # JointState + line
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [self.joint_name]
            js.position, js.velocity, js.effort = [p], [v], [tau]
            self.pub_js.publish(js)
            self.pub_temp.publish(Int32(data=int(temp)))
            self.pub_err.publish(UInt8(data=int(err)))

            # Publish single-line summary (fixed-width columns, sign-aware, 2 decimals)
            line = String()
            line.data = (
                f"can_id=0x{self.arb:03X}  "
                f"p={p:+8.2f} rad  "
                f"p_abs={self._p_abs:+10.2f} rad  "
                f"v={v:+7.2f} rad/s  "
                f"tau={tau:+7.2f} Nm  "
                f"temp={int(temp):3d}C  "
                f"err={int(err):+4d}"
            )
            self.pub_line.publish(line)


    def destroy_node(self):
        self._stop = True
        try: self._rx.join(timeout=0.3)
        except: pass
        try: self.bus.shutdown()
        except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()
