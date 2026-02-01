import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class ControlMetrics(Node):
    """
    Subscribes: /temp (y), /ref (r)
    Publishes: /control_metrics (String)
    Computes online metrics using ROS time:
      e = r - y
      IAE  = ∫ |e| dt
      ISE  = ∫ e^2 dt
      ITAE = ∫ t*|e| dt (time since last setpoint change)
      MAE, RMSE
      Overshoot (%) and settling time for step-like setpoint changes
    """
    def __init__(self):
        super().__init__('control_metrics')

        # --- Parameters (can be changed later to ROS params if you want) ---
        self.settle_band_pct = 0.02     # 2% band around ref (default)
        self.step_detect_eps = 1e-3     # threshold to detect ref change
        self.min_step_mag = 0.2         # ignore tiny ref changes (units of ref)
        self.settle_hold_s = 2.0        # must stay within band for this time
        self.report_period_s = 2.0      # print/publish metrics every N seconds

        # State
        self.y = None
        self.r = None

        self.prev_time = None
        self.prev_e = None

        # Integrals since start
        self.iae = 0.0
        self.ise = 0.0
        self.itae = 0.0
        self.sum_abs_e = 0.0
        self.sum_sq_e = 0.0
        self.n = 0

        # Step-analysis state
        self.step_active = False
        self.step_start_time = None
        self.step_r0 = None
        self.step_r1 = None
        self.step_mag = None
        self.step_peak_y = None
        self.step_settled = False
        self.step_settle_enter_time = None
        self.step_settle_time = None

        # Pub/Sub
        self.sub_temp = self.create_subscription(Float32, '/temp', self.on_temp, 10)
        self.sub_ref  = self.create_subscription(Float32, '/ref',  self.on_ref,  10)

        self.pub_txt = self.create_publisher(String, '/control_metrics', 10)

        self.last_report_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz computation loop

        self.get_logger().info("control_metrics node started. Subscribing to /temp and /ref")

    def on_temp(self, msg: Float32):
        self.y = float(msg.data)

    def on_ref(self, msg: Float32):
        new_r = float(msg.data)
        if self.r is None:
            self.r = new_r
            return

        # Detect step change in reference
        if abs(new_r - self.r) > max(self.step_detect_eps, self.min_step_mag):
            self.start_step(self.r, new_r)
        self.r = new_r

    def start_step(self, r0, r1):
        now = self.get_clock().now()
        self.step_active = True
        self.step_settled = False
        self.step_settle_time = None
        self.step_settle_enter_time = None

        self.step_start_time = now
        self.step_r0 = r0
        self.step_r1 = r1
        self.step_mag = (r1 - r0)
        self.step_peak_y = None

        # Reset ITAE accumulator relative to step (we accumulate globally using elapsed since last step)
        self.get_logger().info(f"Detected ref step: {r0:.3f} -> {r1:.3f} (Δ={self.step_mag:.3f})")

    def tick(self):
        # Need both signals
        if self.y is None or self.r is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        if self.prev_time is None:
            self.prev_time = now
            self.prev_e = (self.r - self.y)
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        e = (self.r - self.y)

        # Trapezoidal integration for IAE/ISE
        e_prev = self.prev_e if self.prev_e is not None else e
        abs_e = abs(e)
        abs_e_prev = abs(e_prev)

        self.iae += 0.5 * (abs_e + abs_e_prev) * dt
        self.ise += 0.5 * (e*e + e_prev*e_prev) * dt

        # ITAE: use time since last step (or since start if no step)
        if self.step_active and self.step_start_time is not None:
            tau = (now - self.step_start_time).nanoseconds * 1e-9
        else:
            tau = (now.nanoseconds * 1e-9)  # absolute time; not ideal but ok if no steps
        self.itae += 0.5 * ((tau*abs_e) + (tau*abs_e_prev)) * dt

        # MAE/RMSE accumulators
        self.sum_abs_e += abs_e
        self.sum_sq_e += e * e
        self.n += 1

        # Step response metrics (overshoot and settling)
        self.update_step_metrics(now)

        # Periodic report
        if (now - self.last_report_time).nanoseconds * 1e-9 >= self.report_period_s:
            self.last_report_time = now
            self.publish_report()

        self.prev_time = now
        self.prev_e = e

    def update_step_metrics(self, now):
        if not self.step_active or self.step_start_time is None:
            return
        if self.y is None:
            return

        # Track peak y after step
        if self.step_peak_y is None:
            self.step_peak_y = self.y
        else:
            # for both + and - steps, peak is the extreme in direction of step
            if self.step_mag >= 0:
                self.step_peak_y = max(self.step_peak_y, self.y)
            else:
                self.step_peak_y = min(self.step_peak_y, self.y)

        # Settling band: ±(pct*|step|) around final ref OR ±pct*|ref| if step small
        mag = abs(self.step_mag) if self.step_mag is not None else 0.0
        band = self.settle_band_pct * max(abs(self.step_r1), mag, 1e-6)

        within = abs(self.y - self.step_r1) <= band

        if within:
            if self.step_settle_enter_time is None:
                self.step_settle_enter_time = now
            else:
                held = (now - self.step_settle_enter_time).nanoseconds * 1e-9
                if (not self.step_settled) and held >= self.settle_hold_s:
                    self.step_settled = True
                    self.step_settle_time = (now - self.step_start_time).nanoseconds * 1e-9
        else:
            self.step_settle_enter_time = None

    def publish_report(self):
        mae = self.sum_abs_e / self.n if self.n > 0 else float('nan')
        rmse = math.sqrt(self.sum_sq_e / self.n) if self.n > 0 else float('nan')

        # Overshoot for last detected step
        overshoot_pct = None
        settle_time = None
        if self.step_active and self.step_mag is not None and abs(self.step_mag) > 1e-9 and self.step_peak_y is not None:
            if self.step_mag >= 0:
                overshoot = (self.step_peak_y - self.step_r1)
            else:
                overshoot = (self.step_r1 - self.step_peak_y)
            overshoot_pct = 100.0 * max(0.0, overshoot) / abs(self.step_mag)
            settle_time = self.step_settle_time

        txt = (
            f"IAE={self.iae:.3f}  ISE={self.ise:.3f}  ITAE={self.itae:.3f}  "
            f"MAE={mae:.3f}  RMSE={rmse:.3f}"
        )
        if overshoot_pct is not None:
            txt += f"  OS={overshoot_pct:.1f}%"
        if settle_time is not None:
            txt += f"  Ts={settle_time:.2f}s"

        msg = String()
        msg.data = txt
        self.pub_txt.publish(msg)
        self.get_logger().info(txt)

def main():
    rclpy.init()
    node = ControlMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
