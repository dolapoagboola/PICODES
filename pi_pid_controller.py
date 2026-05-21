"""
=============================================================
 Roomba Gas Detection Robot - Raspberry Pi 5 PID Controller
=============================================================
 Controls:
   - Straight-line driving (speed PID)
   - Heading correction (heading PID)
 Feedback: LiDAR sensor (via rplidar or similar)
 Output:   Serial commands to Arduino Mega

 Serial protocol to Arduino:
   "L<left_pwm>,R<right_pwm>\n"
   e.g. "L180,R200\n"

 Dependencies:
   pip install pyserial numpy matplotlib rplidar-roboticia
=============================================================
"""

import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ─── CONFIGURATION ───────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyACM0"   # Arduino Mega port on Pi
BAUD_RATE     = 115200
LOOP_HZ       = 20               # PID loop rate (Hz)
LOOP_DT       = 1.0 / LOOP_HZ

BASE_SPEED    = 150              # Base PWM (0–255)
MAX_PWM       = 230              # Safety cap
MIN_PWM       = 60               # Min to overcome stiction

# ─── PID GAINS (tune these!) ─────────────────────────────────
# Straight-line speed PID
KP_SPEED = 1.2
KI_SPEED = 0.05
KD_SPEED = 0.3

# Heading PID
KP_HEAD  = 2.5
KI_HEAD  = 0.02
KD_HEAD  = 0.8

# ─── PID CLASS ───────────────────────────────────────────────
class PIDController:
    def __init__(self, kp, ki, kd, output_min=-255, output_max=255,
                 integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def compute(self, setpoint, measured, dt=None):
        """
        Returns PID output.
        setpoint : desired value
        measured : current sensor reading
        dt       : time delta in seconds (uses internal clock if None)
        """
        now = time.time()
        if dt is None:
            dt = (now - self._prev_time) if self._prev_time else LOOP_DT
        self._prev_time = now

        error      = setpoint - measured
        self._integral += error * dt
        # Anti-windup clamp
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)

        return max(self.output_min, min(self.output_max, output))


# ─── LIDAR HEADING ESTIMATOR ─────────────────────────────────
class LidarHeadingEstimator:
    """
    Wraps rplidar to extract current heading relative to start.
    Uses scan matching (centroid-based) for simplicity.
    Replace with full SLAM if needed.
    """
    def __init__(self):
        try:
            from rplidar import RPLidar
            self.lidar = RPLidar('/dev/ttyUSB0')
            self.lidar.connect()
            self._available = True
            print("[LiDAR] Connected successfully.")
        except Exception as e:
            print(f"[LiDAR] Not available ({e}). Using simulated heading.")
            self._available = False

        self._heading = 0.0   # degrees, 0 = forward

    def get_heading(self):
        """Returns current heading in degrees (–180 to +180)."""
        if not self._available:
            return self._simulated_heading()
        try:
            scan = next(self.lidar.iter_scans())
            angles = [m[1] for m in scan if m[0] > 0]  # quality > 0
            if angles:
                # Use mean angle shift as proxy for rotation
                mean_angle = np.mean(angles)
                self._heading = mean_angle - 180.0
        except Exception:
            pass
        return self._heading

    def _simulated_heading(self):
        """Simulate small heading drift for testing without hardware."""
        self._heading += np.random.normal(0, 0.5)
        self._heading = max(-30, min(30, self._heading))
        return self._heading

    def close(self):
        if self._available:
            self.lidar.stop()
            self.lidar.disconnect()


# ─── MOTOR COMMANDER ─────────────────────────────────────────
class MotorCommander:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)   # wait for Arduino reset
            print(f"[Serial] Connected to Arduino on {port}")
            self._available = True
        except serial.SerialException as e:
            print(f"[Serial] Arduino not found ({e}). Running in simulation mode.")
            self._available = False

    def send(self, left_pwm, right_pwm):
        """
        Clamp and send PWM values to Arduino.
        Protocol: "L<int>,R<int>\n"
        """
        l = int(max(MIN_PWM, min(MAX_PWM, left_pwm)))
        r = int(max(MIN_PWM, min(MAX_PWM, right_pwm)))
        cmd = f"L{l},R{r}\n"
        if self._available:
            self.ser.write(cmd.encode())
        else:
            print(f"  [SIM] → {cmd.strip()}")

    def stop(self):
        if self._available:
            self.ser.write(b"L0,R0\n")
        self._available and self.ser.close()


# ─── REAL-TIME PLOT ──────────────────────────────────────────
class LivePlotter:
    WINDOW = 200   # samples to show

    def __init__(self):
        self.times     = deque(maxlen=self.WINDOW)
        self.headings  = deque(maxlen=self.WINDOW)
        self.left_pwm  = deque(maxlen=self.WINDOW)
        self.right_pwm = deque(maxlen=self.WINDOW)
        self.pid_out   = deque(maxlen=self.WINDOW)

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("Roomba PID — Live Monitor", fontsize=13, fontweight='bold')

    def update(self, t, heading, l, r, pid_correction):
        self.times.append(t)
        self.headings.append(heading)
        self.left_pwm.append(l)
        self.right_pwm.append(r)
        self.pid_out.append(pid_correction)

    def draw(self):
        t = list(self.times)

        self.ax1.cla()
        self.ax1.plot(t, list(self.headings), 'b-', label='Heading (°)')
        self.ax1.axhline(0, color='r', linestyle='--', linewidth=1, label='Target (0°)')
        self.ax1.set_ylabel("Heading (degrees)")
        self.ax1.set_ylim(-45, 45)
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)

        self.ax2.cla()
        self.ax2.plot(t, list(self.left_pwm),  'g-', label='Left PWM')
        self.ax2.plot(t, list(self.right_pwm), 'm-', label='Right PWM')
        self.ax2.set_ylabel("PWM (0–255)")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylim(0, 260)
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.pause(0.001)


# ─── MAIN CONTROL LOOP ───────────────────────────────────────
def main():
    print("=" * 50)
    print("  Roomba Gas Detection Robot — PID Controller")
    print("=" * 50)

    heading_pid = PIDController(KP_HEAD, KI_HEAD, KD_HEAD,
                                output_min=-80, output_max=80)
    speed_pid   = PIDController(KP_SPEED, KI_SPEED, KD_SPEED,
                                output_min=-50, output_max=50)

    lidar   = LidarHeadingEstimator()
    motor   = MotorCommander(SERIAL_PORT, BAUD_RATE)
    plotter = LivePlotter()

    target_heading = 0.0   # degrees — go straight
    target_speed   = BASE_SPEED

    start_time = time.time()

    plt.ion()
    print("\n[PID] Running... Press Ctrl+C to stop.\n")

    try:
        while True:
            loop_start = time.time()
            elapsed    = loop_start - start_time

            # 1. Read LiDAR heading
            current_heading = lidar.get_heading()

            # 2. Heading PID → steering correction
            heading_correction = heading_pid.compute(target_heading,
                                                     current_heading,
                                                     dt=LOOP_DT)

            # 3. Speed PID (simple: keep base speed)
            speed_correction = speed_pid.compute(target_speed,
                                                 target_speed,   # swap for encoder speed
                                                 dt=LOOP_DT)

            # 4. Mix: differential drive
            #    positive heading_correction → turn left (reduce right, add left)
            left_pwm  = target_speed + speed_correction - heading_correction
            right_pwm = target_speed + speed_correction + heading_correction

            # 5. Send to Arduino
            motor.send(left_pwm, right_pwm)

            # 6. Log
            plotter.update(elapsed, current_heading,
                           left_pwm, right_pwm, heading_correction)

            print(f"[{elapsed:6.1f}s] Heading: {current_heading:+6.1f}°  "
                  f"Correction: {heading_correction:+6.1f}  "
                  f"L:{int(left_pwm):3d}  R:{int(right_pwm):3d}")

            # 7. Redraw plot every 10 loops
            if int(elapsed * LOOP_HZ) % 10 == 0:
                plotter.draw()

            # 8. Maintain loop rate
            elapsed_loop = time.time() - loop_start
            sleep_time   = LOOP_DT - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[PID] Stopping robot...")
        motor.stop()
        lidar.close()
        plt.ioff()
        plt.show()
        print("[PID] Done.")


if __name__ == "__main__":
    main()
