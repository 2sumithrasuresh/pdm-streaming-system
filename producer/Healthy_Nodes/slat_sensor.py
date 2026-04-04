import time
import random

class SlatSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.commanded_angle = 0.0
        self.command_target = 0.0
        self.actual_angle = 0.0
        self.motor_current = 0.0

        self.last_time = time.time()
        self.command_update_time = time.time()
        self.current_phase = None
        self.phase_initialized = False

    def on_phase_change(self, phase):
        self.phase_initialized = True
        self.commanded_angle = 0.0
        self.actual_angle = 0.0
        self.motor_current = 0.0
        self.command_target = 0.0
        self.command_update_time = time.time()

        if phase == 'takeoff':
            self.command_target = 12.5  # Fixed deployment during takeoff
        elif phase == 'cruise':
            self.command_target = 0.0   # Fully stowed
        elif phase == 'landing':
            self.command_target = round(random.uniform(6.0, 10.0), 2)  # Dynamic, realistic

    def simulate_commanded_angle(self, phase):
        dt = 1.0
        now = time.time()
        if phase == 'landing' and now - self.command_update_time > 5.0:
            self.command_target = round(random.uniform(6.0, 10.0), 2)
            self.command_update_time = now

        if phase == 'cruise':
            return 0.0

        # Gradually move toward target
        max_change = {
            'takeoff': 1.0,
            'landing': 1.2
        }.get(phase, 0.4)

        diff = self.command_target - self.commanded_angle
        step = max(-max_change, min(max_change, diff))

        if abs(diff) > 0.2:
            noise = random.gauss(0, 0.05)
        else:
            noise = 0.0

        self.commanded_angle += step + noise
        self.commanded_angle = round(max(0.0, min(self.commanded_angle, 25.0)), 2)

        return self.commanded_angle

    def update_slats(self):
        phase = self.phase_controller.get_current_phase()
        dt = 1.0
        now = time.time()
        # Detect phase change
        if phase != self.current_phase:
            self.on_phase_change(phase)
            self.current_phase = phase

        self.commanded_angle = self.simulate_commanded_angle(phase)
        error = self.commanded_angle - self.actual_angle

        # === Movement Step Logic ===
        max_rate = 35.0 if phase in ('takeoff', 'landing') else 10.0
        max_step = max_rate * dt
        step = max(-max_step, min(error, max_step))

        # === Dynamic Adjustment ===
        if abs(error) > 2.0:
            step *= 1.2
        elif 0.15 < abs(error) < 0.5:
            step *= 1.05

        # === Noise Injection ===
        if abs(error) > 0.3:
            noise = random.gauss(0, 0.05 if phase != 'cruise' else 0.0)
        else:
            noise = 0.0

        # === Actual Angle Update ===
        self.actual_angle += step + noise
        self.actual_angle = round(max(0.0, min(self.actual_angle, 25.0)), 2)

        tracking_error = round(self.commanded_angle - self.actual_angle, 2)

        # === Motor Current Simulation ===
        abs_step = abs(step)
        abs_err = abs(tracking_error)

        if phase == 'cruise':
            self.motor_current = 0.0
        elif abs_err > 1.5 or abs_step > 1.0:
            self.motor_current = {
                'landing': random.uniform(3.5, 5.5),
                'takeoff': random.uniform(2.5, 4.5)
            }.get(phase, 0.0)
        elif abs_err > 0.4 or abs_step > 0.3:
            self.motor_current = {
                'landing': random.uniform(1.5, 3.0),
                'takeoff': random.uniform(1.0, 2.5)
            }.get(phase, 0.0)
        else:
            self.motor_current = {
                'landing': random.uniform(0.3, 0.6),
                'takeoff': random.uniform(0.2, 0.5)
            }.get(phase, 0.0)

        self.motor_current = round(min(self.motor_current, 4.5), 2)

        # === Logging ===
        print(
            f"[{phase.upper()}] Command: {self.commanded_angle:.2f}°, "
            f"Actual: {self.actual_angle:.2f}°, "
            f"Error: {tracking_error:.2f}°, "
            f"Motor: {self.motor_current:.2f} A"
        )

        # === Publishing ===
        self.bus.publish("wing/slat", {
            "sensor": "slat",
            "timestamp": time.time(),
            "phase": phase,
            "commanded_angle": self.commanded_angle,
            "actual_angle": self.actual_angle,
            "error": tracking_error,
            "motor_current": self.motor_current
        })
 
        return {
            "sensor": "slat",
            "phase": phase,
            "commanded_angle_deg": self.commanded_angle,
            "actual_angle_deg": self.actual_angle,
            "tracking_error_deg": tracking_error,
            "motor_current_A": self.motor_current
        }
