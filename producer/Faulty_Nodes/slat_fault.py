import time
import random
import math

# =============================================================================
# FAULT TYPE: Motor Current Degradation + Tracking Desynchronisation
#
# FAULT DESCRIPTION:
#   Slat actuators are driven by electric motors that push the slat panels
#   along rack-and-pinion tracks. Two linked failure modes are modelled here:
#
#   (A) Motor Winding Resistance Increase — progressive deterioration of the
#       motor windings (moisture ingress, thermal cycling) causes the motor to
#       draw less current than expected for a given load. This sounds
#       counter-intuitive but matches real-world observations: a degraded
#       winding produces less torque per amp, so the controller reduces current
#       demand to avoid overheating, at the cost of reduced actuator force.
#
#   (B) Tracking Desynchronisation — with reduced actuator force the slat
#       begins to lag behind its commanded position. The tracking error grows
#       from small (within tolerance) to persistently large. Eventually the
#       actual angle freezes at a partial-deployment position because the
#       motor can no longer produce enough torque to move the slat further,
#       while the commanded angle continues to change normally.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — Current Erosion (early, gradual):
#     Motor current reads consistently below the expected range for the load.
#     Tracking error is slightly elevated but not alarming. The actual angle
#     still tracks the commanded angle, just with a growing lag that is
#     proportional to how much the current has dropped.
#
#   Stage 2 — Desync Onset (middle, worsening):
#     The tracking error grows beyond the normal noise floor and becomes
#     persistent — the actual angle falls further and further behind the
#     commanded angle on every tick. Motor current continues to read low.
#     The actual angle can still move, but it is clearly not keeping up.
#
#   Stage 3 — Partial Freeze (late, hard fault):
#     The actual angle freezes near the last position it could reach.
#     The commanded angle continues to update normally (the flight computer
#     still sends commands), but the actual angle is pinned with only
#     small sensor noise. Tracking error now equals the full span between
#     commanded and frozen actual. Motor current collapses toward zero
#     as the controller detects zero movement and reduces drive.
#     The sensor continues publishing this frozen-actual / normal-command
#     divergence for the rest of the flight.
#
# TOTAL FAILURE INDICATOR:
#   - Tracking error persistently growing, not reverting to near-zero
#   - Motor current below 0.15 A while commanded angle > 1 deg
#   - Actual angle frozen (delta < 0.05 deg across multiple ticks) while
#     commanded angle is changing
#
# HEALTHY RANGES (for reference):
#   Takeoff  : commanded=12.5 deg, current=0.2-4.5 A
#   Cruise   : commanded=0 deg, current=0 A
#   Landing  : commanded=6-10 deg (dynamic), current=0.3-4.5 A
# =============================================================================


class SlatFaultInjector:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.fault_active = False
        self.fault_start_time = None
        self.fault_trigger_time = None
        self.flight_start_time = time.time()
        self.fault_was_triggered = False

        # Sensor always keeps publishing
        self.should_stop_publishing = False
        self.stop_publishing_triggered = False
        self.cruise_phase_ended = False
        self.last_known_phase = None

        # Stage 1 — Current erosion
        self.current_reduction_target = 0.0     # fraction of current to suppress (0–1)
        self.current_reduction_now = 0.0        # current accumulated suppression
        self.erosion_duration = 0.0             # seconds for erosion to reach target

        # Stage 2 — Desync
        self.desync_onset_fraction = 0.0        # progress fraction at which desync starts
        self.angle_lag_bias = 0.0               # extra degrees the actual angle lags behind
        self.lag_accumulation_rate = 0.0        # degrees of lag added per second

        # Stage 3 — Freeze
        self.freeze_onset_fraction = 0.0        # progress fraction at which freeze starts
        self.freeze_active = False
        self.frozen_actual_angle = None         # angle at which actual froze
        self.frozen_noise_std = 0.0             # small noise still present in frozen reading

        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            self.current_reduction_target = random.uniform(0.55, 0.80)
            self.erosion_duration = random.uniform(15.0, 22.0)
            self.desync_onset_fraction = random.uniform(0.30, 0.45)
            self.freeze_onset_fraction = random.uniform(0.65, 0.80)
            self.lag_accumulation_rate = random.uniform(0.08, 0.18)
            self.frozen_noise_std = random.uniform(0.02, 0.06)

        elif current_phase == 'cruise':
            # Cruise slats are stowed — degradation accumulates invisibly,
            # most visible impact lands on the next landing phase
            self.current_reduction_target = random.uniform(0.60, 0.85)
            self.erosion_duration = random.uniform(18.0, 28.0)
            self.desync_onset_fraction = random.uniform(0.35, 0.50)
            self.freeze_onset_fraction = random.uniform(0.68, 0.82)
            self.lag_accumulation_rate = random.uniform(0.10, 0.22)
            self.frozen_noise_std = random.uniform(0.02, 0.05)

        elif current_phase == 'landing':
            self.current_reduction_target = random.uniform(0.50, 0.75)
            self.erosion_duration = random.uniform(10.0, 16.0)
            self.desync_onset_fraction = random.uniform(0.25, 0.38)
            self.freeze_onset_fraction = random.uniform(0.55, 0.70)
            self.lag_accumulation_rate = random.uniform(0.12, 0.28)
            self.frozen_noise_std = random.uniform(0.03, 0.07)

        else:
            self.current_reduction_target = random.uniform(0.55, 0.75)
            self.erosion_duration = random.uniform(14.0, 20.0)
            self.desync_onset_fraction = random.uniform(0.30, 0.45)
            self.freeze_onset_fraction = random.uniform(0.65, 0.78)
            self.lag_accumulation_rate = random.uniform(0.08, 0.18)
            self.frozen_noise_std = random.uniform(0.02, 0.05)

        self.fault_configured = True
        print(
            f"[SLAT_FAULT] Configured: current_reduction={self.current_reduction_target:.2f} "
            f"over {self.erosion_duration:.1f}s | "
            f"desync@{self.desync_onset_fraction*100:.0f}% | "
            f"freeze@{self.freeze_onset_fraction*100:.0f}%"
        )

    def trigger_fault(self):
        if self.fault_active:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            delay = random.uniform(1.5, 3.0)
        elif current_phase == 'cruise':
            delay = random.uniform(2.0, 5.0)
        elif current_phase == 'landing':
            delay = random.uniform(0.0, 1.5)
        else:
            delay = random.uniform(1.0, 2.5)

        self.fault_trigger_time = time.time() + delay
        self.fault_active = True
        self.fault_was_triggered = True

        print(
            f"[SLAT_FAULT] Motor winding fault scheduled in {delay:.1f}s "
            f"during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(f"[SLAT_FAULT] Phase transition: {self.last_known_phase} -> {current_phase}")

            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[SLAT_FAULT] Cruise ended — degraded slat sensor continues publishing")

            self.last_known_phase = current_phase

    def update_fault_progression(self):
        if not self.fault_active or self.fault_trigger_time is None:
            return

        current_time = time.time()

        if current_time < self.fault_trigger_time:
            return

        if self.fault_start_time is None:
            self.fault_start_time = current_time
            self.configure_fault()
            print("[SLAT_FAULT] ⚠️  Motor winding degradation INITIATED!")

        elapsed = current_time - self.fault_start_time
        progress = min(elapsed / max(self.erosion_duration, 0.001), 1.0)

        # Current erosion follows a concave curve — fast early, slows at plateau
        erosion_curve = 1.0 - math.exp(-4.0 * progress)
        self.current_reduction_now = self.current_reduction_target * erosion_curve

        # Lag accumulates linearly once desync stage starts
        if progress >= self.desync_onset_fraction:
            desync_elapsed = elapsed - (self.desync_onset_fraction * self.erosion_duration)
            self.angle_lag_bias = self.lag_accumulation_rate * max(desync_elapsed, 0.0)

        # Freeze activates once threshold is crossed
        if progress >= self.freeze_onset_fraction and not self.freeze_active:
            self.freeze_active = True
            print("[SLAT_FAULT] 🔴 PARTIAL FREEZE — actual angle locked, still publishing!")

    def inject_fault(self, commanded_angle, actual_angle, tracking_error, motor_current, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return commanded_angle, actual_angle, tracking_error, motor_current

        self.update_fault_progression()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return commanded_angle, actual_angle, tracking_error, motor_current

        # --- Motor current suppression ---
        faulty_current = motor_current * (1.0 - self.current_reduction_now)
        faulty_current += random.gauss(0, 0.04)
        faulty_current = max(0.0, faulty_current)

        # --- Actual angle ---
        if self.freeze_active:
            # Capture freeze point once
            if self.frozen_actual_angle is None:
                self.frozen_actual_angle = actual_angle
                print(f"[SLAT_FAULT] Frozen at {self.frozen_actual_angle:.2f} deg")

            faulty_actual = self.frozen_actual_angle + random.gauss(0, self.frozen_noise_std)
            faulty_actual = round(max(0.0, min(faulty_actual, 25.0)), 2)

            # Current collapses as controller detects no movement
            faulty_current = max(0.0, random.gauss(0.05, 0.03))

        else:
            # Desync: actual lags by growing bias
            faulty_actual = actual_angle - self.angle_lag_bias
            faulty_actual += random.gauss(0, 0.04)
            faulty_actual = round(max(0.0, min(faulty_actual, 25.0)), 2)

        # --- Tracking error recomputed from faulty values ---
        faulty_error = round(commanded_angle - faulty_actual, 2)

        faulty_current = round(min(faulty_current, 6.0), 2)

        return commanded_angle, faulty_actual, faulty_error, faulty_current

    def is_fault_active(self):
        return (
            self.fault_active
            and self.fault_trigger_time is not None
            and time.time() >= self.fault_trigger_time
        )

    def is_publishing_stopped(self):
        return self.should_stop_publishing

    def get_fault_status(self):
        elapsed = 0.0
        progress = 0.0
        if self.fault_start_time is not None:
            elapsed = time.time() - self.fault_start_time
            progress = min(elapsed / max(self.erosion_duration, 0.001), 1.0)

        stage = 'inactive'
        if self.is_fault_active():
            if self.freeze_active:
                stage = 'freeze'
            elif progress >= self.desync_onset_fraction:
                stage = 'desync'
            else:
                stage = 'erosion'

        return {
            'fault_type': 'motor_winding_tracking_desync',
            'active': self.is_fault_active(),
            'stage': stage,
            'progress_fraction': round(progress, 3),
            'current_reduction_now': round(self.current_reduction_now, 3),
            'current_reduction_target': self.current_reduction_target,
            'angle_lag_bias': round(self.angle_lag_bias, 3),
            'freeze_active': self.freeze_active,
            'frozen_actual_angle': self.frozen_actual_angle,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'sensor_name': 'slat',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# SlatFaultManager
# =============================================================================

class SlatFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'motor_winding_tracking_desync': SlatFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'motor_winding_tracking_desync'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[SLAT FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[SLAT FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, commanded_angle, actual_angle, tracking_error, motor_current, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(
                commanded_angle, actual_angle, tracking_error, motor_current, phase
            )
            return result
        else:
            return commanded_angle, actual_angle, tracking_error, motor_current

    def is_fault_active(self):
        return self.current_fault is not None and self.current_fault.is_fault_active()

    def is_publishing_stopped(self):
        return self.current_fault is not None and self.current_fault.is_publishing_stopped()

    def get_fault_status(self):
        if self.current_fault is not None:
            return self.current_fault.get_fault_status()
        return {
            'fault_type': 'none',
            'active': False,
            'sensor_name': 'slat',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_slat_with_fault_injection
# =============================================================================

def update_slat_with_fault_injection(slat_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()

    healthy_result = slat_sensor.update_slats()

    original_commanded = healthy_result['commanded_angle_deg']
    original_actual    = healthy_result['actual_angle_deg']
    original_error     = healthy_result['tracking_error_deg']
    original_current   = healthy_result['motor_current_A']

    fault_result = fault_manager.inject_fault(
        original_commanded, original_actual, original_error, original_current, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_commanded, faulty_actual, faulty_error, faulty_current = fault_result

        if fault_manager.is_fault_active():
            output_commanded = faulty_commanded
            output_actual    = faulty_actual
            output_error     = faulty_error
            output_current   = faulty_current

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[SLAT FAULT] Healthy → Cmd:{original_commanded:.2f}deg "
                    f"Act:{original_actual:.2f}deg "
                    f"Err:{original_error:.2f}deg "
                    f"I:{original_current:.2f}A"
                )
                print(
                    f"[SLAT FAULT] Faulty  → Cmd:{faulty_commanded:.2f}deg "
                    f"Act:{faulty_actual:.2f}deg "
                    f"Err:{faulty_error:.2f}deg "
                    f"I:{faulty_current:.2f}A | "
                    f"Stage:{fault_status['stage'].upper()}"
                )
        else:
            output_commanded = original_commanded
            output_actual    = original_actual
            output_error     = original_error
            output_current   = original_current
    else:
        output_commanded = original_commanded
        output_actual    = original_actual
        output_error     = original_error
        output_current   = original_current

    bus.publish("wing/slat", {
        "sensor": "slat",
        "phase": current_phase,
        "commanded_angle": output_commanded,
        "actual_angle": output_actual,
        "error": output_error,
        "motor_current": output_current,
        "timestamp": time.time()
    })

    return {
        "sensor": "slat",
        "phase": current_phase,
        "commanded_angle_deg": output_commanded,
        "actual_angle_deg": output_actual,
        "tracking_error_deg": output_error,
        "motor_current_A": output_current,
    }