import time
import random
import math

# =============================================================================
# FAULT TYPE: Deployment Lag Escalation + Partial Deployment Lock
#
# FAULT DESCRIPTION:
#   Spoiler surfaces rely on fast, precise actuation — especially during
#   landing rollout where they must deploy within milliseconds of touchdown.
#   Two linked failure modes are modelled here:
#
#   (A) Lag Escalation — the actuator's servo valve develops internal
#       spool stiction (gumming from degraded hydraulic fluid). Each time
#       a deployment command is issued, the lag (time between command and
#       actual movement) grows. Early in the fault this is subtle — lag
#       goes from the normal 20-80ms range up to 200-400ms. In late stages
#       lag exceeds 600ms and the spoiler is clearly reacting too slowly.
#
#   (B) Partial Deployment Lock — as spool stiction worsens, the actuator
#       can only reach a fraction of the commanded angle before sticking.
#       The actual angle plateaus at a partial position (e.g. commanded 45deg,
#       actual freezes at 18deg). The motor current shows sustained high draw
#       because the controller keeps trying to close the error. Eventually
#       the actuator freezes entirely with a fixed partial angle and the
#       motor current collapses (controller gives up or trips overcurrent
#       protection). The sensor continues publishing the frozen state.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — Lag Growth (early, subtle):
#     Deployment lag grows beyond healthy range. Actual angle still reaches
#     commanded angle eventually, just with longer delay. Motor current is
#     normal. The spoiler is still functional, just slower.
#
#   Stage 2 — Partial Reach (middle, degraded):
#     Actual angle can only reach a progressively smaller fraction of the
#     commanded angle. Tracking error grows persistently. Motor current
#     reads high and sustained (controller fighting stiction). Lag continues
#     to grow with each new command.
#
#   Stage 3 — Full Lock (late, hard failure):
#     Actual angle freezes at whatever partial position it last reached.
#     Tracking error equals commanded minus frozen actual. Motor current
#     briefly spikes (overcurrent attempt) then collapses. Lag reports are
#     extreme (>800ms) but the command is never executed. Sensor publishes
#     these locked values continuously.
#
# TOTAL FAILURE INDICATOR:
#   - Actual angle frozen while commanded angle > 5 deg
#   - Tracking error equals commanded angle minus a fixed constant for
#     multiple consecutive ticks
#   - Motor current = 0 A while commanded > 5 deg (controller tripped)
#   - Lag reported > 800 ms persistently
#
# HEALTHY RANGES (for reference):
#   Takeoff  : commanded=0 deg (locked), actual=0, current=0, lag=0 ms
#   Cruise   : commanded=0-3 deg (suppressed), current=0-0.8 A, lag=15-45 ms
#   Landing  : commanded=30-60 deg, actual tracks, current=1.5-4.5 A, lag=20-150 ms
# =============================================================================


class SpoilerFaultInjector:
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

        # Stage 1 — Lag escalation
        self.lag_growth_rate = 0.0              # ms added per second of fault
        self.lag_current_extra = 0.0            # accumulated extra lag (ms)
        self.lag_growth_duration = 0.0          # seconds to reach peak lag

        # Stage 2 — Partial reach
        self.partial_reach_onset_fraction = 0.0 # progress fraction at which partial starts
        self.reach_fraction = 1.0               # fraction of commanded angle achievable (1 = full)
        self.reach_decay_rate = 0.0             # how fast reach_fraction falls per second
        self.current_boost = 0.0               # extra current during partial reach struggle

        # Stage 3 — Full lock
        self.lock_onset_fraction = 0.0          # progress fraction at which lock happens
        self.lock_active = False
        self.locked_actual_angle = None
        self.overcurrent_fired = False
        self.overcurrent_start_time = None
        self.overcurrent_magnitude = 0.0
        self.overcurrent_duration = 0.0

        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            # Takeoff spoilers are locked — fault builds silently for landing
            self.lag_growth_rate = random.uniform(8.0, 18.0)        # ms/s
            self.lag_growth_duration = random.uniform(16.0, 24.0)
            self.partial_reach_onset_fraction = random.uniform(0.35, 0.50)
            self.lock_onset_fraction = random.uniform(0.68, 0.82)
            self.reach_decay_rate = random.uniform(0.025, 0.055)
            self.current_boost = random.uniform(0.8, 1.6)
            self.overcurrent_magnitude = random.uniform(4.5, 6.0)
            self.overcurrent_duration = random.uniform(0.8, 1.5)

        elif current_phase == 'cruise':
            self.lag_growth_rate = random.uniform(10.0, 22.0)
            self.lag_growth_duration = random.uniform(20.0, 30.0)
            self.partial_reach_onset_fraction = random.uniform(0.30, 0.45)
            self.lock_onset_fraction = random.uniform(0.65, 0.80)
            self.reach_decay_rate = random.uniform(0.030, 0.060)
            self.current_boost = random.uniform(1.0, 2.0)
            self.overcurrent_magnitude = random.uniform(5.0, 6.5)
            self.overcurrent_duration = random.uniform(1.0, 2.0)

        elif current_phase == 'landing':
            self.lag_growth_rate = random.uniform(15.0, 30.0)
            self.lag_growth_duration = random.uniform(10.0, 16.0)
            self.partial_reach_onset_fraction = random.uniform(0.25, 0.40)
            self.lock_onset_fraction = random.uniform(0.58, 0.72)
            self.reach_decay_rate = random.uniform(0.040, 0.080)
            self.current_boost = random.uniform(1.2, 2.5)
            self.overcurrent_magnitude = random.uniform(5.5, 7.0)
            self.overcurrent_duration = random.uniform(1.2, 2.5)

        else:
            self.lag_growth_rate = random.uniform(8.0, 18.0)
            self.lag_growth_duration = random.uniform(15.0, 22.0)
            self.partial_reach_onset_fraction = random.uniform(0.32, 0.48)
            self.lock_onset_fraction = random.uniform(0.65, 0.80)
            self.reach_decay_rate = random.uniform(0.025, 0.050)
            self.current_boost = random.uniform(0.8, 1.5)
            self.overcurrent_magnitude = random.uniform(4.0, 5.5)
            self.overcurrent_duration = random.uniform(0.8, 1.5)

        self.reach_fraction = 1.0
        self.fault_configured = True

        print(
            f"[SPOILER_FAULT] Configured: lag_rate={self.lag_growth_rate:.1f}ms/s "
            f"over {self.lag_growth_duration:.1f}s | "
            f"partial@{self.partial_reach_onset_fraction*100:.0f}% | "
            f"lock@{self.lock_onset_fraction*100:.0f}%"
        )

    def trigger_fault(self):
        if self.fault_active:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            delay = random.uniform(1.0, 2.5)
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
            f"[SPOILER_FAULT] Servo spool stiction fault scheduled in {delay:.1f}s "
            f"during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(f"[SPOILER_FAULT] Phase transition: {self.last_known_phase} -> {current_phase}")

            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[SPOILER_FAULT] Cruise ended — degraded spoiler sensor continues publishing")

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
            print("[SPOILER_FAULT] ⚠️  Servo spool stiction INITIATED!")

        elapsed = current_time - self.fault_start_time
        progress = min(elapsed / max(self.lag_growth_duration, 0.001), 1.0)

        # Stage 1: lag grows with a square-root curve (fast early, flattens)
        self.lag_current_extra = self.lag_growth_rate * self.lag_growth_duration * math.sqrt(progress)

        # Stage 2: reach fraction decays once partial onset is passed
        if progress >= self.partial_reach_onset_fraction:
            partial_elapsed = elapsed - (self.partial_reach_onset_fraction * self.lag_growth_duration)
            self.reach_fraction = max(0.15, 1.0 - self.reach_decay_rate * max(partial_elapsed, 0.0))

        # Stage 3: lock
        if progress >= self.lock_onset_fraction and not self.lock_active:
            self.lock_active = True
            print("[SPOILER_FAULT] 🔴 FULL DEPLOYMENT LOCK — spoiler frozen, still publishing!")

    def inject_fault(self, commanded_angle, actual_angle, tracking_error, motor_current, lag_ms, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return commanded_angle, actual_angle, tracking_error, motor_current, lag_ms

        self.update_fault_progression()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return commanded_angle, actual_angle, tracking_error, motor_current, lag_ms

        elapsed = time.time() - self.fault_start_time if self.fault_start_time else 0.0

        # --- Lag escalation ---
        faulty_lag = int(lag_ms + self.lag_current_extra + random.gauss(0, 8.0))
        faulty_lag = max(0, faulty_lag)

        # --- Actual angle and current ---
        if self.lock_active:
            # Capture lock point once
            if self.locked_actual_angle is None:
                self.locked_actual_angle = actual_angle * self.reach_fraction
                print(f"[SPOILER_FAULT] Locked at {self.locked_actual_angle:.2f} deg")

            faulty_actual = self.locked_actual_angle + random.gauss(0, 0.04)
            faulty_actual = round(max(0.0, min(faulty_actual, 60.0)), 2)

            # Overcurrent attempt then collapse
            if not self.overcurrent_fired:
                if self.overcurrent_start_time is None:
                    self.overcurrent_start_time = time.time()
                    print(f"[SPOILER_FAULT] ⚡ OVERCURRENT: {self.overcurrent_magnitude:.1f}A!")

                spike_elapsed = time.time() - self.overcurrent_start_time
                if spike_elapsed < self.overcurrent_duration:
                    decay = 1.0 - (spike_elapsed / self.overcurrent_duration)
                    faulty_current = self.overcurrent_magnitude * decay + random.gauss(0, 0.12)
                else:
                    faulty_current = max(0.0, random.gauss(0.04, 0.02))
                    self.overcurrent_fired = True
            else:
                faulty_current = max(0.0, random.gauss(0.03, 0.02))

            faulty_lag = int(self.lag_current_extra * 1.5 + random.gauss(0, 15.0))
            faulty_lag = max(0, faulty_lag)

        elif self.reach_fraction < 1.0:
            # Partial reach: actual can only get to reach_fraction of commanded
            target_actual = commanded_angle * self.reach_fraction
            # Smooth move toward partial target with noise
            faulty_actual = actual_angle + (target_actual - actual_angle) * 0.4
            faulty_actual += random.gauss(0, 0.08)
            faulty_actual = round(max(0.0, min(faulty_actual, 60.0)), 2)

            # Current is high and sustained (controller fighting stiction)
            faulty_current = motor_current + self.current_boost
            faulty_current += random.gauss(0, 0.20)
            faulty_current = max(0.0, min(faulty_current, 7.0))

        else:
            # Stage 1: normal movement, just slower lag
            faulty_actual = actual_angle + random.gauss(0, 0.05)
            faulty_actual = round(max(0.0, min(faulty_actual, 60.0)), 2)
            faulty_current = motor_current + random.gauss(0, 0.06)
            faulty_current = max(0.0, faulty_current)

        faulty_error   = round(commanded_angle - faulty_actual, 2)
        faulty_current = round(min(faulty_current, 8.0), 2)

        return commanded_angle, faulty_actual, faulty_error, faulty_current, faulty_lag

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
            progress = min(elapsed / max(self.lag_growth_duration, 0.001), 1.0)

        stage = 'inactive'
        if self.is_fault_active():
            if self.lock_active:
                stage = 'lock'
            elif self.reach_fraction < 1.0:
                stage = 'partial_reach'
            else:
                stage = 'lag_growth'

        return {
            'fault_type': 'servo_spool_stiction_lock',
            'active': self.is_fault_active(),
            'stage': stage,
            'progress_fraction': round(progress, 3),
            'lag_extra_ms': round(self.lag_current_extra, 1),
            'reach_fraction': round(self.reach_fraction, 3),
            'lock_active': self.lock_active,
            'locked_actual_angle': self.locked_actual_angle,
            'overcurrent_fired': self.overcurrent_fired,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'sensor_name': 'spoiler',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# SpoilerFaultManager
# =============================================================================

class SpoilerFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'servo_spool_stiction_lock': SpoilerFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'servo_spool_stiction_lock'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[SPOILER FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[SPOILER FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, commanded_angle, actual_angle, tracking_error, motor_current, lag_ms, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(
                commanded_angle, actual_angle, tracking_error, motor_current, lag_ms, phase
            )
            return result
        else:
            return commanded_angle, actual_angle, tracking_error, motor_current, lag_ms

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
            'sensor_name': 'spoiler',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_spoiler_with_fault_injection
# =============================================================================

def update_spoiler_with_fault_injection(spoiler_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()

    healthy_result = spoiler_sensor.update_spoiler()

    original_commanded = healthy_result['commanded_angle_deg']
    original_actual    = healthy_result['actual_angle_deg']
    original_error     = healthy_result['tracking_error_deg']
    original_current   = healthy_result['motor_current_A']
    original_lag       = healthy_result['lag_ms']

    fault_result = fault_manager.inject_fault(
        original_commanded, original_actual, original_error,
        original_current, original_lag, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_commanded, faulty_actual, faulty_error, faulty_current, faulty_lag = fault_result

        if fault_manager.is_fault_active():
            output_commanded = faulty_commanded
            output_actual    = faulty_actual
            output_error     = faulty_error
            output_current   = faulty_current
            output_lag       = faulty_lag

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[SPOILER FAULT] Healthy → Cmd:{original_commanded:.2f}deg "
                    f"Act:{original_actual:.2f}deg "
                    f"Err:{original_error:.2f}deg "
                    f"I:{original_current:.2f}A "
                    f"Lag:{original_lag}ms"
                )
                print(
                    f"[SPOILER FAULT] Faulty  → Cmd:{faulty_commanded:.2f}deg "
                    f"Act:{faulty_actual:.2f}deg "
                    f"Err:{faulty_error:.2f}deg "
                    f"I:{faulty_current:.2f}A "
                    f"Lag:{faulty_lag}ms | "
                    f"Stage:{fault_status['stage'].upper()}"
                )
        else:
            output_commanded = original_commanded
            output_actual    = original_actual
            output_error     = original_error
            output_current   = original_current
            output_lag       = original_lag
    else:
        output_commanded = original_commanded
        output_actual    = original_actual
        output_error     = original_error
        output_current   = original_current
        output_lag       = original_lag

    bus.publish("wing/spoiler", {
        "sensor": "spoiler",
        "phase": current_phase,
        "commanded_angle": output_commanded,
        "actual_angle": output_actual,
        "error": output_error,
        "motor_current": output_current,
        "lag_ms": output_lag,
        "timestamp": time.time()
    })

    return {
        "sensor": "spoiler",
        "phase": current_phase,
        "commanded_angle_deg": output_commanded,
        "actual_angle_deg": output_actual,
        "tracking_error_deg": output_error,
        "motor_current_A": output_current,
        "lag_ms": output_lag,
    }