import time
import random
import math

# =============================================================================
# FAULT TYPE: Hydraulic Pressure Bleed-Down + Actuator Stiction
#
# FAULT DESCRIPTION:
#   Real aircraft flap hydraulic systems can suffer from internal seal
#   degradation, which causes the hydraulic circuit to slowly lose pressure
#   (bleed-down). As pressure drops, the actuator motor must work harder to
#   compensate, drawing more current. Eventually, insufficient hydraulic force
#   causes the actuator to "stick" — it can no longer overcome static friction
#   and stops moving, freezing the flap at whatever angle it last reached.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — Pressure Bleed (early, gradual):
#     Hydraulic pressure begins declining below its phase-nominal value.
#     The actuator compensates by increasing motor current. Flap angle
#     still tracks the target but with growing lag (slow response).
#     The motor current rises above normal as it fights against reduced
#     hydraulic assistance. Pressure drops at a slow, realistic rate
#     (simulating a seal weeping rather than a rupture).
#
#   Stage 2 — Stiction Onset (middle, intermittent):
#     Pressure has fallen enough that the actuator begins to exhibit stiction —
#     brief freezing followed by sudden catches. The flap angle oscillates
#     around a stiction point rather than smoothly tracking the target.
#     Motor current shows erratic spikes (high current to overcome stiction,
#     then sudden drop when the actuator breaks free). Pressure continues
#     bleeding but more rapidly as the seal degrades further.
#
#   Stage 3 — Full Lock (late, hard failure):
#     Hydraulic pressure falls below the minimum threshold required for
#     actuation. The flap freezes completely — angle stops changing even
#     though the target angle may differ. Motor current surges briefly
#     (overcurrent event) then drops to near-zero as the actuator gives up.
#     Pressure reads near-zero or flatlines. The sensor continues publishing
#     these degraded/locked values throughout the rest of the flight.
#
# TOTAL FAILURE INDICATOR:
#   - Flap angle frozen constant with delta < 0.01 deg across multiple ticks
#   - Hydraulic pressure flatlined near 80 psi (far below 2500 psi minimum)
#   - Motor current post-overcurrent collapses to ~0.04 A
#
# HEALTHY RANGES (for reference):
#   Takeoff  : angle=15 deg, pressure=2700-3450 psi, current=1.5-2.5 A
#   Cruise   : angle=0 deg,  pressure=0 psi (locked), current=0 A
#   Landing  : angle=30 deg, pressure=3000-3450 psi, current=2.0-3.0 A
# =============================================================================


class FlapFaultInjector:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.fault_active = False
        self.fault_start_time = None
        self.fault_trigger_time = None
        self.flight_start_time = time.time()
        self.fault_was_triggered = False

        # Sensor always keeps publishing degraded values
        self.should_stop_publishing = False
        self.stop_publishing_triggered = False
        self.cruise_phase_ended = False
        self.last_known_phase = None

        # Stage 1 - Hydraulic bleed-down
        self.pressure_bleed_rate = 0.0
        self.pressure_current_loss = 0.0
        self.pressure_bleed_target = 0.0
        self.bleed_duration = 0.0

        # Stage 2 - Stiction
        self.stiction_onset_threshold = 0.0
        self.stiction_angle_freeze = None
        self.stiction_release_probability = 0.0
        self.stiction_current_spike = 0.0
        self.in_stiction = False
        self.stiction_ticks = 0
        self.max_stiction_ticks = 0

        # Stage 3 - Full lock
        self.full_lock_threshold = 0.0
        self.full_lock_active = False
        self.overcurrent_event_done = False
        self.overcurrent_magnitude = 0.0
        self.overcurrent_duration = 0.0
        self.overcurrent_start_time = None

        self.motor_current_bias = 0.0
        self.max_motor_bias = 0.0
        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            self.pressure_bleed_target = random.uniform(1800, 2400)
            self.bleed_duration = random.uniform(18.0, 26.0)
            self.stiction_onset_threshold = random.uniform(900, 1300)
            self.full_lock_threshold = random.uniform(1600, 2200)
            self.stiction_release_probability = random.uniform(0.35, 0.55)
            self.stiction_current_spike = random.uniform(3.8, 4.5)
            self.max_motor_bias = random.uniform(0.8, 1.4)
            self.max_stiction_ticks = random.randint(2, 4)
            self.overcurrent_magnitude = random.uniform(4.5, 5.5)
            self.overcurrent_duration = random.uniform(0.8, 1.5)

        elif current_phase == 'cruise':
            self.pressure_bleed_target = random.uniform(2000, 2800)
            self.bleed_duration = random.uniform(20.0, 30.0)
            self.stiction_onset_threshold = random.uniform(1000, 1500)
            self.full_lock_threshold = random.uniform(1800, 2500)
            self.stiction_release_probability = random.uniform(0.25, 0.45)
            self.stiction_current_spike = random.uniform(4.0, 4.8)
            self.max_motor_bias = random.uniform(1.0, 1.8)
            self.max_stiction_ticks = random.randint(3, 5)
            self.overcurrent_magnitude = random.uniform(5.0, 6.0)
            self.overcurrent_duration = random.uniform(1.0, 2.0)

        elif current_phase == 'landing':
            self.pressure_bleed_target = random.uniform(1500, 2200)
            self.bleed_duration = random.uniform(12.0, 18.0)
            self.stiction_onset_threshold = random.uniform(700, 1100)
            self.full_lock_threshold = random.uniform(1300, 2000)
            self.stiction_release_probability = random.uniform(0.20, 0.38)
            self.stiction_current_spike = random.uniform(4.2, 5.2)
            self.max_motor_bias = random.uniform(1.2, 2.0)
            self.max_stiction_ticks = random.randint(4, 7)
            self.overcurrent_magnitude = random.uniform(5.5, 6.5)
            self.overcurrent_duration = random.uniform(1.2, 2.5)

        else:
            self.pressure_bleed_target = random.uniform(1500, 2000)
            self.bleed_duration = random.uniform(15.0, 22.0)
            self.stiction_onset_threshold = random.uniform(800, 1200)
            self.full_lock_threshold = random.uniform(1400, 1900)
            self.stiction_release_probability = random.uniform(0.30, 0.50)
            self.stiction_current_spike = random.uniform(3.5, 4.5)
            self.max_motor_bias = random.uniform(0.7, 1.2)
            self.max_stiction_ticks = random.randint(2, 4)
            self.overcurrent_magnitude = random.uniform(4.0, 5.0)
            self.overcurrent_duration = random.uniform(0.8, 1.5)

        self.pressure_bleed_rate = self.pressure_bleed_target / self.bleed_duration
        self.fault_configured = True

        print(
            f"[FLAP_FAULT] Configured: bleed_target={self.pressure_bleed_target:.0f} psi "
            f"over {self.bleed_duration:.1f}s | "
            f"stiction_onset={self.stiction_onset_threshold:.0f} psi loss | "
            f"full_lock={self.full_lock_threshold:.0f} psi loss"
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
            f"[FLAP_FAULT] Hydraulic seal fault scheduled in {delay:.1f}s "
            f"during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(f"[FLAP_FAULT] Phase transition: {self.last_known_phase} -> {current_phase}")

            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[FLAP_FAULT] Cruise phase ended — degraded sensor continues publishing")

            self.last_known_phase = current_phase

    def update_pressure_bleed(self):
        if not self.fault_active or self.fault_trigger_time is None:
            return

        current_time = time.time()

        if current_time < self.fault_trigger_time:
            return

        if self.fault_start_time is None:
            self.fault_start_time = current_time
            self.configure_fault()
            print("[FLAP_FAULT] ⚠️  Hydraulic bleed-down INITIATED!")

        elapsed = current_time - self.fault_start_time

        # Non-linear bleed curve: slow pinhole -> accelerating crack
        if elapsed < self.bleed_duration:
            progress = elapsed / self.bleed_duration
            bleed_curve = progress ** 1.4
            self.pressure_current_loss = self.pressure_bleed_target * bleed_curve
        else:
            self.pressure_current_loss = self.pressure_bleed_target

        # Motor current bias rises proportionally to pressure loss
        loss_fraction = min(self.pressure_current_loss / max(self.full_lock_threshold, 1.0), 1.0)
        self.motor_current_bias = self.max_motor_bias * loss_fraction

        # Stage 2: stiction onset check
        if self.pressure_current_loss >= self.stiction_onset_threshold and not self.full_lock_active:
            if not self.in_stiction:
                if random.random() < 0.30:
                    self.in_stiction = True
                    self.stiction_ticks = 0
                    print("[FLAP_FAULT] ⚠️  Stiction episode started")

        # Stage 3: full lock check
        if self.pressure_current_loss >= self.full_lock_threshold:
            if not self.full_lock_active:
                self.full_lock_active = True
                self.in_stiction = False
                print("[FLAP_FAULT] 🔴 FULL HYDRAULIC LOCK — flap frozen, still publishing!")

    def inject_fault(self, flap_angle, motor_current, hydraulic_pressure, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return flap_angle, motor_current, hydraulic_pressure

        self.update_pressure_bleed()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return flap_angle, motor_current, hydraulic_pressure

        # --- Hydraulic pressure ---
        pressure_noise = random.gauss(0, 12.0)
        faulty_pressure = max(0.0, hydraulic_pressure - self.pressure_current_loss + pressure_noise)

        if self.full_lock_active:
            faulty_pressure = max(0.0, random.gauss(80.0, 25.0))

        # --- Motor current ---
        if self.full_lock_active:
            if not self.overcurrent_event_done:
                if self.overcurrent_start_time is None:
                    self.overcurrent_start_time = time.time()
                    print(f"[FLAP_FAULT] ⚡ OVERCURRENT EVENT: {self.overcurrent_magnitude:.1f}A spike!")

                spike_elapsed = time.time() - self.overcurrent_start_time
                if spike_elapsed < self.overcurrent_duration:
                    spike_decay = 1.0 - (spike_elapsed / self.overcurrent_duration)
                    faulty_current = self.overcurrent_magnitude * spike_decay
                    faulty_current += random.gauss(0, 0.15)
                else:
                    faulty_current = max(0.0, random.gauss(0.05, 0.03))
                    self.overcurrent_event_done = True
            else:
                faulty_current = max(0.0, random.gauss(0.04, 0.02))

        elif self.in_stiction:
            faulty_current = motor_current + self.motor_current_bias
            faulty_current += random.gauss(0, 0.25)
            if random.random() < 0.20:
                faulty_current += random.uniform(0.5, max(0.5, self.stiction_current_spike - motor_current))

        else:
            faulty_current = motor_current + self.motor_current_bias
            faulty_current += random.gauss(0, 0.08)

        faulty_current = max(0.0, faulty_current)

        # --- Flap angle ---
        if self.full_lock_active:
            if self.stiction_angle_freeze is None:
                self.stiction_angle_freeze = flap_angle
            faulty_angle = self.stiction_angle_freeze + random.gauss(0, 0.03)

        elif self.in_stiction:
            self.stiction_ticks += 1

            if self.stiction_angle_freeze is None:
                self.stiction_angle_freeze = flap_angle

            oscillation = 0.08 * math.sin(self.stiction_ticks * 1.2) * random.uniform(0.8, 1.2)
            faulty_angle = self.stiction_angle_freeze + oscillation

            if (
                self.stiction_ticks >= self.max_stiction_ticks
                or random.random() < self.stiction_release_probability
            ):
                self.in_stiction = False
                self.stiction_angle_freeze = None
                self.stiction_ticks = 0
                print("[FLAP_FAULT] Stiction released — actuator broke free")

        else:
            lag_factor = min(
                self.pressure_current_loss / max(self.stiction_onset_threshold, 1.0), 1.0
            )
            lag_noise = random.gauss(0, 0.05 + 0.15 * lag_factor)
            faulty_angle = flap_angle + lag_noise

        faulty_angle    = round(max(0.0, min(faulty_angle, 35.0)), 2)
        faulty_current  = round(min(faulty_current, 7.0), 2)
        faulty_pressure = round(min(faulty_pressure, 3500.0), 2)

        return faulty_angle, faulty_current, faulty_pressure

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
        if self.fault_start_time is not None:
            elapsed = time.time() - self.fault_start_time

        return {
            'fault_type': 'hydraulic_bleed_stiction',
            'active': self.is_fault_active(),
            'pressure_loss_current': self.pressure_current_loss,
            'pressure_bleed_target': self.pressure_bleed_target,
            'motor_current_bias': self.motor_current_bias,
            'in_stiction': self.in_stiction,
            'full_lock_active': self.full_lock_active,
            'overcurrent_event_done': self.overcurrent_event_done,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'sensor_name': 'flap',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# FlapFaultManager
# =============================================================================

class FlapFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'hydraulic_bleed_stiction': FlapFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'hydraulic_bleed_stiction'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[FLAP FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[FLAP FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, flap_angle, motor_current, hydraulic_pressure, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(
                flap_angle, motor_current, hydraulic_pressure, phase
            )
            return result
        else:
            return flap_angle, motor_current, hydraulic_pressure

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
            'sensor_name': 'flap',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_flap_with_fault_injection
# =============================================================================

def update_flap_with_fault_injection(flap_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()

    healthy_result = flap_sensor.update_flap_sensor()

    original_angle    = healthy_result['flap_angle']
    original_current  = healthy_result['motor_current']
    original_pressure = healthy_result['hydraulic_pressure']

    fault_result = fault_manager.inject_fault(
        original_angle, original_current, original_pressure, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_angle, faulty_current, faulty_pressure = fault_result

        if fault_manager.is_fault_active():
            output_angle    = faulty_angle
            output_current  = faulty_current
            output_pressure = faulty_pressure

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[FLAP FAULT] Healthy → Angle:{original_angle:.2f}deg "
                    f"Current:{original_current:.2f}A "
                    f"Pressure:{original_pressure:.0f}psi"
                )
                print(
                    f"[FLAP FAULT] Faulty  → Angle:{faulty_angle:.2f}deg "
                    f"Current:{faulty_current:.2f}A "
                    f"Pressure:{faulty_pressure:.0f}psi | "
                    f"Loss:{fault_status['pressure_loss_current']:.0f}psi"
                )
        else:
            output_angle    = original_angle
            output_current  = original_current
            output_pressure = original_pressure
    else:
        output_angle    = original_angle
        output_current  = original_current
        output_pressure = original_pressure

    bus.publish("wing/flap", {
        "sensor": "flap",
        "phase": current_phase,
        "flap_angle": output_angle,
        "motor_current": output_current,
        "hydraulic_pressure": output_pressure,
        "timestamp": time.time()
    })

    return {
        "sensor": "flap",
        "phase": current_phase,
        "flap_angle": output_angle,
        "motor_current": output_current,
        "hydraulic_pressure": output_pressure,
    }