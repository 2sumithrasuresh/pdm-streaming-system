import time
import random
import math

# =============================================================================
# FAULT TYPE: Strain Gauge Drift + Spurious Overload Bias
#
# FAULT DESCRIPTION:
#   Wing spar load sensors use bonded strain gauges in a Wheatstone bridge
#   configuration. Two linked failure modes are modelled here:
#
#   (A) Strain Gauge Drift — one arm of the Wheatstone bridge develops a
#       resistance offset due to adhesive creep (the gauge partially
#       de-bonds from the spar surface). This causes all three load channels
#       (bending moment, shear force, axial load) to drift upward from their
#       true values, since the bridge output is no longer balanced. The drift
#       is slow and progressive — millimetres of de-bonding over seconds.
#
#   (B) Spurious Overload Bias — the unbalanced bridge also amplifies dynamic
#       loads incorrectly. During phases with high structural loading (takeoff,
#       landing), the sensor begins reporting bending moments and shear forces
#       that trend significantly above the true structural load. The readings
#       can eventually reach values that, if real, would indicate a structural
#       overload event — even though the aircraft is flying normally. This is
#       particularly dangerous because it could trigger false structural
#       warnings or cause unnecessary flight envelope restrictions.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — Baseline Drift (early, subtle):
#     All three load channels read slightly above their true values.
#     The drift is proportional across channels — bending, shear, and axial
#     all drift by a consistent multiplier above healthy levels. The readings
#     remain within normal variance for their phase but trend upward over time.
#     Individual ticks may look normal; only a time-series analysis reveals
#     the consistent upward bias.
#
#   Stage 2 — Amplification Onset (middle, worsening):
#     The upward drift accelerates. Bending moment diverges fastest (largest
#     gauge area, most affected by de-bonding). Shear force follows at a
#     slower rate. Axial load drifts least of all. The readings begin to
#     exceed the normal phase ranges and approach values associated with
#     heavy turbulence or hard maneuvers, even during calm cruise.
#     Cross-channel inconsistency becomes detectable — the ratio of
#     bending to shear diverges from its healthy value.
#
#   Stage 3 — Spurious Overload (late, dangerous false readings):
#     Bending moment and shear force report values in the range associated
#     with near-limit structural loading (bending > 60000 Nm during cruise,
#     shear > 30000 N). Axial load shows large oscillations due to bridge
#     imbalance causing coupling between channels. The sensor continues
#     publishing these spurious overload values — they do not reflect real
#     structural state but would appear alarming to any monitoring system.
#
# TOTAL FAILURE INDICATOR:
#   - Bending moment > 65000 Nm during cruise (physically implausible)
#   - Shear / bending ratio deviates > 40% from healthy ~0.5 ratio
#   - Axial load oscillating with amplitude > 15000 N with no flight maneuver
#   - All three channels trending upward simultaneously (not random noise)
#
# HEALTHY RANGES (for reference):
#   Takeoff  : bending=8000-53000 Nm, shear=4000-24000 N, axial=6000-41000 N
#   Cruise   : bending=~12000 Nm (+/-2000), shear=~6000 N, axial=~8000 N
#   Landing  : bending=35000-62000 Nm, shear=18000-28000 N, axial=-25000 to 50000 N
# =============================================================================


class WingSparLoadFaultInjector:
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

        # Stage 1 — Baseline drift biases (Nm / N)
        self.bending_drift_target = 0.0         # total bending moment offset at plateau
        self.shear_drift_target = 0.0
        self.axial_drift_target = 0.0
        self.drift_duration = 0.0               # seconds to reach drift targets

        self.bending_drift_now = 0.0            # current accumulated drift
        self.shear_drift_now = 0.0
        self.axial_drift_now = 0.0

        # Stage 2 — Amplification onset
        self.amplification_onset_fraction = 0.0
        self.bending_amp_rate = 0.0             # Nm/s additional drift rate once amplification starts
        self.shear_amp_rate = 0.0
        self.axial_oscillation_amplitude = 0.0  # amplitude of axial oscillation in stage 2
        self.oscillation_phase_offset = 0.0

        # Stage 3 — Overload bias
        self.overload_onset_fraction = 0.0
        self.overload_bending_target = 0.0      # final spurious bending reading
        self.overload_shear_target = 0.0

        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            self.bending_drift_target = random.uniform(4000, 8000)
            self.shear_drift_target = random.uniform(1500, 3500)
            self.axial_drift_target = random.uniform(1000, 2500)
            self.drift_duration = random.uniform(16.0, 24.0)
            self.amplification_onset_fraction = random.uniform(0.35, 0.50)
            self.bending_amp_rate = random.uniform(600, 1200)
            self.shear_amp_rate = random.uniform(200, 500)
            self.axial_oscillation_amplitude = random.uniform(3000, 7000)
            self.overload_onset_fraction = random.uniform(0.68, 0.82)
            self.overload_bending_target = random.uniform(58000, 72000)
            self.overload_shear_target = random.uniform(28000, 36000)

        elif current_phase == 'cruise':
            # In cruise, spurious overload is especially alarming since
            # healthy cruise bending is only ~12000 Nm
            self.bending_drift_target = random.uniform(6000, 12000)
            self.shear_drift_target = random.uniform(2000, 5000)
            self.axial_drift_target = random.uniform(1500, 3500)
            self.drift_duration = random.uniform(20.0, 30.0)
            self.amplification_onset_fraction = random.uniform(0.30, 0.45)
            self.bending_amp_rate = random.uniform(800, 1600)
            self.shear_amp_rate = random.uniform(300, 700)
            self.axial_oscillation_amplitude = random.uniform(4000, 9000)
            self.overload_onset_fraction = random.uniform(0.62, 0.78)
            self.overload_bending_target = random.uniform(62000, 78000)
            self.overload_shear_target = random.uniform(30000, 40000)

        elif current_phase == 'landing':
            self.bending_drift_target = random.uniform(5000, 10000)
            self.shear_drift_target = random.uniform(2000, 4500)
            self.axial_drift_target = random.uniform(1200, 3000)
            self.drift_duration = random.uniform(12.0, 18.0)
            self.amplification_onset_fraction = random.uniform(0.28, 0.42)
            self.bending_amp_rate = random.uniform(700, 1400)
            self.shear_amp_rate = random.uniform(250, 600)
            self.axial_oscillation_amplitude = random.uniform(5000, 11000)
            self.overload_onset_fraction = random.uniform(0.58, 0.74)
            self.overload_bending_target = random.uniform(60000, 75000)
            self.overload_shear_target = random.uniform(29000, 38000)

        else:
            self.bending_drift_target = random.uniform(4000, 8000)
            self.shear_drift_target = random.uniform(1500, 3500)
            self.axial_drift_target = random.uniform(1000, 2500)
            self.drift_duration = random.uniform(15.0, 22.0)
            self.amplification_onset_fraction = random.uniform(0.35, 0.50)
            self.bending_amp_rate = random.uniform(500, 1000)
            self.shear_amp_rate = random.uniform(200, 450)
            self.axial_oscillation_amplitude = random.uniform(3000, 7000)
            self.overload_onset_fraction = random.uniform(0.65, 0.80)
            self.overload_bending_target = random.uniform(55000, 70000)
            self.overload_shear_target = random.uniform(26000, 34000)

        self.oscillation_phase_offset = random.uniform(0, 2 * math.pi)
        self.fault_configured = True

        print(
            f"[SPAR_FAULT] Configured: bending_drift={self.bending_drift_target:.0f}Nm "
            f"over {self.drift_duration:.1f}s | "
            f"amplification@{self.amplification_onset_fraction*100:.0f}% | "
            f"overload@{self.overload_onset_fraction*100:.0f}%"
        )

    def trigger_fault(self):
        if self.fault_active:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            delay = random.uniform(1.5, 3.5)
        elif current_phase == 'cruise':
            delay = random.uniform(2.0, 5.0)
        elif current_phase == 'landing':
            delay = random.uniform(0.0, 2.0)
        else:
            delay = random.uniform(1.0, 3.0)

        self.fault_trigger_time = time.time() + delay
        self.fault_active = True
        self.fault_was_triggered = True

        print(
            f"[SPAR_FAULT] Strain gauge de-bonding fault scheduled in {delay:.1f}s "
            f"during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(f"[SPAR_FAULT] Phase transition: {self.last_known_phase} -> {current_phase}")

            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[SPAR_FAULT] Cruise ended — degraded spar sensor continues publishing")

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
            print("[SPAR_FAULT] ⚠️  Strain gauge de-bonding INITIATED!")

        elapsed = current_time - self.fault_start_time
        progress = min(elapsed / max(self.drift_duration, 0.001), 1.0)

        # Stage 1: drift grows with sigmoid curve — slow, then accelerates, then plateaus
        sigmoid = 1.0 / (1.0 + math.exp(-8.0 * (progress - 0.35)))
        self.bending_drift_now = self.bending_drift_target * sigmoid
        self.shear_drift_now   = self.shear_drift_target   * sigmoid * 0.85
        self.axial_drift_now   = self.axial_drift_target   * sigmoid * 0.60

        # Stage 2: amplification adds extra rate on top of drift
        if progress >= self.amplification_onset_fraction:
            amp_elapsed = elapsed - (self.amplification_onset_fraction * self.drift_duration)
            amp_elapsed = max(amp_elapsed, 0.0)
            self.bending_drift_now += self.bending_amp_rate * amp_elapsed
            self.shear_drift_now   += self.shear_amp_rate   * amp_elapsed

    def inject_fault(self, bending_moment, shear_force, axial_load, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return bending_moment, shear_force, axial_load

        self.update_fault_progression()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return bending_moment, shear_force, axial_load

        elapsed = time.time() - self.fault_start_time if self.fault_start_time else 0.0
        progress = min(elapsed / max(self.drift_duration, 0.001), 1.0)

        in_overload = progress >= self.overload_onset_fraction

        # --- Bending moment ---
        if in_overload:
            # Smoothly approach overload target
            overload_progress = (progress - self.overload_onset_fraction) / max(
                1.0 - self.overload_onset_fraction, 0.001
            )
            faulty_bending = bending_moment + self.bending_drift_now
            faulty_bending += (self.overload_bending_target - faulty_bending) * overload_progress * 0.4
            faulty_bending += random.gauss(0, 400)
        else:
            faulty_bending = bending_moment + self.bending_drift_now
            faulty_bending += random.gauss(0, 200)

        # --- Shear force ---
        if in_overload:
            overload_progress = (progress - self.overload_onset_fraction) / max(
                1.0 - self.overload_onset_fraction, 0.001
            )
            faulty_shear = shear_force + self.shear_drift_now
            faulty_shear += (self.overload_shear_target - faulty_shear) * overload_progress * 0.35
            faulty_shear += random.gauss(0, 180)
        else:
            faulty_shear = shear_force + self.shear_drift_now
            faulty_shear += random.gauss(0, 100)

        # --- Axial load ---
        # Axial oscillates in stages 2+, proportional to bridge imbalance
        if progress >= self.amplification_onset_fraction:
            osc_time = elapsed * 0.4 + self.oscillation_phase_offset
            oscillation = self.axial_oscillation_amplitude * math.sin(osc_time) * (
                (progress - self.amplification_onset_fraction) /
                max(1.0 - self.amplification_onset_fraction, 0.001)
            )
            faulty_axial = axial_load + self.axial_drift_now + oscillation
            faulty_axial += random.gauss(0, 300)
        else:
            faulty_axial = axial_load + self.axial_drift_now
            faulty_axial += random.gauss(0, 150)

        # Clamp to sensor physical limits
        faulty_bending = round(max(-80000, min(faulty_bending, 80000)), 2)
        faulty_shear   = round(max(-40000, min(faulty_shear,   40000)), 2)
        faulty_axial   = round(max(-60000, min(faulty_axial,   60000)), 2)

        return faulty_bending, faulty_shear, faulty_axial

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
            progress = min(elapsed / max(self.drift_duration, 0.001), 1.0)

        stage = 'inactive'
        if self.is_fault_active():
            if progress >= self.overload_onset_fraction:
                stage = 'overload'
            elif progress >= self.amplification_onset_fraction:
                stage = 'amplification'
            else:
                stage = 'drift'

        return {
            'fault_type': 'strain_gauge_debond_overload_bias',
            'active': self.is_fault_active(),
            'stage': stage,
            'progress_fraction': round(progress, 3),
            'bending_drift_now': round(self.bending_drift_now, 1),
            'shear_drift_now': round(self.shear_drift_now, 1),
            'axial_drift_now': round(self.axial_drift_now, 1),
            'bending_drift_target': self.bending_drift_target,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'sensor_name': 'spar',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# WingSparLoadFaultManager
# =============================================================================

class WingSparLoadFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'strain_gauge_debond_overload_bias': WingSparLoadFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'strain_gauge_debond_overload_bias'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[SPAR FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[SPAR FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, bending_moment, shear_force, axial_load, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(
                bending_moment, shear_force, axial_load, phase
            )
            return result
        else:
            return bending_moment, shear_force, axial_load

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
            'sensor_name': 'spar',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_wingsparload_with_fault_injection
# =============================================================================

def update_wingsparload_with_fault_injection(spar_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()

    healthy_result = spar_sensor.update_sensor()

    original_bending = healthy_result['bending_moment_Nm']
    original_shear   = healthy_result['shear_force_N']
    original_axial   = healthy_result['axial_load_N']

    fault_result = fault_manager.inject_fault(
        original_bending, original_shear, original_axial, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_bending, faulty_shear, faulty_axial = fault_result

        if fault_manager.is_fault_active():
            output_bending = faulty_bending
            output_shear   = faulty_shear
            output_axial   = faulty_axial

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[SPAR FAULT] Healthy → "
                    f"Bend:{original_bending:.0f}Nm "
                    f"Shear:{original_shear:.0f}N "
                    f"Axial:{original_axial:.0f}N"
                )
                print(
                    f"[SPAR FAULT] Faulty  → "
                    f"Bend:{faulty_bending:.0f}Nm "
                    f"Shear:{faulty_shear:.0f}N "
                    f"Axial:{faulty_axial:.0f}N | "
                    f"Stage:{fault_status['stage'].upper()} "
                    f"Drift:{fault_status['bending_drift_now']:.0f}Nm"
                )
        else:
            output_bending = original_bending
            output_shear   = original_shear
            output_axial   = original_axial
    else:
        output_bending = original_bending
        output_shear   = original_shear
        output_axial   = original_axial

    bus.publish("wing/spar", {
        "sensor": "spar",
        "phase": current_phase,
        "bending_moment": output_bending,
        "shear_force": output_shear,
        "axial_load": output_axial,
        "timestamp": time.time()
    })

    return {
        "sensor": "spar",
        "phase": current_phase,
        "bending_moment_Nm": output_bending,
        "shear_force_N": output_shear,
        "axial_load_N": output_axial,
    }