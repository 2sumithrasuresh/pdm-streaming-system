import time
import random
import math

# =============================================================================
# FAULT TYPE: Sensor Coupling Breakdown + Deflection Channel Saturation
#
# FAULT DESCRIPTION:
#   The WingtipNodeSensor is a composite sensor that fuses three physically
#   coupled measurements: acceleration, deflection, and strain. In a healthy
#   sensor, these three channels are correlated — higher acceleration drives
#   higher deflection, which in turn drives higher strain. The sensor's
#   internal coupling model (accel_strain_correlation, strain_acceleration_factor)
#   enforces this relationship.
#
#   Two linked failure modes are modelled here:
#
#   (A) Coupling Breakdown — the deflection transducer's reference capacitor
#       begins to drift (a common failure in capacitive MEMS displacement
#       sensors). The deflection reading drifts upward independently of the
#       actual mechanical deflection, breaking the physical correlation between
#       the three channels. Acceleration and strain continue reporting realistic
#       values while deflection trends high — creating an internally inconsistent
#       sensor output that is characteristic of a transducer reference fault.
#
#   (B) Deflection Channel Saturation — as the reference drift accelerates,
#       the deflection reading approaches and eventually pins near the sensor's
#       upper range limit (90 cm). Strain then also reads high because the
#       healthy sensor uses deflection as a primary input to its strain
#       calculation, creating a secondary fault on the strain channel that is
#       entirely caused by the primary deflection fault. Acceleration continues
#       reading correctly, which makes the cross-channel inconsistency
#       (high deflection/strain, normal acceleration) the key diagnostic signal.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — Reference Drift (early, subtle):
#     Deflection reads consistently above what acceleration would predict.
#     The accel-deflection correlation (healthy: deflection ~ 12*accel + base)
#     breaks down — deflection is too high for the observed acceleration.
#     Strain is slightly elevated because the sensor computes it partly from
#     deflection. The readings are individually plausible but inconsistent.
#
#   Stage 2 — Divergence (middle, worsening):
#     Deflection drifts clearly beyond phase-appropriate ranges. The
#     inconsistency becomes large — acceleration reads normal cruise values
#     (0.18-0.45g) while deflection reads landing-level values (70-80cm).
#     Strain follows deflection upward. The rate-of-change limiter in the
#     healthy sensor is bypassed by the fault, so deflection can jump faster
#     than the 4.5cm/tick healthy limit.
#
#   Stage 3 — Saturation (late, hard fault):
#     Deflection pins near or at the sensor upper bound (90cm) with small
#     noise oscillations. Strain follows into its upper range (approaching
#     400 microstrain). Acceleration remains normal. The three channels are
#     now completely decoupled from each other and from physical reality.
#     The sensor continues publishing these saturated values.
#
# TOTAL FAILURE INDICATOR:
#   - Deflection >= 87 cm while acceleration < 0.5g (physically impossible)
#   - Strain > 370 microstrain during cruise (healthy cruise max ~325)
#   - Accel-deflection correlation coefficient drops below 0.2
#   - Deflection changing faster than 4.5cm/tick (healthy rate limiter bypassed)
#
# HEALTHY RANGES (for reference):
#   Takeoff  : accel=0.6-2.5g, deflection=15-55cm, strain=0-400 microstrain
#   Cruise   : accel=0.18-0.85g, deflection=38-90cm, strain=125-325 microstrain
#   Landing  : accel=0.6-2.0g, deflection=15-78cm, strain=0-380 microstrain
# =============================================================================


class WingtipNodeFaultInjector:
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

        # Stage 1 — Reference drift
        self.deflection_drift_rate = 0.0        # cm/s of upward drift
        self.deflection_drift_now = 0.0         # accumulated drift (cm)
        self.deflection_drift_target = 0.0      # total drift at plateau
        self.drift_duration = 0.0               # seconds to reach target

        # Stage 2 — Divergence
        self.divergence_onset_fraction = 0.0    # progress fraction for stage 2
        self.bypass_rate_limiter = False        # whether to exceed healthy rate limit
        self.extra_drift_rate = 0.0             # additional cm/s added in stage 2

        # Stage 3 — Saturation
        self.saturation_onset_fraction = 0.0
        self.saturation_target = 0.0            # deflection value to rail at (cm)
        self.saturation_noise_std = 0.0         # small noise on saturated reading

        # Strain secondary effect
        self.strain_amplification = 1.0         # multiplier applied to strain in stage 2+

        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            self.deflection_drift_target = random.uniform(20.0, 35.0)
            self.drift_duration = random.uniform(15.0, 22.0)
            self.divergence_onset_fraction = random.uniform(0.35, 0.50)
            self.saturation_onset_fraction = random.uniform(0.65, 0.80)
            self.extra_drift_rate = random.uniform(1.2, 2.5)
            self.saturation_target = random.uniform(84.0, 89.5)
            self.saturation_noise_std = random.uniform(0.3, 0.8)
            self.strain_amplification = random.uniform(1.15, 1.35)

        elif current_phase == 'cruise':
            # Most alarming in cruise — deflection should be 38-85cm,
            # saturation at 90cm during stable cruise is a clear fault signal
            self.deflection_drift_target = random.uniform(15.0, 28.0)
            self.drift_duration = random.uniform(18.0, 28.0)
            self.divergence_onset_fraction = random.uniform(0.30, 0.45)
            self.saturation_onset_fraction = random.uniform(0.62, 0.76)
            self.extra_drift_rate = random.uniform(1.0, 2.0)
            self.saturation_target = random.uniform(86.0, 89.8)
            self.saturation_noise_std = random.uniform(0.2, 0.6)
            self.strain_amplification = random.uniform(1.20, 1.45)

        elif current_phase == 'landing':
            self.deflection_drift_target = random.uniform(18.0, 32.0)
            self.drift_duration = random.uniform(10.0, 16.0)
            self.divergence_onset_fraction = random.uniform(0.28, 0.42)
            self.saturation_onset_fraction = random.uniform(0.58, 0.72)
            self.extra_drift_rate = random.uniform(1.5, 3.0)
            self.saturation_target = random.uniform(85.0, 89.9)
            self.saturation_noise_std = random.uniform(0.4, 1.0)
            self.strain_amplification = random.uniform(1.10, 1.30)

        else:
            self.deflection_drift_target = random.uniform(15.0, 25.0)
            self.drift_duration = random.uniform(14.0, 20.0)
            self.divergence_onset_fraction = random.uniform(0.35, 0.50)
            self.saturation_onset_fraction = random.uniform(0.65, 0.78)
            self.extra_drift_rate = random.uniform(1.0, 2.0)
            self.saturation_target = random.uniform(84.0, 89.5)
            self.saturation_noise_std = random.uniform(0.3, 0.7)
            self.strain_amplification = random.uniform(1.12, 1.30)

        self.deflection_drift_rate = self.deflection_drift_target / self.drift_duration
        self.fault_configured = True

        print(
            f"[WINGTIP_NODE_FAULT] Configured: deflection_drift={self.deflection_drift_target:.1f}cm "
            f"over {self.drift_duration:.1f}s | "
            f"divergence@{self.divergence_onset_fraction*100:.0f}% | "
            f"saturation@{self.saturation_onset_fraction*100:.0f}% "
            f"(target={self.saturation_target:.1f}cm)"
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
            f"[WINGTIP_NODE_FAULT] Deflection reference drift fault scheduled "
            f"in {delay:.1f}s during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(
                f"[WINGTIP_NODE_FAULT] Phase transition: "
                f"{self.last_known_phase} -> {current_phase}"
            )
            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[WINGTIP_NODE_FAULT] Cruise ended — degraded sensor continues publishing")

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
            print("[WINGTIP_NODE_FAULT] ⚠️  Deflection reference capacitor drift INITIATED!")

        elapsed = current_time - self.fault_start_time
        progress = min(elapsed / max(self.drift_duration, 0.001), 1.0)

        # Stage 1: sigmoid drift curve — slow, then accelerates
        sigmoid = 1.0 / (1.0 + math.exp(-7.0 * (progress - 0.3)))
        self.deflection_drift_now = self.deflection_drift_target * sigmoid

        # Stage 2: extra drift rate added on top
        if progress >= self.divergence_onset_fraction:
            self.bypass_rate_limiter = True
            extra_elapsed = elapsed - (self.divergence_onset_fraction * self.drift_duration)
            self.deflection_drift_now += self.extra_drift_rate * max(extra_elapsed, 0.0)

        # Clamp deflection drift to prevent going beyond saturation target
        self.deflection_drift_now = min(
            self.deflection_drift_now,
            self.saturation_target  # ceiling
        )

    def inject_fault(self, acceleration, deflection, strain, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return acceleration, deflection, strain

        self.update_fault_progression()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return acceleration, deflection, strain

        elapsed = time.time() - self.fault_start_time if self.fault_start_time else 0.0
        progress = min(elapsed / max(self.drift_duration, 0.001), 1.0)

        in_saturation = progress >= self.saturation_onset_fraction

        # --- Acceleration: unaffected (this IS the diagnostic signal) ---
        faulty_accel = acceleration

        # --- Deflection ---
        if in_saturation:
            # Rail near saturation target with small noise
            faulty_deflection = self.saturation_target + random.gauss(0, self.saturation_noise_std)
            faulty_deflection = round(max(0.0, min(faulty_deflection, 90.0)), 2)
        else:
            faulty_deflection = deflection + self.deflection_drift_now
            faulty_deflection += random.gauss(0, 0.15)
            faulty_deflection = round(max(0.0, min(faulty_deflection, 90.0)), 2)

        # --- Strain: secondary effect driven by inflated deflection ---
        if progress >= self.divergence_onset_fraction:
            # Strain is computed from faulty deflection in the secondary effect
            # We amplify what the healthy sensor would compute from the drifted deflection
            if phase == 'cruise':
                secondary_strain = 125 + (faulty_deflection * 2.4) + (acceleration * 200)
            elif phase == 'landing':
                secondary_strain = (faulty_deflection * 4.2) + (acceleration * 35)
            else:
                secondary_strain = (faulty_deflection * 3.5) + (faulty_deflection ** 1.2 * 0.5)

            secondary_strain *= self.strain_amplification

            # Blend original strain toward secondary with progress
            blend = min(
                (progress - self.divergence_onset_fraction) /
                max(1.0 - self.divergence_onset_fraction, 0.001),
                1.0
            )
            faulty_strain = strain * (1.0 - blend) + secondary_strain * blend
            faulty_strain += random.gauss(0, 2.0)
        else:
            # Stage 1: strain slightly elevated due to drift
            faulty_strain = strain + self.deflection_drift_now * 1.5
            faulty_strain += random.gauss(0, 1.0)

        # Clamp to sensor physical limits
        phase_strain_max = {'cruise': 325.0, 'landing': 380.0}.get(phase, 400.0)
        faulty_strain = round(max(0.0, min(faulty_strain, phase_strain_max * 1.15)), 2)

        return faulty_accel, faulty_deflection, faulty_strain

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
            if progress >= self.saturation_onset_fraction:
                stage = 'saturation'
            elif progress >= self.divergence_onset_fraction:
                stage = 'divergence'
            else:
                stage = 'drift'

        return {
            'fault_type': 'deflection_reference_drift_saturation',
            'active': self.is_fault_active(),
            'stage': stage,
            'progress_fraction': round(progress, 3),
            'deflection_drift_now': round(self.deflection_drift_now, 2),
            'deflection_drift_target': self.deflection_drift_target,
            'saturation_target': self.saturation_target,
            'strain_amplification': self.strain_amplification,
            'bypass_rate_limiter': self.bypass_rate_limiter,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'sensor_name': 'wingtip_node',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# WingtipNodeFaultManager
# =============================================================================

class WingtipNodeFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'deflection_reference_drift_saturation': WingtipNodeFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'deflection_reference_drift_saturation'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[WINGTIP NODE FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[WINGTIP NODE FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, acceleration, deflection, strain, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(acceleration, deflection, strain, phase)
            return result
        else:
            return acceleration, deflection, strain

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
            'sensor_name': 'wingtip_node',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_wingtip_node_with_fault_injection
# =============================================================================

def update_wingtip_node_with_fault_injection(wingtip_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()

    healthy_result = wingtip_sensor.update_wingtip_node()

    original_accel      = healthy_result['acceleration_g']
    original_deflection = healthy_result['deflection_cm']
    original_strain     = healthy_result['strain_microstrain']

    fault_result = fault_manager.inject_fault(
        original_accel, original_deflection, original_strain, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_accel, faulty_deflection, faulty_strain = fault_result

        if fault_manager.is_fault_active():
            output_accel      = faulty_accel
            output_deflection = faulty_deflection
            output_strain     = faulty_strain

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[WINGTIP NODE FAULT] Healthy → "
                    f"Accel:{original_accel:.2f}g "
                    f"Defl:{original_deflection:.2f}cm "
                    f"Strain:{original_strain:.2f}me"
                )
                print(
                    f"[WINGTIP NODE FAULT] Faulty  → "
                    f"Accel:{faulty_accel:.2f}g "
                    f"Defl:{faulty_deflection:.2f}cm "
                    f"Strain:{faulty_strain:.2f}me | "
                    f"Stage:{fault_status['stage'].upper()} "
                    f"Drift:{fault_status['deflection_drift_now']:.1f}cm"
                )
        else:
            output_accel      = original_accel
            output_deflection = original_deflection
            output_strain     = original_strain
    else:
        output_accel      = original_accel
        output_deflection = original_deflection
        output_strain     = original_strain

    bus.publish("wing/wingtip_node", {
        "sensor": "wingtip_node",
        "phase": current_phase,
        "acceleration (g)": output_accel,
        "deflection (cm)": output_deflection,
        "strain (me)": output_strain,
        "timestamp": time.time()
    })

    return {
        "sensor": "wingtip_node",
        "phase": current_phase,
        "acceleration_g": output_accel,
        "deflection_cm": output_deflection,
        "strain_microstrain": output_strain,
    }