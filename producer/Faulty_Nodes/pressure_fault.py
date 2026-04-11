import time
import random
import math

# =============================================================================
# FAULT TYPE: Aerodynamic Asymmetry Drift + Pressure Transducer Saturation
#
# FAULT DESCRIPTION:
#   Wing surface pressure sensors measure the differential pressure across the
#   wing to infer aerodynamic load distribution. Two realistic failure modes
#   are combined here:
#
#   (A) Asymmetry Drift — a partial blockage of one of the pressure taps
#       (e.g. insect ingestion, micro-crack in the static port manifold)
#       causes the reported pressure on one side of the wing to read
#       artificially higher, making the asymmetry index trend upward over
#       time even in stable, symmetric flight.
#
#   (B) Pressure Transducer Saturation — an internal reference voltage
#       instability in the transducer causes the dynamic pressure reading to
#       drift upward and eventually rail (saturate) at the sensor upper
#       limit, while the gradient reading becomes erratic and uncorrelated
#       with actual aerodynamic conditions.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — Tap Blockage (early, subtle):
#     Asymmetry index starts a slow upward drift above its healthy range.
#     Dynamic pressure reads slightly high (+3-8% above nominal) due to the
#     partially blocked tap acting as a partial Pitot restriction. Gradient
#     remains plausible but slightly elevated. No obvious alarm — the values
#     are still in range but trending consistently in one direction.
#
#   Stage 2 — Instability Onset (middle, worsening):
#     Asymmetry index now clearly exceeds healthy limits for the phase and
#     shows erratic oscillations (the blockage is intermittently clearing
#     and re-blocking). Dynamic pressure shows periodic spikes +10-20%
#     above nominal. Pressure gradient becomes noisy — high variance, no
#     longer follows the smooth sinusoidal pattern of healthy operation.
#     The validate_readings() warnings fire consistently.
#
#   Stage 3 — Transducer Saturation (late, hard fault):
#     Dynamic pressure rails toward the upper bound of the sensor range
#     (1150 Pa in cruise, 1200 Pa in landing, 1100 Pa in takeoff). The
#     gradient reading becomes nonsensical — large positive then large
#     negative swings. Asymmetry index is pinned at or above the phase
#     maximum. The sensor continues publishing these saturated/nonsensical
#     values for the rest of the flight.
#
# TOTAL FAILURE INDICATOR:
#   - Dynamic pressure pinned at or very near the sensor upper bound
#     for multiple consecutive ticks
#   - Asymmetry index >= phase_maximum for sustained period
#   - Gradient swings from large positive to large negative within 2 ticks
#
# HEALTHY RANGES (for reference):
#   Takeoff  : pressure=160-1100 Pa, gradient=1.4-2.6 Pa/cm, asymmetry=0-0.20
#   Cruise   : pressure=1050-1150 Pa, gradient=3.3-3.9 Pa/cm, asymmetry=0-0.08
#   Landing  : pressure=970-1200 Pa, gradient=3.9-4.2 Pa/cm, asymmetry=0-0.12
# =============================================================================


class PressureFaultInjector:
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

        # Stage 1 — Tap blockage / asymmetry drift
        self.asymmetry_drift_rate = 0.0
        self.asymmetry_current_bias = 0.0
        self.asymmetry_drift_target = 0.0
        self.drift_duration = 0.0

        # Stage 2 — Instability / oscillation
        self.instability_onset_fraction = 0.0
        self.pressure_spike_magnitude = 0.0
        self.gradient_noise_scale = 0.0
        self.oscillation_period = 0.0

        # Stage 3 — Saturation
        self.saturation_onset_fraction = 0.0
        self.saturation_rate = 0.0
        self.pressure_saturation_bias = 0.0
        self.gradient_sign_flip_period = 0.0

        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            self.asymmetry_drift_target = random.uniform(0.22, 0.35)
            self.drift_duration = random.uniform(20.0, 30.0)
            self.instability_onset_fraction = random.uniform(0.35, 0.50)
            self.saturation_onset_fraction = random.uniform(0.65, 0.80)
            self.pressure_spike_magnitude = random.uniform(60.0, 120.0)
            self.gradient_noise_scale = random.uniform(0.4, 0.8)
            self.oscillation_period = random.uniform(3.0, 6.0)
            self.saturation_rate = random.uniform(12.0, 22.0)
            self.gradient_sign_flip_period = random.uniform(1.5, 3.0)

        elif current_phase == 'cruise':
            self.asymmetry_drift_target = random.uniform(0.10, 0.18)
            self.drift_duration = random.uniform(22.0, 32.0)
            self.instability_onset_fraction = random.uniform(0.30, 0.45)
            self.saturation_onset_fraction = random.uniform(0.60, 0.75)
            self.pressure_spike_magnitude = random.uniform(30.0, 70.0)
            self.gradient_noise_scale = random.uniform(0.3, 0.6)
            self.oscillation_period = random.uniform(4.0, 8.0)
            self.saturation_rate = random.uniform(8.0, 16.0)
            self.gradient_sign_flip_period = random.uniform(2.0, 4.0)

        elif current_phase == 'landing':
            self.asymmetry_drift_target = random.uniform(0.15, 0.28)
            self.drift_duration = random.uniform(12.0, 20.0)
            self.instability_onset_fraction = random.uniform(0.28, 0.42)
            self.saturation_onset_fraction = random.uniform(0.58, 0.72)
            self.pressure_spike_magnitude = random.uniform(50.0, 100.0)
            self.gradient_noise_scale = random.uniform(0.5, 0.9)
            self.oscillation_period = random.uniform(2.5, 5.0)
            self.saturation_rate = random.uniform(15.0, 25.0)
            self.gradient_sign_flip_period = random.uniform(1.2, 2.5)

        else:
            self.asymmetry_drift_target = random.uniform(0.12, 0.22)
            self.drift_duration = random.uniform(18.0, 28.0)
            self.instability_onset_fraction = random.uniform(0.35, 0.50)
            self.saturation_onset_fraction = random.uniform(0.65, 0.78)
            self.pressure_spike_magnitude = random.uniform(40.0, 90.0)
            self.gradient_noise_scale = random.uniform(0.35, 0.65)
            self.oscillation_period = random.uniform(3.5, 7.0)
            self.saturation_rate = random.uniform(10.0, 20.0)
            self.gradient_sign_flip_period = random.uniform(1.8, 3.5)

        self.asymmetry_drift_rate = self.asymmetry_drift_target / self.drift_duration
        self.fault_configured = True

        print(
            f"[PRESSURE_FAULT] Configured: asymmetry_target={self.asymmetry_drift_target:.3f} "
            f"over {self.drift_duration:.1f}s | "
            f"instability@{self.instability_onset_fraction*100:.0f}% | "
            f"saturation@{self.saturation_onset_fraction*100:.0f}%"
        )

    def trigger_fault(self):
        if self.fault_active:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            delay = random.uniform(1.0, 3.0)
        elif current_phase == 'cruise':
            delay = random.uniform(2.0, 6.0)
        elif current_phase == 'landing':
            delay = random.uniform(0.0, 1.5)
        else:
            delay = random.uniform(1.0, 3.0)

        self.fault_trigger_time = time.time() + delay
        self.fault_active = True
        self.fault_was_triggered = True

        print(
            f"[PRESSURE_FAULT] Tap blockage fault scheduled in {delay:.1f}s "
            f"during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(f"[PRESSURE_FAULT] Phase transition: {self.last_known_phase} -> {current_phase}")

            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[PRESSURE_FAULT] Cruise phase ended — degraded sensor continues publishing")

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
            print("[PRESSURE_FAULT] ⚠️  Tap blockage / asymmetry drift INITIATED!")

        elapsed = current_time - self.fault_start_time
        progress = min(elapsed / self.drift_duration, 1.0)

        # Sigmoid asymmetry rise: slow start, acceleration, plateau
        sigmoid_progress = 1.0 / (1.0 + math.exp(-8.0 * (progress - 0.4)))
        self.asymmetry_current_bias = self.asymmetry_drift_target * sigmoid_progress

        # Saturation bias only grows after saturation_onset_fraction
        if progress >= self.saturation_onset_fraction:
            sat_progress = (progress - self.saturation_onset_fraction) / max(
                1.0 - self.saturation_onset_fraction, 0.001
            )
            self.pressure_saturation_bias = self.saturation_rate * sat_progress * elapsed
        else:
            self.pressure_saturation_bias = 0.0

    def inject_fault(self, dynamic_pressure, pressure_gradient, asymmetry_index, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return dynamic_pressure, pressure_gradient, asymmetry_index

        self.update_fault_progression()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return dynamic_pressure, pressure_gradient, asymmetry_index

        elapsed = time.time() - self.fault_start_time if self.fault_start_time else 0.0
        progress = min(elapsed / max(self.drift_duration, 0.001), 1.0)

        in_instability = progress >= self.instability_onset_fraction
        in_saturation  = progress >= self.saturation_onset_fraction

        # --- Asymmetry index ---
        faulty_asymmetry = asymmetry_index + self.asymmetry_current_bias

        if in_instability:
            osc = 0.03 * math.sin(
                elapsed * (2 * math.pi / max(self.oscillation_period, 0.001))
            ) * random.uniform(0.8, 1.2)
            faulty_asymmetry += osc

        if in_saturation:
            phase_max_asymmetry = {'takeoff': 0.20, 'cruise': 0.08, 'landing': 0.12}.get(phase, 0.15)
            faulty_asymmetry = max(
                faulty_asymmetry,
                phase_max_asymmetry + random.uniform(0.005, 0.025)
            )

        faulty_asymmetry = round(max(0.0, min(faulty_asymmetry, 0.50)), 3)

        # --- Dynamic pressure ---
        phase_upper = {'takeoff': 1100.0, 'cruise': 1150.0, 'landing': 1200.0}.get(phase, 1150.0)
        phase_lower = {'takeoff': 160.0,  'cruise': 1050.0, 'landing': 970.0 }.get(phase, 1050.0)

        # Stage 1: modest upward bias
        early_bias = (phase_upper - dynamic_pressure) * 0.06 * progress
        faulty_pressure = dynamic_pressure + early_bias

        if in_instability and not in_saturation:
            if random.random() < 0.30:
                faulty_pressure += random.uniform(
                    self.pressure_spike_magnitude * 0.4,
                    self.pressure_spike_magnitude
                )

        if in_saturation:
            faulty_pressure += self.pressure_saturation_bias
            faulty_pressure = min(faulty_pressure, phase_upper + random.gauss(0, 5.0))

        faulty_pressure = round(max(phase_lower * 0.95, min(faulty_pressure, phase_upper * 1.02)), 2)

        # --- Pressure gradient ---
        faulty_gradient = pressure_gradient

        if in_instability and not in_saturation:
            noise = random.gauss(0, self.gradient_noise_scale)
            faulty_gradient += noise

        if in_saturation:
            flip_index = int(elapsed / max(self.gradient_sign_flip_period, 0.001))
            sign = 1.0 if flip_index % 2 == 0 else -1.0
            magnitude = random.uniform(1.5, 3.5) * self.gradient_noise_scale
            faulty_gradient = sign * (abs(pressure_gradient) + magnitude)
            faulty_gradient += random.gauss(0, self.gradient_noise_scale * 0.5)

        faulty_gradient = round(max(-6.0, min(faulty_gradient, 8.0)), 2)

        return faulty_pressure, faulty_gradient, faulty_asymmetry

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

        in_instability = progress >= self.instability_onset_fraction
        in_saturation  = progress >= self.saturation_onset_fraction

        stage = 'inactive'
        if self.is_fault_active():
            if in_saturation:
                stage = 'saturation'
            elif in_instability:
                stage = 'instability'
            else:
                stage = 'bleed'

        return {
            'fault_type': 'tap_blockage_transducer_saturation',
            'active': self.is_fault_active(),
            'stage': stage,
            'progress_fraction': round(progress, 3),
            'asymmetry_bias': round(self.asymmetry_current_bias, 4),
            'asymmetry_drift_target': self.asymmetry_drift_target,
            'pressure_saturation_bias': round(self.pressure_saturation_bias, 2),
            'in_instability': in_instability,
            'in_saturation': in_saturation,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'drift_duration': self.drift_duration,
            'sensor_name': 'surface_pressure',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# PressureFaultManager
# =============================================================================

class PressureFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'tap_blockage_transducer_saturation': PressureFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'tap_blockage_transducer_saturation'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[PRESSURE FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[PRESSURE FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, dynamic_pressure, pressure_gradient, asymmetry_index, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(
                dynamic_pressure, pressure_gradient, asymmetry_index, phase
            )
            return result
        else:
            return dynamic_pressure, pressure_gradient, asymmetry_index

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
            'sensor_name': 'surface_pressure',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_pressure_with_fault_injection
# =============================================================================

def update_pressure_with_fault_injection(pressure_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()

    healthy_result = pressure_sensor.update_wing_surface_pressure()

    original_pressure  = healthy_result['dynamic_pressure_Pa']
    original_gradient  = healthy_result['pressure_gradient_Pa_per_cm']
    original_asymmetry = healthy_result['asymmetry_index_percent']

    fault_result = fault_manager.inject_fault(
        original_pressure, original_gradient, original_asymmetry, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_pressure, faulty_gradient, faulty_asymmetry = fault_result

        if fault_manager.is_fault_active():
            output_pressure  = faulty_pressure
            output_gradient  = faulty_gradient
            output_asymmetry = faulty_asymmetry

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[PRESSURE FAULT] Healthy → "
                    f"P:{original_pressure:.1f}Pa "
                    f"G:{original_gradient:.2f}Pa/cm "
                    f"A:{original_asymmetry:.3f}"
                )
                print(
                    f"[PRESSURE FAULT] Faulty  → "
                    f"P:{faulty_pressure:.1f}Pa "
                    f"G:{faulty_gradient:.2f}Pa/cm "
                    f"A:{faulty_asymmetry:.3f} | "
                    f"Stage:{fault_status['stage'].upper()}"
                )
        else:
            output_pressure  = original_pressure
            output_gradient  = original_gradient
            output_asymmetry = original_asymmetry
    else:
        output_pressure  = original_pressure
        output_gradient  = original_gradient
        output_asymmetry = original_asymmetry

    bus.publish("wing/surface_pressure", {
        "sensor": "surface_pressure",
        "phase": current_phase,
        "dynamic_pressure (Pa)": output_pressure,
        "pressure_gradient (Pa/cm)": output_gradient,
        "asymmetry_index (%)": output_asymmetry,
        "timestamp": time.time()
    })

    return {
        "sensor": "surface_pressure",
        "phase": current_phase,
        "dynamic_pressure (Pa)": output_pressure,
        "pressure_gradient (Pa/cm)": output_gradient,
        "asymmetry_index (%)": output_asymmetry,
    }