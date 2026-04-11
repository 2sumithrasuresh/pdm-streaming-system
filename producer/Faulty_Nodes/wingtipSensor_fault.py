import time
import random
import math

# =============================================================================
# FAULT TYPE: SNR Degradation + Strain Channel Noise Floor Collapse
#
# FAULT DESCRIPTION:
#   The WingtipStrainSensor uses a fibre Bragg grating (FBG) interrogator
#   to measure strain via optical wavelength shift, with temperature as a
#   thermal compensation channel and SNR as the signal quality metric.
#   Two linked failure modes are modelled here:
#
#   (A) SNR Degradation — the optical connector at the wingtip end of the
#       sensing fibre develops a micro-crack (due to wing flex fatigue).
#       The crack causes increasing optical insertion loss. The SNR reported
#       by the interrogator drops steadily as the crack widens. This is the
#       primary fault and the earliest visible signal — SNR falls below the
#       healthy range before either the strain or temperature channels are
#       affected. A healthy FBG interrogator requires SNR > 20 dB to produce
#       accurate wavelength shift measurements.
#
#   (B) Noise Floor Collapse — once SNR drops below ~22 dB the interrogator
#       can no longer distinguish the Bragg peak from the noise floor. The
#       strain reading degrades in two ways: first, the noise amplitude
#       increases dramatically (the reading becomes very jittery), then the
#       reading begins to drift toward a false value (the interrogator locks
#       onto a noise peak instead of the Bragg peak). Temperature is less
#       affected because it is measured via a separate reference channel,
#       but it too shows elevated noise as the SNR deteriorates.
#
# DEGRADATION PATTERN (3 stages):
#
#   Stage 1 — SNR Drop (early, subtle):
#     SNR falls steadily from its healthy range (22-35 dB) toward the
#     critical threshold (~20 dB). Strain and temperature remain normal.
#     This is a "pre-fault warning" stage — the measurement quality is
#     degrading but readings are still accurate.
#
#   Stage 2 — Noise Floor Onset (middle, noisy but plausible):
#     SNR crosses the ~22 dB threshold. Strain noise amplitude increases
#     progressively — standard deviation grows from the healthy 5-8 to
#     20-40 microstrain. Readings are now unreliable but still centred
#     roughly around the true value. Temperature shows moderately elevated
#     noise. The strain readings are highly variable tick-to-tick.
#
#   Stage 3 — Peak Lock Failure (late, false readings):
#     SNR drops below 19 dB. The interrogator locks onto a noise peak,
#     causing the strain reading to drift to a false plateau value that
#     is unrelated to actual structural strain. The false value is
#     typically near 0 microstrain or a large positive value, depending
#     on which noise peak the interrogator locked onto. Noise amplitude
#     remains large. Temperature drifts slightly due to reference channel
#     contamination. SNR flatlines near its lowest value. The sensor
#     continues publishing throughout.
#
# TOTAL FAILURE INDICATOR:
#   - SNR < 19 dB for more than 3 consecutive ticks
#   - Strain standard deviation (rolling 5-tick) > 35 microstrain
#   - Strain reading at a physically implausible value for phase
#     (e.g. 0 microstrain during landing, or 480 during cruise)
#   - Strain no longer correlating with temperature changes
#
# HEALTHY RANGES (for reference):
#   Takeoff  : strain=350-450 me, temp=15-35C, snr=22-28 dB
#   Cruise   : strain=80-120 me, temp=-30 to -10C, snr=28-35 dB
#   Landing  : strain=310-390 me, temp=-10 to 25C, snr=20-27 dB
# =============================================================================


class WingtipStrainFaultInjector:
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

        # Stage 1 — SNR drop
        self.snr_drop_target = 0.0              # total dB drop from healthy baseline
        self.snr_drop_rate = 0.0               # dB/s drop rate
        self.snr_drop_now = 0.0                # accumulated dB dropped so far
        self.snr_floor = 0.0                   # minimum SNR the sensor reaches
        self.snr_drop_duration = 0.0           # seconds to reach floor

        # Stage 2 — Noise floor onset
        self.noise_onset_fraction = 0.0        # progress fraction for stage 2
        self.strain_noise_scale = 0.0          # growing noise std for strain (microstrain)
        self.temp_noise_scale = 0.0            # growing noise std for temperature (C)
        self.max_strain_noise = 0.0            # peak strain noise std

        # Stage 3 — Peak lock failure
        self.lock_onset_fraction = 0.0
        self.lock_active = False
        self.locked_strain_value = None        # false strain value interrogator locked onto
        self.temp_drift_rate = 0.0             # C/s temperature reference contamination

        self.fault_configured = False

    def configure_fault(self):
        if self.fault_configured:
            return

        current_phase = self.phase_controller.get_current_phase()

        if current_phase == 'takeoff':
            # Takeoff SNR healthy range: 22-28 dB
            self.snr_drop_target = random.uniform(7.0, 12.0)
            self.snr_floor = random.uniform(14.0, 18.5)
            self.snr_drop_duration = random.uniform(15.0, 22.0)
            self.noise_onset_fraction = random.uniform(0.35, 0.50)
            self.lock_onset_fraction = random.uniform(0.68, 0.82)
            self.max_strain_noise = random.uniform(28.0, 45.0)
            self.temp_noise_scale = random.uniform(0.6, 1.4)
            self.temp_drift_rate = random.uniform(0.05, 0.15)
            self.locked_strain_value = random.choice([
                random.uniform(-10.0, 15.0),        # near-zero false lock
                random.uniform(480.0, 520.0)        # high false lock
            ])

        elif current_phase == 'cruise':
            # Cruise SNR healthy range: 28-35 dB — most headroom to fall
            self.snr_drop_target = random.uniform(12.0, 18.0)
            self.snr_floor = random.uniform(13.0, 17.5)
            self.snr_drop_duration = random.uniform(20.0, 30.0)
            self.noise_onset_fraction = random.uniform(0.30, 0.45)
            self.lock_onset_fraction = random.uniform(0.62, 0.78)
            self.max_strain_noise = random.uniform(32.0, 55.0)
            self.temp_noise_scale = random.uniform(0.8, 1.8)
            self.temp_drift_rate = random.uniform(0.04, 0.12)
            self.locked_strain_value = random.choice([
                random.uniform(-8.0, 12.0),
                random.uniform(450.0, 500.0)
            ])

        elif current_phase == 'landing':
            # Landing SNR healthy range: 20-27 dB — smallest margin
            self.snr_drop_target = random.uniform(5.0, 10.0)
            self.snr_floor = random.uniform(12.0, 17.0)
            self.snr_drop_duration = random.uniform(10.0, 16.0)
            self.noise_onset_fraction = random.uniform(0.28, 0.42)
            self.lock_onset_fraction = random.uniform(0.58, 0.72)
            self.max_strain_noise = random.uniform(25.0, 42.0)
            self.temp_noise_scale = random.uniform(0.5, 1.2)
            self.temp_drift_rate = random.uniform(0.06, 0.18)
            self.locked_strain_value = random.choice([
                random.uniform(-5.0, 10.0),
                random.uniform(460.0, 510.0)
            ])

        else:
            self.snr_drop_target = random.uniform(8.0, 14.0)
            self.snr_floor = random.uniform(13.5, 18.0)
            self.snr_drop_duration = random.uniform(14.0, 22.0)
            self.noise_onset_fraction = random.uniform(0.35, 0.50)
            self.lock_onset_fraction = random.uniform(0.65, 0.80)
            self.max_strain_noise = random.uniform(28.0, 48.0)
            self.temp_noise_scale = random.uniform(0.6, 1.4)
            self.temp_drift_rate = random.uniform(0.05, 0.14)
            self.locked_strain_value = random.choice([
                random.uniform(-8.0, 12.0),
                random.uniform(460.0, 510.0)
            ])

        self.snr_drop_rate = self.snr_drop_target / self.snr_drop_duration
        self.fault_configured = True

        print(
            f"[WINGTIP_STRAIN_FAULT] Configured: snr_drop={self.snr_drop_target:.1f}dB "
            f"to floor={self.snr_floor:.1f}dB over {self.snr_drop_duration:.1f}s | "
            f"noise@{self.noise_onset_fraction*100:.0f}% | "
            f"lock@{self.lock_onset_fraction*100:.0f}% "
            f"(false_strain={self.locked_strain_value:.1f}me)"
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
            f"[WINGTIP_STRAIN_FAULT] Fibre micro-crack fault scheduled "
            f"in {delay:.1f}s during {current_phase} phase"
        )

    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()

        if self.last_known_phase != current_phase:
            print(
                f"[WINGTIP_STRAIN_FAULT] Phase transition: "
                f"{self.last_known_phase} -> {current_phase}"
            )
            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print("[WINGTIP_STRAIN_FAULT] Cruise ended — degraded sensor continues publishing")

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
            print("[WINGTIP_STRAIN_FAULT] ⚠️  Fibre Bragg grating micro-crack INITIATED!")

        elapsed = current_time - self.fault_start_time
        progress = min(elapsed / max(self.snr_drop_duration, 0.001), 1.0)

        # SNR drops on a concave-up curve (fast early, then slows near floor)
        drop_curve = 1.0 - math.exp(-3.5 * progress)
        self.snr_drop_now = self.snr_drop_target * drop_curve

        # Noise floor grows in stage 2 — linear ramp from onset to lock
        if progress >= self.noise_onset_fraction:
            noise_progress = (progress - self.noise_onset_fraction) / max(
                self.lock_onset_fraction - self.noise_onset_fraction, 0.001
            )
            noise_progress = min(noise_progress, 1.0)
            self.strain_noise_scale = self.max_strain_noise * noise_progress

        # Stage 3 lock
        if progress >= self.lock_onset_fraction and not self.lock_active:
            self.lock_active = True
            print(
                f"[WINGTIP_STRAIN_FAULT] 🔴 PEAK LOCK FAILURE — "
                f"interrogator locked to noise peak at {self.locked_strain_value:.1f}me"
            )

    def inject_fault(self, strain, temperature, snr, phase):
        self.check_phase_transition()

        if not self.fault_active:
            return strain, temperature, snr

        self.update_fault_progression()

        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return strain, temperature, snr

        elapsed = time.time() - self.fault_start_time if self.fault_start_time else 0.0
        progress = min(elapsed / max(self.snr_drop_duration, 0.001), 1.0)

        # --- SNR: drops steadily, floors at snr_floor ---
        faulty_snr = snr - self.snr_drop_now
        faulty_snr += random.gauss(0, 0.15)   # small measurement noise
        faulty_snr = round(max(self.snr_floor, min(faulty_snr, 40.0)), 2)

        # --- Strain ---
        if self.lock_active:
            # Interrogator locked to wrong peak — reads false constant (+noise)
            blend_progress = min(
                (elapsed - self.lock_onset_fraction * self.snr_drop_duration) / 3.0,
                1.0
            )
            blend_progress = max(blend_progress, 0.0)
            # Smooth transition from last real value to false lock value
            faulty_strain = strain * (1.0 - blend_progress) + self.locked_strain_value * blend_progress
            faulty_strain += random.gauss(0, self.max_strain_noise * 0.6)
            # Clamp to physical sensor range
            faulty_strain = round(max(-500.0, min(faulty_strain, 520.0)), 2)

        elif self.strain_noise_scale > 0:
            # Stage 2: correct mean but very noisy
            faulty_strain = strain + random.gauss(0, self.strain_noise_scale)
            faulty_strain = round(max(-500.0, min(faulty_strain, 520.0)), 2)

        else:
            # Stage 1: strain still accurate
            faulty_strain = strain

        # --- Temperature: elevated noise in stages 2-3, slight drift in stage 3 ---
        if self.lock_active:
            temp_drift = self.temp_drift_rate * (elapsed - self.lock_onset_fraction * self.snr_drop_duration)
            temp_drift = max(temp_drift, 0.0)
            faulty_temp = temperature + temp_drift + random.gauss(0, self.temp_noise_scale)
        elif self.strain_noise_scale > 0:
            noise_fraction = self.strain_noise_scale / max(self.max_strain_noise, 0.001)
            faulty_temp = temperature + random.gauss(0, self.temp_noise_scale * noise_fraction)
        else:
            faulty_temp = temperature

        # Clamp temperature to physical sensor range
        faulty_temp = round(max(-50.0, min(faulty_temp, 60.0)), 2)

        return faulty_strain, faulty_temp, faulty_snr

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
            progress = min(elapsed / max(self.snr_drop_duration, 0.001), 1.0)

        stage = 'inactive'
        if self.is_fault_active():
            if self.lock_active:
                stage = 'peak_lock_failure'
            elif self.strain_noise_scale > 0:
                stage = 'noise_floor_onset'
            else:
                stage = 'snr_drop'

        return {
            'fault_type': 'fibre_crack_snr_noise_floor_collapse',
            'active': self.is_fault_active(),
            'stage': stage,
            'progress_fraction': round(progress, 3),
            'snr_drop_now': round(self.snr_drop_now, 2),
            'snr_drop_target': self.snr_drop_target,
            'snr_floor': self.snr_floor,
            'strain_noise_scale': round(self.strain_noise_scale, 2),
            'lock_active': self.lock_active,
            'locked_strain_value': self.locked_strain_value,
            'elapsed_time': elapsed,
            'flight_time': time.time() - self.flight_start_time,
            'sensor_name': 'wingtip_strain',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


# =============================================================================
# WingtipStrainFaultManager
# =============================================================================

class WingtipStrainFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'fibre_crack_snr_noise_floor_collapse': WingtipStrainFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False

    def trigger_random_fault(self):
        fault_type = 'fibre_crack_snr_noise_floor_collapse'

        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[WINGTIP STRAIN FAULT MANAGER] Fault activated: {fault_type}")
        else:
            print(f"[WINGTIP STRAIN FAULT MANAGER] Unknown fault type: {fault_type}")

    def inject_fault(self, strain, temperature, snr, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(strain, temperature, snr, phase)
            return result
        else:
            return strain, temperature, snr

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
            'sensor_name': 'wingtip_strain',
            'publishing_stopped': False,
            'cruise_phase_ended': False
        }


# =============================================================================
# update_wingtip_strain_with_fault_injection
# =============================================================================

def update_wingtip_strain_with_fault_injection(
    strain_sensor, fault_manager, bus, phase_controller
):
    current_phase = phase_controller.get_current_phase()

    healthy_result = strain_sensor.update_wingtip_strain()

    original_strain = healthy_result['strain_microstrain']
    original_temp   = healthy_result['temperature_C']
    original_snr    = healthy_result['snr_dB']

    fault_result = fault_manager.inject_fault(
        original_strain, original_temp, original_snr, current_phase
    )

    if fault_result and fault_result[0] is not None:
        faulty_strain, faulty_temp, faulty_snr = fault_result

        if fault_manager.is_fault_active():
            output_strain = faulty_strain
            output_temp   = faulty_temp
            output_snr    = faulty_snr

            fault_status = fault_manager.get_fault_status()
            if fault_status['active'] and fault_status['elapsed_time'] < 6.0:
                print(
                    f"[WINGTIP STRAIN FAULT] Healthy → "
                    f"Strain:{original_strain:.2f}me "
                    f"Temp:{original_temp:.2f}C "
                    f"SNR:{original_snr:.2f}dB"
                )
                print(
                    f"[WINGTIP STRAIN FAULT] Faulty  → "
                    f"Strain:{faulty_strain:.2f}me "
                    f"Temp:{faulty_temp:.2f}C "
                    f"SNR:{faulty_snr:.2f}dB | "
                    f"Stage:{fault_status['stage'].upper()} "
                    f"SNR_drop:{fault_status['snr_drop_now']:.1f}dB"
                )
        else:
            output_strain = original_strain
            output_temp   = original_temp
            output_snr    = original_snr
    else:
        output_strain = original_strain
        output_temp   = original_temp
        output_snr    = original_snr

    bus.publish("wing/wingtip_strain", {
        "sensor": "wingtip_strain",
        "phase": current_phase,
        "strain (microstrain)": output_strain,
        "temperature (Degree C)": output_temp,
        "snr (dB)": output_snr,
        "timestamp": time.time()
    })

    return {
        "sensor": "wingtip_strain",
        "phase": current_phase,
        "strain_microstrain": output_strain,
        "temperature_C": output_temp,
        "snr_dB": output_snr,
    }