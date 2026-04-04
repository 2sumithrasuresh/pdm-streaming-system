import time
import random

class FlutterSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.freq = 0.0
        self.amp = 0.0
        self.damp = 0.0

        self.reading_counter = 0
        self.prev_freq = None
        self.freq_plateau_count = 0
        self.prev_amp = 0.0
        self.flutter_suppression_active = False

    def apply_noise(self, value, stddev, min_val, max_val):
        noisy = value + random.normalvariate(0, stddev)
        return round(max(min(noisy, max_val), min_val), 3)

    def frequency_stability_filter(self, new_freq, max_change_rate=0.5):
        if self.prev_freq is not None:
            freq_change = abs(new_freq - self.prev_freq)
            if freq_change > max_change_rate:
                stabilized_freq = self.prev_freq + (max_change_rate if new_freq > self.prev_freq else -max_change_rate)
                return stabilized_freq
        return new_freq

    def flutter_suppression_logic(self, phase, amp, damp):
        if phase == 'landing':
            if amp > 4.0 or damp < 0.06:
                self.flutter_suppression_active = True
                amp = min(amp * 0.8, 4.0)
                damp = max(damp * 1.2, 0.06)
            elif self.flutter_suppression_active and amp < 3.0 and damp > 0.055:
                self.flutter_suppression_active = False
        
        elif phase == 'takeoff':
            if damp < 0.045:
                damp = max(damp * 1.15, 0.045)
            if amp > 3.3 and damp < 0.05:
                amp = min(amp * 0.9, 3.3)
        
        elif phase == 'cruise':
            if damp < 0.09:
                damp = max(damp * 1.05, 0.09)
        
        return amp, damp

    def set_phase_targets(self, phase):
        if phase == 'takeoff':
            self.freq = random.uniform(4.0, 6.0)
            self.amp = random.uniform(2.0, 3.3)
            self.damp = random.uniform(0.045, 0.073)
        elif phase == 'cruise':
            self.freq = random.uniform(8.5, 10.0)
            self.amp = random.uniform(0.5, 1.5)
            self.damp = random.uniform(0.09, 0.115)
        elif phase == 'landing':
            base_freq = 6.5
            step = 0.05
            i = self.reading_counter
            self.freq = base_freq - step * i + random.uniform(-0.05, 0.05)
            self.amp = random.uniform(2.0, 4.0)
            self.damp = random.uniform(0.06, 0.08)

    def generate_next(self, phase):
        self.reading_counter += 1

        if self.prev_freq is None:
            self.prev_freq = self.freq

        if phase == 'takeoff':
            base = self.prev_freq + random.uniform(-0.03, 0.03)
            freq = self.apply_noise(base, 0.02, 4.0, 15.0)
        elif phase == 'cruise':
            freq = random.uniform(8.5, 10.0)
            freq = self.frequency_stability_filter(freq)
        else:
            base_freq = 6.5
            step = 0.05
            i = self.reading_counter
            freq = base_freq - step * i + random.uniform(-0.05, 0.05)

        freq = max(4.0, min(freq, 50.0))

        if abs(freq - self.prev_freq) < 0.01:
            self.freq_plateau_count += 1
        else:
            self.freq_plateau_count = 0

        if self.freq_plateau_count >= 3:
            freq += random.uniform(0.02, 0.05)
            self.freq_plateau_count = 0

        self.freq = round(freq, 2)
        self.prev_freq = self.freq

        if phase == 'takeoff':
            amp_target = 3.0
        elif phase == 'cruise':
            amp_target = 1.0
        else:
            amp_target = 3.5

        amp_trend = (amp_target - self.amp) * 0.1
        self.amp += amp_trend + random.normalvariate(0, 0.05)

        if phase == 'cruise':
            self.amp = max(0.2, min(self.amp, 1.8))
        elif phase == 'takeoff':
            self.amp = max(0.05, min(self.amp, 3.3))
        else:
            self.amp = max(0.05, min(self.amp, 4.0))

        self.amp = round(self.amp, 2)

        if phase == 'cruise':
            base = 0.115 - (self.amp * 0.01)
            base = max(0.09, min(base, 0.115))
        elif phase == 'takeoff':
            base = random.uniform(0.045, 0.073)
        else:
            base = random.uniform(0.06, 0.08)

        if phase == 'cruise':
            self.damp = self.apply_noise(base, 0.005, 0.09, 0.115)
        elif phase == 'takeoff':
            self.damp = self.apply_noise(base, 0.01, 0.045, 0.7)
        else:
            self.damp = self.apply_noise(base, 0.01, 0.06, 0.7)

        self.amp, self.damp = self.flutter_suppression_logic(phase, self.amp, self.damp)

    def update_flutter_sensor(self):
        phase = self.phase_controller.get_current_phase()

        if self.reading_counter == 0:
            self.set_phase_targets(phase)

        self.generate_next(phase)

        print(
            f"[{phase.upper()}] Frequency: {self.freq:.2f} Hz, "
            f"Amplitude: {self.amp:.2f} mm, "
            f"Damping Ratio: {self.damp:.3f}"
        )

        self.bus.publish("wing/flutter", {
            "sensor": "flutter",
            "phase": phase,
            "frequency (Hz)": self.freq,
            "amplitude (mm)": self.amp,
            "damping_ratio": self.damp,
            "timestamp": time.time()
        })

        return {
            "sensor": "flutter",
            "phase": phase,
            "frequency_Hz": self.freq,
            "amplitude_mm": self.amp,
            "damping_ratio": self.damp
        }