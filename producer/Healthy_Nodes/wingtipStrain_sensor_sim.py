import time
import random
import math

class WingtipStrainSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.strain = 0.0
        self.temperature = 0.0
        self.snr = 0.0
        self.reading_counter = 0
        self.last_phase = None

    def apply_gaussian_noise(self, value, stddev, min_val, max_val):
        noisy_value = value + random.gauss(0, stddev)
        return round(max(min(noisy_value, max_val), min_val), 2)

    def generate_next(self, phase):
        self.reading_counter += 1
        t = self.reading_counter

        # === Strain (µε) ===
        if phase == 'takeoff':
            base = 400 + math.sin(t * 0.2) * 50
            self.strain = self.apply_gaussian_noise(base, 8.0, -500, 500)
        elif phase == 'cruise':
            base = 100 + math.sin(t * 0.1) * 20
            self.strain = self.apply_gaussian_noise(base, 5.0, -200, 200)
        elif phase == 'landing':
            base = 350 + math.sin(t * 0.3) * 40
            self.strain = self.apply_gaussian_noise(base, 8.0, -500, 500)

        # === Temperature (°C) ===
        if phase == 'takeoff':
            base = 20 + (t * 0.4)
            self.temperature = self.apply_gaussian_noise(base, 0.8, 15, 35)
        elif phase == 'cruise':
            self.temperature = self.apply_gaussian_noise(-20, 1.0, -30, -10)
        elif phase == 'landing':
            base = -5 + (t * 0.5)
            self.temperature = self.apply_gaussian_noise(base, 0.9, -10, 25)

        # === SNR (dB) ===
        if phase == 'takeoff':
            base = 25 - abs(math.sin(t * 0.3)) * 3
            self.snr = self.apply_gaussian_noise(base, 0.4, 22, 28)
        elif phase == 'cruise':
            base = 32 + math.sin(t * 0.2) * 1.5
            self.snr = self.apply_gaussian_noise(base, 0.3, 28, 35)
        elif phase == 'landing':
            base = 24 - abs(math.sin(t * 0.25)) * 3
            self.snr = self.apply_gaussian_noise(base, 0.4, 20, 27)

    def update_wingtip_strain(self):
        phase = self.phase_controller.get_current_phase()

        if phase != self.last_phase:
            self.reading_counter = 0
            self.last_phase = phase

        now = time.time()
        self.generate_next(phase)

        print(
            f"[{phase.upper()}] Strain: {self.strain:.2f} µε, "
            f"Temperature: {self.temperature:.2f} °C, "
            f"SNR: {self.snr:.2f} dB"
        )

        self.bus.publish("wing/wingtip_strain", {
            "sensor": "wingtip_strain",
            "phase": phase,
            "strain (microstrain)": self.strain,
            "temperature (Degree C)": self.temperature,
            "snr (dB)": self.snr,
            "timestamp": now
        })
        
        return {
            "sensor": "wingtip_strain",
            "phase": phase,
            "strain_microstrain": self.strain,
            "temperature_C": self.temperature,
            "snr_dB": self.snr
        }