import time
import random
import math

class WingSparLoadSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.bending_moment = 8000.0
        self.shear_force = 4000.0
        self.axial_load = 6000.0

        self.target_bending = 8000.0
        self.target_shear = 4000.0
        self.target_axial = 6000.0

        self.last_time = time.time()
        self.last_update_time = time.time()
        
        self.cruise_base_bending = 12000
        self.cruise_base_shear = 6000
        self.cruise_base_axial = 8000
        
        self.oscillation_phase = 0.0
        self.gust_amplitude = 0.0
        self.gust_decay = 0.95
        self.previous_axial = 6000.0

    def update_sensor(self):
        dt = 1.0
        now = time.time()
        phase = self.phase_controller.get_current_phase()

        if now - self.last_update_time > 2.5:
            self.target_bending, self.target_shear, self.target_axial = self.generate_target_values(phase)
            self.last_update_time = now

        approach_rate = 12000 if phase == 'takeoff' else 8000
        self.bending_moment = self.approach(self.bending_moment, self.target_bending, dt, rate=approach_rate)
        self.shear_force = self.approach(self.shear_force, self.target_shear, dt, rate=approach_rate//2)
        self.axial_load = self.approach(self.axial_load, self.target_axial, dt, rate=approach_rate//1.5)

        if phase == 'cruise':
            self.apply_cruise_dynamics()
        else:
            noise_factor = 0.3 if phase == 'takeoff' else 0.5
            self.bending_moment += self.get_noise(phase, 300 * noise_factor)
            self.shear_force += self.get_noise(phase, 150 * noise_factor)
            self.axial_load += self.get_noise(phase, 200 * noise_factor)

        self.bending_moment = round(max(-80000, min(self.bending_moment, 80000)), 2)
        self.shear_force = round(max(-40000, min(self.shear_force, 40000)), 2)
        self.axial_load = round(max(-60000, min(self.axial_load, 60000)), 2)

        print(
            f"[{phase.upper()}] Bending: {self.bending_moment:.2f} Nm, "
            f"Shear: {self.shear_force:.2f} N, "
            f"Axial: {self.axial_load:.2f} N"
        )

        self.bus.publish("wing/spar", {
            "sensor": "spar",
            "timestamp": now,
            "phase": phase,
            "bending_moment": self.bending_moment,
            "shear_force": self.shear_force,
            "axial_load": self.axial_load
        })

        return {
            "sensor": "spar",
            "phase": phase,
            "bending_moment_Nm": self.bending_moment,
            "shear_force_N": self.shear_force,
            "axial_load_N": self.axial_load
        }

    def generate_target_values(self, phase):
        if phase == 'takeoff':
            progression = min(1.0, (time.time() - self.last_time) / 10.0)
            base_bending = 8000 + (45000 * progression)
            base_shear = 4000 + (20000 * progression)
            base_axial = 6000 + (35000 * progression)
            
            return (
                base_bending + random.uniform(-3000, 5000),
                base_shear + random.uniform(-2000, 4000),
                base_axial + random.uniform(-2000, 8000)
            )
        elif phase == 'cruise':
            variation = random.uniform(0.88, 1.12)
            return (
                self.cruise_base_bending * variation,
                self.cruise_base_shear * variation,
                self.cruise_base_axial * variation
            )
        elif phase == 'landing':
            axial_change = random.uniform(-8000, 8000)
            smooth_axial = self.previous_axial + (axial_change * 0.3)
            smooth_axial = max(-25000, min(smooth_axial, 50000))
            self.previous_axial = smooth_axial
            
            return (
                random.uniform(35000, 62000),
                random.uniform(18000, 28000),
                smooth_axial
            )
        return (8000.0, 4000.0, 6000.0)

    def apply_cruise_dynamics(self):
        self.oscillation_phase += 0.15
        
        if random.random() < 0.08:
            self.gust_amplitude = random.uniform(800, 2000)
        self.gust_amplitude *= self.gust_decay
        
        smooth_osc = math.sin(self.oscillation_phase) * 600
        gust_effect = self.gust_amplitude * math.sin(self.oscillation_phase * 2.3)
        
        coupling_factor = 0.3
        bending_influence = (self.bending_moment - self.cruise_base_bending) * coupling_factor
        
        self.bending_moment += smooth_osc + gust_effect * 0.8
        self.shear_force += smooth_osc * 0.6 + gust_effect * 0.5
        self.axial_load += smooth_osc * 0.4 + gust_effect * 0.3 + bending_influence * 0.2
        
        if self.axial_load < 1000:
            self.axial_load = max(1000, self.axial_load + random.uniform(500, 1500))
        
        base_noise = random.gauss(0, 120)
        self.bending_moment += base_noise
        self.shear_force += base_noise * 0.7
        self.axial_load += base_noise * 0.5

    def approach(self, current, target, dt, rate):
        max_step = rate * dt
        delta = target - current
        step = max(-max_step, min(delta, max_step))
        return current + step

    def get_noise(self, phase, std_dev):
        if phase == 'cruise':
            noise = random.gauss(0, std_dev * 0.15)
        elif phase == 'takeoff':
            noise = random.gauss(0, std_dev * 0.4)
        elif phase == 'landing':
            noise = random.gauss(0, std_dev * 0.6)
        else:
            noise = random.gauss(0, std_dev * 0.25)
        return max(-std_dev, min(noise, std_dev))