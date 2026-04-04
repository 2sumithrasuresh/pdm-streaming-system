import time
import random
import math

class WingtipNodeSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.acceleration = 0.0
        self.deflection = 0.0
        self.strain = 0.0

        self.prev_acceleration = 0.0
        self.prev_deflection = 0.0
        self.prev_strain = 0.0

        self.transitioning = False
        self.transition_tick = 0
        self.transition_duration = 3
        self.current_phase = None
        self.tick_count = 0

        self.accel_strain_correlation = 0.9
        self.base_cruise_strain = 125
        self.strain_acceleration_factor = 200
        self.negative_accel_counter = 0
        self.max_negative_streak = 1

        self.accel_history = []
        self.strain_history = []
        self.deflection_history = []
        self.history_length = 3

        self.max_change_per_tick = {
            'acceleration': 0.35,
            'deflection': 4.5,
            'strain': 15.0
        }

    def apply_noise(self, value, std_dev):
        return value + random.gauss(0, std_dev * 0.3)

    def smooth_transition(self, start_value, end_value, duration, tick_elapsed):
        if duration == 0:
            return end_value
        t = min(tick_elapsed / duration, 1.0)
        smooth_t = 0.5 * (1 - math.cos(t * math.pi))
        return (1 - smooth_t) * start_value + smooth_t * end_value

    def limit_rate_of_change(self, new_value, prev_value, max_change):
        if abs(new_value - prev_value) > max_change:
            if new_value > prev_value:
                return prev_value + max_change
            else:
                return prev_value - max_change
        return new_value

    def get_moving_average(self, history, new_value, window_size=2):
        if len(history) >= window_size:
            return sum(history[-window_size:]) / window_size
        return new_value

    def simulate_acceleration(self, phase):
        if phase == 'takeoff':
            if self.tick_count < 3:
                base = random.uniform(0.8, 1.5)
            elif self.tick_count < 6:
                base = random.uniform(1.2, 2.2)
            else:
                base = random.uniform(0.6, 1.4)
        elif phase == 'cruise':
            base = random.uniform(0.18, 0.45)
            
            if self.strain > 290 and base < 0.15:
                base = random.uniform(0.15, 0.35)
            
            if self.negative_accel_counter < self.max_negative_streak and random.random() < 0.015:
                base = random.uniform(-0.03, 0.05)
                self.negative_accel_counter += 1
            else:
                self.negative_accel_counter = 0
                
            if random.random() < 0.008:
                base = random.uniform(0.55, 0.85)
        elif phase == 'landing':
            if self.tick_count < 4:
                base = random.uniform(0.6, 1.2)
            elif self.tick_count < 8:
                base = random.uniform(1.0, 1.8)
            else:
                base = random.uniform(1.2, 2.0)
        else:
            base = 0.0

        noisy = self.apply_noise(base, 0.12)
        limited = self.limit_rate_of_change(
            noisy, self.prev_acceleration, self.max_change_per_tick['acceleration']
        )
        
        return round(max(min(limited, 2.5), -0.05), 2)

    def simulate_deflection(self, phase, acceleration):
        if self.transitioning:
            base = self.smooth_transition(
                self.prev_deflection,
                self.target_deflection,
                self.transition_duration,
                self.transition_tick
            )
        else:
            if phase == 'takeoff':
                base_deflection = 15 + (acceleration * 20)
                base = base_deflection + random.uniform(-5, 5)
            elif phase == 'cruise':
                base = random.uniform(38, 85)
                base += acceleration * 12
                
                if self.tick_count > 8:
                    base = max(base, 70)
            elif phase == 'landing':
                accel_factor = max(0.5, 1.0 - ((acceleration - 0.5) * 0.3))
                
                if self.tick_count < 4:
                    base = random.uniform(70, 78) * accel_factor
                elif self.tick_count < 8:
                    base = random.uniform(45, 65) * accel_factor
                else:
                    base = random.uniform(25, 45) * accel_factor
                    
                base = max(base, 15)
            else:
                base = 0.0

        noisy = self.apply_noise(base, 0.6)
        limited = self.limit_rate_of_change(
            noisy, self.prev_deflection, self.max_change_per_tick['deflection']
        )
        
        return round(max(min(limited, 90.0), 0.0), 2)

    def simulate_strain(self, deflection, acceleration, phase):
        if phase == 'cruise':
            base_strain = self.base_cruise_strain + (deflection * 2.4)
            accel_contribution = acceleration * self.strain_acceleration_factor
            base_strain += accel_contribution
            
            if acceleration < 0.15 and self.strain > 280:
                base_strain = max(base_strain, self.prev_strain * 0.95)
                
            if abs(deflection - self.prev_deflection) < 0.5:
                base_strain = self.prev_strain + (base_strain - self.prev_strain) * 0.3
                
        elif phase == 'landing':
            base_strain = (deflection * 4.2) + (acceleration * 35)
            base_strain = max(base_strain, deflection * 3.8)
        else:
            base_strain = (deflection * 3.5) + (deflection ** 1.2 * 0.5)
            if acceleration > 0:
                base_strain += acceleration * 15
        
        noisy_strain = self.apply_noise(base_strain, 1.0)
        limited = self.limit_rate_of_change(
            noisy_strain, self.prev_strain, self.max_change_per_tick['strain']
        )
        
        if phase == 'cruise':
            cruise_max = 325
        elif phase == 'landing':
            cruise_max = 380
        else:
            cruise_max = 400
            
        return round(max(min(limited, cruise_max), 0.0), 2)

    def update_sensor_history(self):
        self.accel_history.append(self.acceleration)
        self.strain_history.append(self.strain)
        self.deflection_history.append(self.deflection)
        
        if len(self.accel_history) > self.history_length:
            self.accel_history.pop(0)
            self.strain_history.pop(0)
            self.deflection_history.pop(0)

    def apply_sensor_coupling(self):
        if len(self.accel_history) >= 2:
            accel_avg = sum(self.accel_history[-2:]) / 2
            strain_correction = accel_avg * 50
            
            if self.acceleration < 0.12 and self.strain > 280:
                self.strain = max(self.strain * 0.98, self.strain - 15)

    def update_wingtip_node(self):
        phase = self.phase_controller.get_current_phase()

        if phase != self.current_phase:
            self.prev_deflection = self.deflection
            self.prev_strain = self.strain
            self.prev_acceleration = self.acceleration
            self.transitioning = True
            self.transition_tick = 0
            self.tick_count = 0
            self.negative_accel_counter = 0
            self.accel_history = []
            self.strain_history = []
            self.deflection_history = []

            if phase == 'cruise':
                self.target_deflection = random.uniform(75, 85)
                self.target_strain = self.base_cruise_strain + (self.target_deflection * 2.4)
            elif phase == 'landing':
                self.target_deflection = 75.0
                self.target_strain = self.target_deflection * 4.2
            elif phase == 'takeoff':
                self.target_deflection = random.uniform(25, 45)
                self.target_strain = self.target_deflection * 3.5

            self.current_phase = phase

        self.tick_count += 1
        if self.transitioning:
            self.transition_tick += 1
            if self.transition_tick >= self.transition_duration:
                self.transitioning = False

        now = time.time()
        
        new_acceleration = self.simulate_acceleration(phase)
        new_deflection = self.simulate_deflection(phase, new_acceleration)
        new_strain = self.simulate_strain(new_deflection, new_acceleration, phase)

        self.prev_acceleration = self.acceleration
        self.prev_deflection = self.deflection
        self.prev_strain = self.strain

        self.acceleration = new_acceleration
        self.deflection = new_deflection
        self.strain = new_strain

        self.update_sensor_history()
        self.apply_sensor_coupling()

        print(
            f"[{phase.upper()}] Accel: {self.acceleration:.2f} g, "
            f"Deflection: {self.deflection:.2f} cm, "
            f"Strain: {self.strain:.2f} με"
        )

        self.bus.publish("wing/wingtip_node", {
            "sensor": "wingtip_node",
            "phase": phase,
            "acceleration (g)": self.acceleration,
            "deflection (cm)": self.deflection,
            "strain (με)": self.strain,
            "timestamp": now
        })

        return {
            "sensor": "wingtip_node",
            "phase": phase,
            "acceleration_g": self.acceleration,
            "deflection_cm": self.deflection,
            "strain_microstrain": self.strain
        }
    
class MockBus:
    def publish(self, topic, data):
        pass

class MockPhaseController:
    def __init__(self):
        self.phase = 'takeoff'
    
    def get_current_phase(self):
        return self.phase

if __name__ == "__main__":
    bus = MockBus()
    phase_controller = MockPhaseController()
    sensor = WingtipNodeSensor(bus, phase_controller)
    
    for i in range(10):
        sensor.update_wingtip_node()
        time.sleep(1)