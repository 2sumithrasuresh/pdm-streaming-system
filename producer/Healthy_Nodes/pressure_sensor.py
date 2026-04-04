import time
import random
import math

class WingSurfacePressureSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.dynamic_pressure = 0.0
        self.pressure_gradient = 0.0
        self.asymmetry_index = 0.0
        self.reading_counter = 0
        self.last_phase = None
        
        # Enhanced asymmetry smoothing with longer history
        self.asymmetry_history = []
        self.pressure_history = []
        self.gradient_history = []
        
        # More restrictive phase limits for 99% accuracy
        self.phase_limits = {
            'takeoff': {
                'asymmetry_max': 0.20,  # Reduced from 0.25
                'spike_probability': 0.01,  # Reduced from 0.015
                'pressure_range': (160, 1100),
                'gradient_range': (1.4, 2.6)
            },
            'cruise': {
                'asymmetry_max': 0.08,  # Reduced from 0.12
                'spike_probability': 0.003,  # Reduced from 0.005
                'pressure_range': (1050, 1150),
                'gradient_range': (3.3, 3.9)
            },
            'landing': {
                'asymmetry_max': 0.12,  # Reduced from 0.28
                'spike_probability': 0.01,  # Reduced from 0.02
                'pressure_range': (970, 1200),
                'gradient_range': (3.9, 4.2)
            }
        }

    def apply_gaussian_noise(self, value, stddev, min_val, max_val):
        noisy_value = value + random.gauss(0, stddev)
        return round(max(min(noisy_value, max_val), min_val), 2)

    def apply_uniform_noise(self, value, jitter_range, min_val, max_val):
        noisy_value = value + random.uniform(-jitter_range, jitter_range)
        return round(max(min(noisy_value, max_val), min_val), 2)
    
    def smooth_asymmetry(self, new_value, phase):
        # Extended history for better smoothing with phase-specific decay
        if len(self.asymmetry_history) >= 6:  # Increased to 6 for better stability
            self.asymmetry_history.pop(0)
        
        self.asymmetry_history.append(new_value)
        
        if len(self.asymmetry_history) == 1:
            return new_value
        
        # Phase-specific asymmetry control
        if phase == 'cruise':
            # Ultra-stable cruise with asymmetry decay
            if len(self.asymmetry_history) >= 3:
                # Apply decay factor to keep asymmetry from trending upward
                recent_avg = sum(self.asymmetry_history[-3:]) / 3
                if recent_avg > 0.06:  # If trending high, apply stronger decay
                    decay_factor = 0.85
                    new_value *= decay_factor
        
        elif phase == 'landing':
            # Aggressive smoothing for landing to prevent spikes
            if new_value > 0.12:  # Cap high values more aggressively
                new_value = min(new_value, 0.12 + (new_value - 0.12) * 0.3)
        
        # Enhanced weighted smoothing
        if len(self.asymmetry_history) == 2:
            weights = [0.2, 0.8]
        elif len(self.asymmetry_history) == 3:
            weights = [0.1, 0.3, 0.6]
        elif len(self.asymmetry_history) == 4:
            weights = [0.05, 0.15, 0.35, 0.45]
        elif len(self.asymmetry_history) == 5:
            weights = [0.03, 0.07, 0.2, 0.3, 0.4]
        else:  # 6 values
            weights = [0.02, 0.05, 0.13, 0.2, 0.3, 0.3]
        
        weighted_sum = sum(val * weight for val, weight in 
                          zip(self.asymmetry_history, weights[-len(self.asymmetry_history):]))
        weight_total = sum(weights[-len(self.asymmetry_history):])
        
        smoothed = weighted_sum / weight_total
        
        max_asymmetry = self.phase_limits[phase]['asymmetry_max']
        return round(min(smoothed, max_asymmetry), 3)

    def smooth_pressure(self, new_value, phase):
        # Enhanced pressure smoothing to prevent drops and unrealistic jumps
        if len(self.pressure_history) >= 3:
            self.pressure_history.pop(0)
        
        self.pressure_history.append(new_value)
        
        if len(self.pressure_history) == 1:
            return new_value
        
        # Phase-specific smoothing with monotonic enforcement for takeoff
        if phase == 'takeoff' and len(self.pressure_history) >= 2:
            # Prevent pressure drops during takeoff
            if new_value < self.pressure_history[-2]:
                # Apply stronger smoothing to prevent drops
                smoothed = self.pressure_history[-2] * 0.7 + new_value * 0.3
            else:
                smoothed = self.pressure_history[-2] * 0.4 + new_value * 0.6
        else:
            # Normal smoothing for cruise and landing
            if len(self.pressure_history) == 2:
                smoothed = (self.pressure_history[0] * 0.3 + self.pressure_history[1] * 0.7)
            else:  # 3 values
                smoothed = (self.pressure_history[0] * 0.2 + self.pressure_history[1] * 0.3 + self.pressure_history[2] * 0.5)
        
        return round(smoothed, 2)

    def smooth_gradient(self, new_value):
        # Smooth gradient readings
        if len(self.gradient_history) >= 3:
            self.gradient_history.pop(0)
        
        self.gradient_history.append(new_value)
        
        if len(self.gradient_history) == 1:
            return new_value
        
        # Simple moving average
        if len(self.gradient_history) == 2:
            return round((self.gradient_history[0] * 0.4 + self.gradient_history[1] * 0.6), 2)
        else:  # 3 values
            return round((self.gradient_history[0] * 0.2 + self.gradient_history[1] * 0.3 + self.gradient_history[2] * 0.5), 2)

    def generate_next(self, phase):
        self.reading_counter += 1
        t = self.reading_counter

        # Enhanced pressure generation with better bounds
        if phase == 'takeoff':
            # Monotonic increase with micro-adjustments to prevent drops
            base = 170 + 750 * (1 - math.cos(t * 0.075)) / 2
            # Add small progressive increment to ensure no drops
            base += t * 2.5
            raw_pressure = self.apply_uniform_noise(base, 18.0, 160, 1100)
        elif phase == 'cruise':
            # Ultra-stable cruise with minimal oscillation
            base = 1112 + math.sin(t * 0.03) * 12
            raw_pressure = self.apply_uniform_noise(base, 6.0, 1050, 1150)
        elif phase == 'landing':
            # Controlled descent with smoother profile
            base = 1150 - 580 * (1 - math.cos(t * 0.065)) / 2
            raw_pressure = self.apply_uniform_noise(base, 15.0, 970, 1200)
        
        self.dynamic_pressure = self.smooth_pressure(raw_pressure, phase)

        # Enhanced gradient generation
        if phase == 'takeoff':
            base = 1.4 + 1.2 * (1 - math.cos(t * 0.07)) / 2
            raw_gradient = self.apply_uniform_noise(base, 0.2, 1.4, 2.8)
        elif phase == 'cruise':
            base = 3.6 + math.sin(t * 0.06) * 0.15
            raw_gradient = self.apply_uniform_noise(base, 0.15, 3.2, 4.0)
        elif phase == 'landing':
            base = 4.0 + math.sin(t * 0.08) * 0.2
            # Reduce landing gradient spike intensity
            if 15 <= t <= 20:
                base += 0.3 * math.sin((t - 15) * math.pi / 5)
            raw_gradient = self.apply_uniform_noise(base, 0.25, 3.8, 4.3)
        
        self.pressure_gradient = self.smooth_gradient(raw_gradient)

        # Enhanced asymmetry generation with ultra-tight control
        phase_config = self.phase_limits[phase]
        
        if phase == 'takeoff':
            # Stable takeoff asymmetry
            base = 0.05 + math.sin(t * 0.06) * 0.02
            if random.random() < phase_config['spike_probability']:
                base += random.uniform(0.01, 0.03)
            raw_asymmetry = self.apply_uniform_noise(base, 0.05, 0.0, 0.15)
            
        elif phase == 'cruise':
            # Ultra-stable cruise with decay mechanism
            base = 0.035 + math.sin(t * 0.03) * 0.015
            # Apply slow decay to prevent upward trending
            if t > 10:
                decay = 0.998 ** (t - 10)  # Gradual decay after initial readings
                base *= decay
            raw_asymmetry = self.apply_uniform_noise(base, 0.02, 0.0, 0.08)
            
        elif phase == 'landing':
            # Strictly controlled landing asymmetry
            base = 0.06 + math.sin(t * 0.07) * 0.02
            
            if random.random() < phase_config['spike_probability']:
                base += random.uniform(0.02, 0.04)
            
            # Minimal turbulence effect
            if t > 20:
                turbulence = 0.015 * math.sin(t * 0.2) * random.uniform(0.98, 1.02)
                base += turbulence
            
            raw_asymmetry = self.apply_uniform_noise(base, 0.05, 0.0, 0.12)
        
        self.asymmetry_index = self.smooth_asymmetry(raw_asymmetry, phase)

    def update_wing_surface_pressure(self):
        phase = self.phase_controller.get_current_phase()

        if phase != self.last_phase:
            self.reading_counter = 0
            self.last_phase = phase
            self.asymmetry_history = []
            self.pressure_history = []
            self.gradient_history = []

        now = time.time()
        self.generate_next(phase)

        validation_status = self.validate_readings(phase)
        
        print(
            f"[{phase.upper()}] Dynamic Pressure: {self.dynamic_pressure:.2f} Pa, "
            f"Pressure Gradient: {self.pressure_gradient:.2f} Pa/cm, "
            f"Asymmetry Index: {self.asymmetry_index:.3f} % {validation_status}"
        )

        self.bus.publish("wing/surface_pressure", {
            "sensor": "surface_pressure",
            "phase": phase,
            "dynamic_pressure (Pa)": self.dynamic_pressure,
            "pressure_gradient (Pa/cm)": self.pressure_gradient,
            "asymmetry_index (%)": self.asymmetry_index,
            "timestamp": now
        })
    
        return {
            "sensor": "surface_pressure",
            "phase": phase,
            "dynamic_pressure_Pa": self.dynamic_pressure,
            "pressure_gradient_Pa_per_cm": self.pressure_gradient,
            "asymmetry_index_percent": self.asymmetry_index,
        }
    
    def validate_readings(self, phase):
        warnings = []
        
        if phase == 'takeoff':
            if self.asymmetry_index > 0.15:
                warnings.append("⚠️ HIGH_ASYMMETRY")
            if self.dynamic_pressure < 165 or self.dynamic_pressure > 1080:
                warnings.append("⚠️ PRESSURE_ANOMALY")
            if self.pressure_gradient > 2.5:
                warnings.append("⚠️ HIGH_GRADIENT")
                
        elif phase == 'cruise':
            if self.asymmetry_index > 0.075:
                warnings.append("⚠️ CRUISE_ASYMMETRY")
            if self.dynamic_pressure < 1055 or self.dynamic_pressure > 1145:
                warnings.append("⚠️ PRESSURE_DEVIATION")
            if self.pressure_gradient < 3.35 or self.pressure_gradient > 3.85:
                warnings.append("⚠️ GRADIENT_DEVIATION")
                
        elif phase == 'landing':
            if self.asymmetry_index > 0.11:
                warnings.append("⚠️ LANDING_ASYMMETRY")
            if self.pressure_gradient > 4.15:
                warnings.append("⚠️ HIGH_GRADIENT")
            if self.dynamic_pressure < 975:
                warnings.append("⚠️ LOW_PRESSURE")
        
        return " ".join(warnings) if warnings else "✅"