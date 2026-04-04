import time
import random
import math

class FuelTempSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller

        self.phase_ranges = {
            'takeoff': (9, 12),
            'cruise': (-20, -5),
            'landing': (-4, 2)
        }

        # Enhanced thermal parameters for realistic behavior
        self.thermal_mass_factor = 0.85  # Fuel thermal inertia (0.8-0.9 for large tanks)
        self.max_gradient_limits = {
            'takeoff': 0.8,   # Higher during takeoff due to engine heat
            'cruise': 0.3,    # Very limited during cruise (steady state)
            'landing': 0.6    # Moderate during landing approach
        }
        
        # Thermal lag parameters
        self.thermal_history = []  # Store last N temperature readings
        self.thermal_history_size = 10
        self.ambient_influence = 0.05  # How much ambient temp affects fuel temp
        
        self.reading_counter = 0
        self.temp_log = []
        self.transition_buffer = []
        self.current_phase = None
        self.smoothed_temp = None
        self.prev_temp = None
        self.alpha = 0.15  # Reduced for more thermal inertia
        self.last_time = time.time()
        
        # Anomaly detection
        self.gradient_violations = 0
        self.max_violations_before_alert = 3

    def init_temperature(self, phase):
        """Initialize temperature within phase range, considering thermal history"""
        base_temp = random.uniform(*self.phase_ranges[phase])
        
        # If we have thermal history, bias towards recent temperatures
        if self.thermal_history:
            recent_avg = sum(self.thermal_history[-3:]) / len(self.thermal_history[-3:])
            phase_min, phase_max = self.phase_ranges[phase]
            # Blend recent temperature with phase-appropriate temperature
            if phase_min <= recent_avg <= phase_max:
                base_temp = recent_avg + random.uniform(-0.5, 0.5)
            else:
                # Gradual transition towards phase range
                if recent_avg < phase_min:
                    base_temp = recent_avg + random.uniform(0.2, 0.8)
                else:
                    base_temp = recent_avg - random.uniform(0.2, 0.8)
        
        return round(max(min(base_temp, self.phase_ranges[phase][1]), 
                        self.phase_ranges[phase][0]), 2)

    def apply_thermal_inertia(self, new_temp, phase):
        """Apply thermal mass effects to temperature changes"""
        if not self.thermal_history:
            return new_temp
        
        # Calculate weighted average of recent temperatures
        weights = [0.4, 0.3, 0.2, 0.1]  # Recent temps have higher weight
        weighted_avg = 0
        total_weight = 0
        
        for i, weight in enumerate(weights):
            if i < len(self.thermal_history):
                weighted_avg += self.thermal_history[-(i+1)] * weight
                total_weight += weight
        
        if total_weight > 0:
            weighted_avg /= total_weight
            
            # Apply thermal mass factor
            inertia_temp = (new_temp * (1 - self.thermal_mass_factor) + 
                           weighted_avg * self.thermal_mass_factor)
            return round(inertia_temp, 2)
        
        return new_temp

    def validate_gradient(self, current_temp, prev_temp, elapsed_sec, phase):
        """Validate and limit temperature gradient to realistic values"""
        if prev_temp is None or elapsed_sec <= 0:
            return 0.0
        
        delta_temp = current_temp - prev_temp
        delta_min = elapsed_sec / 60.0
        raw_gradient = delta_temp / delta_min
        
        # Apply phase-specific gradient limits
        max_gradient = self.max_gradient_limits[phase]
        
        if abs(raw_gradient) > max_gradient:
            # Limit the gradient and adjust temperature accordingly
            limited_gradient = max_gradient if raw_gradient > 0 else -max_gradient
            adjusted_temp = prev_temp + (limited_gradient * delta_min)
            
            # Ensure it stays within phase range
            adjusted_temp = self.clamp_to_phase_range(adjusted_temp, phase)
            
            # Track gradient violations for anomaly detection
            self.gradient_violations += 1
            if self.gradient_violations >= self.max_violations_before_alert:
                print(f"⚠️ WARNING: Thermal gradient anomaly detected in {phase.upper()} phase")
                print(f"   Attempted gradient: {raw_gradient:.2f} °C/min")
                print(f"   Limited to: {limited_gradient:.2f} °C/min")
                self.gradient_violations = 0  # Reset counter
            
            return adjusted_temp, limited_gradient
        else:
            # Reset violation counter on normal operation
            self.gradient_violations = max(0, self.gradient_violations - 1)
            return current_temp, raw_gradient

    def apply_noise(self, value, stddev):
        """Apply realistic sensor noise based on phase"""
        return round(value + random.gauss(0, stddev), 2)

    def clamp_to_phase_range(self, temp, phase):
        """Ensure temperature stays within realistic phase bounds"""
        min_val, max_val = self.phase_ranges[phase]
        return round(max(min(temp, max_val), min_val), 2)

    def calculate_gradient(self, current_temp, prev_temp, elapsed_sec, phase):
        """Calculate temperature gradient with validation"""
        delta_temp = current_temp - prev_temp
        delta_min = elapsed_sec / 60.0
        if delta_min <= 0:
            return 0.0
        gradient = delta_temp / delta_min

        # Apply realistic limits
        max_gradient = self.max_gradient_limits[phase]
        gradient = max(min(gradient, max_gradient), -max_gradient)
        
        return round(gradient, 2) if abs(gradient) > 0.01 else 0.0

    def get_environmental_influence(self, phase):
        """Simulate environmental temperature influence on fuel"""
        # Cruise phase: stable cold environment
        if phase == 'cruise':
            # Simulate slight ambient temperature variations at altitude
            ambient_variation = random.uniform(-0.02, 0.02)
            return ambient_variation * self.ambient_influence
        elif phase == 'takeoff':
            # Warmer ground conditions
            return random.uniform(0.01, 0.03)
        else:  # landing
            # Approach conditions
            return random.uniform(-0.01, 0.02)

    def interpolate_transition(self, current, target, phase, steps):
        """Create smooth transition between phases with thermal inertia"""
        min_val, max_val = self.phase_ranges[phase]
        
        # Calculate step size with thermal damping
        base_step = (target - current) / steps
        damping_factor = 0.7  # Reduce step size for thermal inertia
        
        values = []
        for i in range(1, steps + 1):
            # Apply exponential smoothing for more realistic thermal response
            progress = i / steps
            smoothed_progress = 1 - math.exp(-3 * progress)  # Exponential approach
            
            interpolated = current + (target - current) * smoothed_progress * damping_factor
            
            # Add minimal noise during transition
            interpolated += random.uniform(-0.02, 0.02)
            
            # Ensure within bounds
            clamped = round(max(min(interpolated, max_val), min_val), 2)
            values.append(clamped)
        
        return values

    def get_dynamic_jitter(self, phase, reading_count, temp_delta):
        """Calculate realistic sensor jitter based on phase and conditions"""
        if phase == 'cruise':
            # Lower jitter in stable cruise conditions
            base_jitter = 0.03 * math.exp(-reading_count / 30.0)
            base_jitter = max(base_jitter, 0.01)  # Minimum sensor noise
        elif phase == 'landing':
            # Moderate jitter during approach
            base_jitter = 0.02 + min(0.04, reading_count * 0.001)
        else:  # takeoff
            # Higher jitter during takeoff due to engine vibration
            base_jitter = 0.04 + random.uniform(-0.01, 0.01)
        
        # Limit jitter to reasonable sensor specifications
        base_jitter = max(min(base_jitter, 0.08), 0.01)
        return round(base_jitter, 3)

    def update_fueltemp_sensor(self):
        """Main sensor update method with enhanced thermal modeling"""
        phase = self.phase_controller.get_current_phase()
        now = time.time()
        elapsed_time = now - self.last_time
        self.last_time = now

        # Detect phase change
        if phase != self.current_phase:
            print(f"🔄 Phase transition: {self.current_phase} → {phase}")
            next_temp = self.init_temperature(phase)
            transition_steps = 25  # Longer transition for thermal inertia
            
            if self.smoothed_temp is not None:
                self.transition_buffer = self.interpolate_transition(
                    self.smoothed_temp, next_temp, phase, transition_steps)
            
            self.reading_counter = 0
            self.temp_log = []
            self.current_phase = phase
            
            if not self.transition_buffer:
                self.smoothed_temp = next_temp
            self.prev_temp = self.smoothed_temp

        self.reading_counter += 1

        # Handle temperature calculation
        if self.transition_buffer:
            # Use pre-calculated transition values
            temp = self.transition_buffer.pop(0)
        else:
            # Normal operation with enhanced thermal modeling
            target_min, target_max = self.phase_ranges[phase]
            mid_target = (target_min + target_max) / 2
            
            # Calculate natural thermal drift
            direction = 1 if mid_target > self.smoothed_temp else -1
            
            # Phase-specific thermal behavior
            if phase == 'cruise':
                # Very stable with minimal drift
                base_trend = random.uniform(0.0005, 0.002) * direction
                stddev = 0.008  # Reduced noise for cruise stability
            elif phase == 'takeoff':
                # More dynamic due to engine heat
                base_trend = random.uniform(0.003, 0.008) * direction
                stddev = 0.02
            else:  # landing
                # Moderate changes during approach
                base_trend = random.uniform(0.001, 0.005) * direction
                stddev = 0.015
            
            # Apply environmental influence
            env_influence = self.get_environmental_influence(phase)
            
            # Calculate new temperature
            new_temp = self.smoothed_temp + base_trend + env_influence
            
            # Apply sensor noise
            noisy_temp = self.apply_noise(new_temp, stddev)
            
            # Apply thermal inertia
            inertia_temp = self.apply_thermal_inertia(noisy_temp, phase)
            
            # Validate gradient and adjust if necessary
            adjusted_temp, gradient = self.validate_gradient(
                inertia_temp, self.prev_temp, elapsed_time, phase)
            
            # Final smoothing
            temp = round(self.alpha * adjusted_temp + (1 - self.alpha) * self.smoothed_temp, 2)
            temp = self.clamp_to_phase_range(temp, phase)

        # Update thermal history
        self.thermal_history.append(temp)
        if len(self.thermal_history) > self.thermal_history_size:
            self.thermal_history.pop(0)

        # Prevent identical consecutive readings
        if self.temp_log and abs(temp - self.temp_log[-1]) < 0.005:
            temp += random.uniform(-0.01, 0.01)
            temp = round(temp, 2)
            temp = self.clamp_to_phase_range(temp, phase)

        # Calculate final gradient
        gradient = 0.0
        if self.reading_counter > 1 and self.prev_temp is not None:
            gradient = self.calculate_gradient(temp, self.prev_temp, 1.0, phase)

        # Update state
        self.smoothed_temp = temp
        self.temp_log.append(temp)
        if len(self.temp_log) > 10:
            self.temp_log.pop(0)
        
        self.prev_temp = temp

        # Calculate jitter
        temp_delta = temp - (self.temp_log[-2] if len(self.temp_log) > 1 else temp)
        jitter = self.get_dynamic_jitter(phase, self.reading_counter, temp_delta)

        # Enhanced logging with thermal status
        thermal_stability = "STABLE" if abs(gradient) < 0.1 else "CHANGING"
        print(
            f"[{phase.upper()}] Temp: {temp:.2f}°C | "
            f"Grad: {gradient:.2f}°C/min | Jitter: {jitter:.3f}°C | "
            f"Status: {thermal_stability}"
        )

        # Publish sensor data
        self.bus.publish("wing/fueltemp", {
            "sensor": "fueltemp",
            "phase": phase,
            "temperature (Degree C)": temp,
            "gradient (Degree C/min)": gradient,
            "jitter (Degree C)": jitter,
            "timestamp": now
        })

        return {
            "sensor": "fueltemp",
            "phase": phase,
            "temperature_C": temp,
            "gradient_C_per_min": gradient,
            "jitter_C": jitter,
            "thermal_stability": thermal_stability
        }
    
    def get_diagnostic_info(self):
        """Return diagnostic information about sensor state"""
        return {
            "current_phase": self.current_phase,
            "current_temp": self.smoothed_temp,
            "thermal_history": self.thermal_history.copy(),
            "gradient_violations": self.gradient_violations,
            "reading_counter": self.reading_counter,
            "transition_buffer_size": len(self.transition_buffer)
        }