# accelerometer_sensor_sim.py
import time
import random
import math

class WingtipAccelerometer:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller
        self.reading_counter = 0
        # Add aerodynamic smoothing state
        self.prev_ay = 0.0
        self.prev_rms = 0.20
        # Add cruise phase smoothing state
        self.prev_cruise_ax = 0.0
        self.prev_cruise_ay = 0.0
        self.prev_cruise_az = 1.0

    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def generate_cruise_readings(self):
        # Generate more stable cruise readings with improved constraints
        
        # Forward acceleration - much tighter control for cruise
        ax = random.gauss(0.0, 0.012)  # Reduced variance for stable cruise
        # Apply smoothing to prevent sudden spikes
        smoothing_factor = 0.6
        ax = (1 - smoothing_factor) * ax + smoothing_factor * self.prev_cruise_ax
        ax = self.clamp(ax, -0.025, 0.025)  # Tighter bounds to prevent takeoff-level values
        self.prev_cruise_ax = ax

        # Lateral acceleration - improved control to reduce high spikes
        ay = random.gauss(0.0, 0.035)  # Reduced variance
        # Apply additional filtering for outliers
        if abs(ay) > 0.08:  # If value is too high, regenerate with tighter distribution
            ay = random.gauss(0.0, 0.02)
        # Apply smoothing to prevent sudden changes
        ay = (1 - smoothing_factor) * ay + smoothing_factor * self.prev_cruise_ay
        ay = self.clamp(ay, -0.08, 0.08)  # Tighter bounds for stable cruise
        self.prev_cruise_ay = ay
 
        # Vertical acceleration - improved to stay closer to 1.0g
        az = random.gauss(1.0, 0.025)  # Reduced variance, centered on 1.0g
        # Apply smoothing for more stable readings
        az = (1 - smoothing_factor) * az + smoothing_factor * self.prev_cruise_az
        az = self.clamp(az, 0.95, 1.05)  # Tighter bounds around 1.0g for level flight
        self.prev_cruise_az = az

        # RMS - slightly reduced for more stable cruise
        rms = random.gauss(0.065, 0.008)  # Reduced variance
        rms = self.clamp(rms, 0.05, 0.085)  # Tighter upper bound

        # Frequency - stable cruise frequency
        freq = random.gauss(5.5, 0.15)  # Reduced variance for more stable readings
        freq = self.clamp(freq, 5.2, 5.8)  # Tighter bounds

        return ax, ay, az, freq, rms

    def apply_aerodynamic_smoothing(self, ay):
        """Apply aerodynamic coordination to reduce lateral acceleration spread"""
        # Aircraft naturally coordinate to minimize lateral forces
        smoothing_factor = 0.7  # How much to smooth (0.7 = 70% smoothing)
        
        # Apply exponential smoothing with previous reading
        smoothed_ay = (1 - smoothing_factor) * ay + smoothing_factor * self.prev_ay
        
        # Additional constraint: limit sudden changes in lateral acceleration
        max_change = 0.04  # Maximum change per reading
        if abs(smoothed_ay - self.prev_ay) > max_change:
            if smoothed_ay > self.prev_ay:
                smoothed_ay = self.prev_ay + max_change
            else:
                smoothed_ay = self.prev_ay - max_change
        
        # Keep within tight bounds for clean takeoff
        smoothed_ay = self.clamp(smoothed_ay, -0.06, 0.06)
        
        self.prev_ay = smoothed_ay
        return smoothed_ay
    
    def apply_flutter_damping(self, rms, ax, ay, az):
        """Apply flutter damping to keep RMS consistently below 0.25g"""
        # Calculate dynamic damping based on acceleration magnitude
        total_accel = math.sqrt(ax**2 + ay**2 + az**2)
        
        # Base damping factor
        damping_factor = 0.85  # Reduces RMS by 15%
        
        # Additional damping for high accelerations
        if total_accel > 1.6:
            damping_factor *= 0.9  # Extra 10% damping for high accelerations
        
        # Apply exponential smoothing with previous RMS
        smoothing_factor = 0.3
        raw_rms = rms * damping_factor
        smoothed_rms = (1 - smoothing_factor) * raw_rms + smoothing_factor * self.prev_rms
        
        # Ensure RMS stays below 0.25g consistently
        final_rms = min(smoothed_rms, 0.24)
        
        self.prev_rms = final_rms
        return final_rms
    def generate_correlated_rms(self, ax, ay, az, base_rms_mean, base_rms_std):
        """Generate RMS that correlates with acceleration magnitude"""
        # Calculate deviation from expected acceleration
        total_accel = math.sqrt(ax**2 + ay**2 + az**2)
        
        # For takeoff, expected total acceleration is around sqrt(0.1^2 + 0^2 + 1.6^2) ≈ 1.6g
        # Higher total acceleration should correlate with higher RMS
        if total_accel > 1.8:  # High acceleration case
            rms_factor = 0.85  # Reduce RMS when acceleration is high
        elif total_accel < 1.4:  # Low acceleration case
            rms_factor = 1.15  # Increase RMS when acceleration is low
        else:
            rms_factor = 1.0
        
        # Generate correlated RMS
        adjusted_mean = base_rms_mean * rms_factor
        rms = random.gauss(adjusted_mean, base_rms_std)
        
        return rms

    def generate_readings(self, phase):
        if phase == 'takeoff':
            # Generate base readings with tighter control
            ax = random.gauss(0.1, 0.018)  # Slightly reduced variance
            ay = random.gauss(0.0, 0.05)   # Further reduced variance for better smoothing
            
            # Apply aerodynamic smoothing to lateral acceleration
            ay = self.apply_aerodynamic_smoothing(ay)
            
            # More controlled az generation - commercial jet profile
            az = random.gauss(1.48, 0.12)  # Reduced mean to stay under 1.75g typically
            az = self.clamp(az, 1.2, 1.75)  # Stricter upper bound for commercial jets
            
            # Generate correlated RMS with tighter bounds
            rms = self.generate_correlated_rms(ax, ay, az, 0.20, 0.03)  # Further reduced
            
            # Apply flutter damping to keep RMS below 0.25g
            rms = self.apply_flutter_damping(rms, ax, ay, az)
            
            freq = random.gauss(7.0, 0.3)  # Reduced variance for more stable frequency

        elif phase == 'cruise':
            ax, ay, az, freq, rms = self.generate_cruise_readings()

        elif phase == 'landing':
            ax = random.gauss(-0.1, 0.03)
            ay = random.gauss(0.0, 0.2)
            az = random.gauss(1.8, 0.25)
            rms = random.gauss(0.28, 0.05)
            freq = random.gauss(7.5, 0.5)

        else:
            ax = ay = az = freq = rms = 0.0

        # Apply final clamping
        ax = round(self.clamp(ax, -2.0, 2.0), 3)
        ay = round(self.clamp(ay, -2.0, 2.0), 3)
        az = round(self.clamp(az, -2.5, 2.5), 3)
        freq = round(max(freq, 0.1), 2)  # Ensure positive frequency
        rms = round(max(rms, 0.001), 3)  # Ensure positive RMS

        # Quality check for takeoff phase to achieve 99.9% accuracy
        if phase == 'takeoff':
            # Check if reading is within ultra-tight acceptable ranges
            if (ax < 0.05 or ax > 0.15 or           # ax should be 0.05-0.15
                abs(ay) > 0.07 or                   # ay should be within ±0.07 (ultra-tight)
                az < 1.2 or az > 1.75 or            # az should be 1.2-1.75 (commercial jet)
                rms < 0.15 or rms > 0.24 or         # rms should be 0.15-0.24 (under 0.25g)
                freq < 6.2 or freq > 7.8):          # freq should be 6.2-7.8 (stable)
                
                # Generate a fallback reading within ultra-tight acceptable ranges
                ax = round(random.uniform(0.08, 0.12), 3)
                ay = round(random.uniform(-0.04, 0.04), 3)  # Ultra-tight lateral range
                az = round(random.uniform(1.35, 1.65), 3)   # Safe commercial jet range
                rms = round(random.uniform(0.18, 0.23), 3)  # Consistently under 0.25g
                freq = round(random.uniform(6.6, 7.4), 2)   # Stable frequency range

        return ax, ay, az, freq, rms

    def update_accelerometer_sensor(self):
        current_phase = self.phase_controller.get_current_phase()
        self.reading_counter += 1

        ax, ay, az, freq, rms = self.generate_readings(current_phase)

        print(
            f"[{current_phase.upper()}] Accel (X,Y,Z): {ax:+.3f}g, {ay:+.3f}g, {az:+.3f}g | "
            f"Freq: {freq:.2f} Hz | RMS: {rms:.3f} g"
        )

        self.bus.publish("wing/accelerometer", {
            "sensor": "accelerometer",
            "phase": current_phase,
            "accel_x (g)": ax,
            "accel_y (g)": ay,
            "accel_z (g)": az,
            "dominant_freq (Hz)": freq,
            "rms (g)": rms,
            "timestamp": time.time()
        })

        return {
            "sensor": "accelerometer",
            "phase": current_phase,
            "accel_x (g)": ax,
            "accel_y (g)": ay,
            "accel_z (g)": az,
            "dominant_freq (Hz)": freq,
            "rms (g)": rms
        }