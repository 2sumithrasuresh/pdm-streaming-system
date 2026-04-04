import time
import random
import math

class FlapSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller
        self.flap_angle = 0.0
        self.last_flap_angle = 0.0
        self.target_angle = 0.0
        self.actuator_velocity = 0.0
        self.hydraulic_pressure = 0.0
        self.motor_current = 0.0
        self.time_since_last_movement = 0.0
        self.hold_pressure_timer = 0.0
        self.is_locked = False
        self.last_update_time = time.time()

    def update_flap_sensor(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        phase = self.phase_controller.get_current_phase()

        if phase == 'takeoff':
            self.target_angle = 15.0
        elif phase == 'cruise':
            self.target_angle = 0.0
        elif phase == 'landing':
            self.target_angle = 30.0

        angle_error = self.target_angle - self.flap_angle
        tolerance = 0.2
        
        if abs(angle_error) > tolerance:
            max_velocity = 3.0
            desired_velocity = max(-max_velocity, min(max_velocity, angle_error * 2.0))
            
            proximity_factor = max(0.0, 1.0 - (abs(angle_error) / 5.0))
            if proximity_factor > 0:
                desired_velocity *= (1 - proximity_factor * 0.7)
            
            if self.hydraulic_pressure > 3450:
                desired_velocity *= 0.8
            
            max_acceleration = 5.0
            velocity_change = desired_velocity - self.actuator_velocity
            max_change = max_acceleration * dt
            
            if abs(velocity_change) > max_change:
                velocity_change = max_change if velocity_change > 0 else -max_change
            
            self.actuator_velocity += velocity_change
            
            angle_change = self.actuator_velocity * dt
            proposed_angle = self.flap_angle + angle_change
            
            if proposed_angle > self.target_angle and angle_change > 0:
                max_overshoot = 0.1
                if proposed_angle > self.target_angle + max_overshoot:
                    self.flap_angle = self.target_angle + max_overshoot
                    self.actuator_velocity = 0.0
                else:
                    self.flap_angle = proposed_angle
                    if proposed_angle > self.target_angle:
                        self.actuator_velocity *= 0.3
            elif proposed_angle < self.target_angle and angle_change < 0:
                max_overshoot = 0.1
                if proposed_angle < self.target_angle - max_overshoot:
                    self.flap_angle = self.target_angle - max_overshoot
                    self.actuator_velocity = 0.0
                else:
                    self.flap_angle = proposed_angle
                    if proposed_angle < self.target_angle:
                        self.actuator_velocity *= 0.3
            else:
                self.flap_angle = proposed_angle
            
            self.flap_angle = max(0.0, min(self.flap_angle, 30.0))
            
            self.time_since_last_movement = 0.0
            self.is_locked = False
        else:
            self.actuator_velocity *= 0.8
            if abs(self.actuator_velocity) < 0.01:
                self.actuator_velocity = 0.0
                self.time_since_last_movement += dt
                
                if self.time_since_last_movement > 0.3:
                    self.is_locked = True
        
        if abs(angle_error) < 0.1 and self.motor_current < 0.4 and abs(self.actuator_velocity) < 0.1:
            if self.time_since_last_movement > 0.1:
                self.is_locked = True
        
        if abs(angle_error) < 0.3:
            self.hold_pressure_timer += dt
        else:
            self.hold_pressure_timer = 0.0

        is_flap_moving = abs(self.actuator_velocity) > 0.01
        
        angle_delta = abs(self.flap_angle - self.last_flap_angle)
        is_actively_moving = angle_delta > (0.02 * dt)
        
        if phase == 'cruise':
            if is_actively_moving or not self.is_locked:
                self.motor_current = self.simulate_motor_current_improved(phase, is_flap_moving, angle_error)
            else:
                self.motor_current = 0.0
        else:
            self.motor_current = self.simulate_motor_current_improved(phase, is_flap_moving, angle_error)

        if phase == 'cruise':
            if is_actively_moving or not self.is_locked:
                self.hydraulic_pressure = self.simulate_hydraulic_pressure_improved(phase, is_flap_moving)
            else:
                self.hydraulic_pressure = 0.0
        else:
            self.hydraulic_pressure = self.simulate_hydraulic_pressure_improved(phase, is_flap_moving)

        if phase == 'cruise':
            if self.is_locked:
                drift_noise = random.gauss(0, 0.03)
                noisy_angle = round(self.flap_angle + drift_noise, 2)
            else:
                noisy_angle = round(self.flap_angle + random.gauss(0, 0.05), 2)
            
            if is_actively_moving or not self.is_locked:
                noisy_motor = self.motor_current + (random.gauss(0, 0.03) if self.motor_current > 0 else 0)
                noisy_pressure = self.hydraulic_pressure + (random.gauss(0, 3) if self.hydraulic_pressure > 0 else 0)
            else:
                noisy_motor = 0.0
                noisy_pressure = 0.0
        else:
            noisy_angle = round(self.flap_angle + random.gauss(0, 0.15), 2)
            noisy_motor = self.motor_current + (random.gauss(0, 0.03) if self.motor_current > 0 else 0)
            noisy_pressure = self.hydraulic_pressure + (random.gauss(0, 3) if self.hydraulic_pressure > 0 else 0)

        noisy_angle = round(max(0.0, min(noisy_angle, 35.0)), 2)
        noisy_motor = round(max(0.0, noisy_motor), 2)
        noisy_pressure = round(max(0.0, noisy_pressure), 2)

        status = "MOVING" if is_flap_moving else ("LOCKED" if self.is_locked else "HOLDING")
        
        print(
            f"[{phase.upper()}] Flap Angle: {noisy_angle:.2f}°, "
            f"Motor Current: {noisy_motor:.2f} A, "
            f"Hydraulic Pressure: {noisy_pressure:.2f} psi, "
            f"Status: {status}"
        )

        self.bus.publish("wing/flap", {
            "sensor": "flap",
            "timestamp": current_time,
            "phase": phase,
            "flap_angle": noisy_angle,
            "motor_current": noisy_motor,
            "hydraulic_pressure": noisy_pressure,
        })

        self.last_flap_angle = self.flap_angle

        return {
            "sensor": "flap",
            "phase": phase,
            "flap_angle": noisy_angle,
            "motor_current": noisy_motor,
            "hydraulic_pressure": noisy_pressure,
            "status": status
        }
    
    def simulate_motor_current_improved(self, phase, is_moving, angle_error):
        if self.is_locked:
            return 0.0
        
        base_current = {
            'takeoff': 2.0,
            'landing': 2.5,
            'cruise': 1.5
        }.get(phase, 0.0)
        
        if is_moving:
            velocity_factor = min(abs(self.actuator_velocity) / 3.0, 1.0)
            
            position_factor = 1.0 + 0.3 * (self.flap_angle / 30.0)
            
            error_factor = 1.0 + min(abs(angle_error) / 15.0, 0.5)
            
            target_current = base_current * velocity_factor * position_factor * error_factor
            
            if self.hydraulic_pressure > 3450:
                target_current *= 0.9
            
            if abs(self.actuator_velocity) < 1.0:
                target_current *= (abs(self.actuator_velocity) / 1.0) * 0.8 + 0.2
            
            if hasattr(self, 'previous_current'):
                smoothing_factor = 0.7
                current = smoothing_factor * target_current + (1 - smoothing_factor) * self.previous_current
            else:
                current = target_current
            
            current += random.uniform(-0.05, 0.05)
            
            self.previous_current = current
            return max(0.0, current)
        else:
            if not self.is_locked:
                if abs(angle_error) < 0.3:
                    decay_factor = max(0.05, 1.0 - (self.hold_pressure_timer / 0.5))
                else:
                    decay_factor = max(0.1, 1.0 - (self.time_since_last_movement / 1.0))
                
                target_current = base_current * 0.15 * decay_factor
                
                if phase == 'cruise':
                    min_current = 0.15
                    target_current = max(target_current, min_current)
                
                if hasattr(self, 'previous_current'):
                    current = 0.8 * target_current + 0.2 * self.previous_current
                else:
                    current = target_current
                
                self.previous_current = current
                return max(0.0, current)
            else:
                self.previous_current = 0.0
                return 0.0

    def simulate_hydraulic_pressure_improved(self, phase, is_moving):
        if self.is_locked:
            return 0.0
        
        base_pressure = {
            'takeoff': 2700,
            'landing': 3000,
            'cruise': 2500
        }.get(phase, 0.0)
        
        PRESSURE_RELIEF_THRESHOLD = 3450.0
        MAX_PRESSURE = 3500.0
        
        if is_moving:
            velocity_factor = 0.8 + 0.4 * min(abs(self.actuator_velocity) / 3.0, 1.0)
            
            position_factor = 1.0 + 0.2 * (self.flap_angle / 30.0)
            
            target_pressure = base_pressure * velocity_factor * position_factor
            
            if target_pressure > PRESSURE_RELIEF_THRESHOLD:
                excess = target_pressure - PRESSURE_RELIEF_THRESHOLD
                target_pressure = PRESSURE_RELIEF_THRESHOLD + (excess * 0.3)
            
            target_pressure = min(target_pressure, MAX_PRESSURE)
            
            if hasattr(self, 'previous_pressure'):
                smoothing_factor = 0.6
                pressure = smoothing_factor * target_pressure + (1 - smoothing_factor) * self.previous_pressure
            else:
                pressure = target_pressure
            
            pressure += random.uniform(-30, 30)
            
            self.previous_pressure = pressure
            return max(0.0, pressure)
        else:
            if not self.is_locked:
                angle_error = abs(self.target_angle - self.flap_angle)
                
                if angle_error < 0.3:
                    bleed_factor = max(0.3, 1.0 - (self.hold_pressure_timer / 0.4))
                else:
                    bleed_factor = max(0.4, 1.0 - (self.time_since_last_movement / 1.0))
                
                target_pressure = base_pressure * bleed_factor
                
                if phase == 'cruise':
                    min_pressure = 300
                    target_pressure = max(target_pressure, min_pressure)
                
                target_pressure = min(target_pressure, PRESSURE_RELIEF_THRESHOLD)
                
                if hasattr(self, 'previous_pressure'):
                    pressure = 0.7 * target_pressure + 0.3 * self.previous_pressure
                else:
                    pressure = target_pressure
                
                self.previous_pressure = pressure
                return max(0.0, pressure)
            else:
                self.previous_pressure = 0.0
                return 0.0

    def get_diagnostics(self):
        return {
            "flap_angle": self.flap_angle,
            "target_angle": self.target_angle,
            "velocity": self.actuator_velocity,
            "motor_current": self.motor_current,
            "hydraulic_pressure": self.hydraulic_pressure,
            "is_locked": self.is_locked,
            "time_since_last_movement": self.time_since_last_movement
        }