import time
import random

class SpoilerSensor:
    def __init__(self, bus, phase_controller):
        self.bus = bus
        self.phase_controller = phase_controller
        self.commanded_angle = 0.0
        self.actual_angle = 0.0
        self.motor_current = 0.0
        self.last_time = time.time()
        self.command_applied_time = 0.0
        self.deployment_lag = 0
        self.last_phase = None
        self.stable_command = 0.0
        
        self.landing_entry_count = 0
        self.is_first_landing_entry = False
        
        self.max_lag_tolerance = {
            'cruise': 100,
            'landing': 200
        }
        self.fault_flags = {
            'excessive_lag': False,
            'actuator_stall': False,
            'signal_miss': False
        }
        self.command_history = []
        self.stall_detection_time = 0.0
        self.stall_timeout = 2.0
        
        self.cruise_actuation_enabled = True
        self.cruise_suppression_active = False
        self.cruise_min_threshold = 0.5

    def simulate_commanded_angle(self, phase):
        if phase == 'landing':
            if self.stable_command == 0.0:
                self.stable_command = round(random.uniform(30.0, 60.0), 2)
            return self.stable_command
        elif phase == 'cruise':
            if random.random() < 0.08:
                angle = round(random.uniform(0.0, 3.0), 2)
                if angle > 0.0 and random.random() < 0.3:
                    self.cruise_suppression_active = True
                    print(f"[CRUISE SUPPRESSION] Command {angle:.2f}° suppressed by flight control")
                    return 0.0
                else:
                    self.cruise_suppression_active = False
                return angle
        else:
            self.stable_command = 0.0
        return 0.0

    def simulate_deployment_lag(self, phase):
        base_lag = 0
        
        if phase == 'landing':
            if self.is_first_landing_entry:
                base_lag = random.randint(20, 80)
                print(f"[FIRST LANDING] Optimized lag: {base_lag}ms")
            else:
                base_lag = random.randint(50, 150)
        elif phase == 'cruise':
            base_lag = random.randint(15, 45)
        
        if not self.is_first_landing_entry and random.random() < 0.02:
            base_lag *= 2
            self.fault_flags['excessive_lag'] = True
            print(f"[FAULT] Excessive lag detected: {base_lag}ms")
        
        return base_lag

    def detect_actuator_faults(self, phase, now):
        if (self.commanded_angle > 0.1 and 
            abs(self.commanded_angle - self.actual_angle) > 2.0 and
            self.motor_current == 0.0 and
            self.command_applied_time <= now):
            
            if self.stall_detection_time == 0.0:
                self.stall_detection_time = now
            elif now - self.stall_detection_time > self.stall_timeout:
                self.fault_flags['actuator_stall'] = True
                print(f"[FAULT] Actuator stall detected - No movement for {self.stall_timeout}s")
        else:
            self.stall_detection_time = 0.0
            self.fault_flags['actuator_stall'] = False

        if (self.commanded_angle > 0.1 and 
            self.deployment_lag == 0 and 
            self.command_applied_time <= now):
            self.fault_flags['signal_miss'] = True
            print(f"[FAULT] Signal miss detected - Command without proper lag")

        if self.deployment_lag > self.max_lag_tolerance.get(phase, 100):
            self.fault_flags['excessive_lag'] = True
            print(f"[FAULT] Excessive lag: {self.deployment_lag}ms > {self.max_lag_tolerance.get(phase, 100)}ms")

    def update_spoiler(self):
        dt = 1.0
        now = time.time()
        phase = self.phase_controller.get_current_phase()

        if phase != self.last_phase:
            self.last_phase = phase
            self.deployment_lag = 0
            self.fault_flags = {key: False for key in self.fault_flags}
            self.cruise_suppression_active = False
            
            if phase == 'landing':
                self.landing_entry_count = 0
                self.is_first_landing_entry = True
                print(f"[PHASE CHANGE] Switched to LANDING - First entry tracking enabled")
            else:
                self.landing_entry_count = 0
                self.is_first_landing_entry = False
                if phase != 'landing':
                    self.stable_command = 0.0
                print(f"[PHASE CHANGE] Switched to {phase.upper()}")

        if phase == 'landing':
            self.landing_entry_count += 1
            if self.landing_entry_count > 1:
                self.is_first_landing_entry = False

        if phase == 'takeoff':
            return self._reset_takeoff(now)
            

        new_command = self.simulate_commanded_angle(phase)
        
        command_changed = abs(new_command - self.commanded_angle) > 0.1
        if command_changed:
            self.command_history.append({
                'timestamp': now,
                'command': new_command,
                'previous': self.commanded_angle
            })
            
            if len(self.command_history) > 10:
                self.command_history.pop(0)
            
            self.commanded_angle = new_command
            
            if self.commanded_angle > 0.0:
                self.deployment_lag = self.simulate_deployment_lag(phase)
                self.command_applied_time = now + self.deployment_lag / 1000.0
                
                if self.is_first_landing_entry:
                    print(f"[FIRST LANDING COMMAND] {self.commanded_angle:.2f}°, Lag: {self.deployment_lag} ms")
                else:
                    print(f"[NEW COMMAND] {self.commanded_angle:.2f}°, Lag: {self.deployment_lag} ms")
            else:
                self.deployment_lag = 0
                self.command_applied_time = now

        self.detect_actuator_faults(phase, now)

        if self.is_first_landing_entry and phase == 'landing':
            return self._handle_first_landing_entry(now)

        # Fixed: In cruise phase, don't process commands with lag - wait for them to be 0
        if phase == 'cruise' and self.command_applied_time > now and self.commanded_angle > 0.0:
            # For cruise phase, reset the command to 0 and don't process it until lag expires
            self.commanded_angle = 0.0
            self.deployment_lag = 0
            self.command_applied_time = 0.0
            
            # Set everything to idle state
            self.motor_current = 0.0
            self.actual_angle = 0.0
            
            return self._log_and_publish(phase, now, "idle")
            
        if self.command_applied_time > now and self.commanded_angle > 0.0:
            lag_remaining = self.command_applied_time - now
            self.motor_current = 0.0
            
            if lag_remaining > 0:
                print(f"[WAITING] Remaining lag: {lag_remaining:.3f}s")
                return self._log_and_publish(phase, now, f"waiting ({int(lag_remaining*1000)} ms)")

        error = self.commanded_angle - self.actual_angle
        max_rate = 50.0 if phase == 'landing' else 25.0
        max_step = max_rate * dt

        noise = 0.0
        if self.commanded_angle > 0.0:
            if phase == 'landing':
                noise = random.gauss(0, 0.15)
            elif phase == 'cruise':
                noise = random.gauss(0, 0.05)

        if self.commanded_angle > 0.0 and abs(error) > 0.02:
            overshoot = self.actual_angle - self.commanded_angle
            
            if overshoot > 0.5 and phase == 'landing':
                correction_step = min(max_step * 0.8, overshoot * 0.6)
                self.actual_angle -= correction_step
                self._update_motor_current(phase, abs(error))
                status = f"overshoot_correction (overshoot: {overshoot:.2f}°)"
                print(f"[LANDING] Overshoot correction: {overshoot:.2f}° > 0.5°, correcting by {correction_step:.2f}°")
            else:
                step = max(-max_step, min(error, max_step))
                
                if phase == 'landing':
                    predicted_angle = self.actual_angle + step + noise
                    overshoot_limit = 0.4
                    
                    if predicted_angle > self.commanded_angle + overshoot_limit:
                        max_allowed = self.commanded_angle + overshoot_limit - self.actual_angle
                        step = max(0, max_allowed * 0.7)
                        noise = random.gauss(0, 0.1)
                        print(f"[LANDING] Overshoot prevention: limiting step to {step:.2f}°")
                    
                    if predicted_angle < self.commanded_angle - overshoot_limit:
                        min_allowed = self.commanded_angle - overshoot_limit - self.actual_angle
                        step = min(0, min_allowed * 0.7)
                        noise = random.gauss(0, 0.1)
                        print(f"[LANDING] Undershoot prevention: limiting step to {step:.2f}°")
                
                self.actual_angle += step + noise
                self._update_motor_current(phase, abs(error))
                
                status = "actuating"
                if phase == 'cruise':
                    status = f"cruise_actuating (target: {self.commanded_angle:.2f}°)"
                
        elif self.commanded_angle == 0.0 and self.actual_angle > 0.02:
            correction = min(max_step, self.actual_angle)
            self.actual_angle -= correction
            self.motor_current = 0.15 if phase == 'cruise' else 0.1
            status = "retracting"
        else:
            if self.commanded_angle > 0.0:
                self.motor_current = 0.08 if phase == 'cruise' else 0.1
                status = "holding"
            else:
                self.motor_current = 0.0
                status = "idle"

        self.actual_angle = round(max(0.0, min(self.actual_angle, 60.0)), 2)
        
        if phase == 'landing':
            overshoot = self.actual_angle - self.commanded_angle
            if abs(overshoot) > 0.5:
                if overshoot > 0.5:
                    self.actual_angle = self.commanded_angle + 0.5
                elif overshoot < -0.5:
                    self.actual_angle = self.commanded_angle - 0.5
                print(f"[LANDING] Hard limit applied: clamped to {self.actual_angle:.2f}°")
        
        if self.commanded_angle == 0.0 and abs(self.actual_angle) < 0.02:
            self.actual_angle = 0.0

        return self._log_and_publish(phase, now, status)

    def _update_motor_current(self, phase, error):
        overshoot = self.actual_angle - self.commanded_angle
        is_overshooting = overshoot > 0.5
        
        if phase == 'landing':
            if self.is_first_landing_entry:
                base_current = random.uniform(1.5, 3.5)
                noise = random.gauss(0, 0.2)
                self.motor_current = max(1.5, base_current + noise)
            else:
                if is_overshooting:
                    base_current = random.uniform(0.5, 1.5)
                    noise = random.gauss(0, 0.1)
                    print(f"[LANDING] Overshoot protection: reducing current to {base_current:.2f}A")
                else:
                    base_current = random.uniform(1.5, 4.5)
                    noise = random.gauss(0, 0.25)
                self.motor_current = max(0.0, base_current + noise)
        elif phase == 'cruise':
            if self.commanded_angle > 0.02:
                base_current = random.uniform(0.2, 0.8)
                noise = random.gauss(0, 0.05)
                self.motor_current = max(0.15, base_current + noise)
            else:
                base_current = random.uniform(0.1, 0.5)
                noise = random.gauss(0, 0.1)
                self.motor_current = max(0.0, base_current + noise)
        else:
            self.motor_current = 0.0
        
        if error < 0.1 and self.commanded_angle > 0.02:
            self.motor_current = max(self.motor_current * 0.8, 0.1)
        elif error < 0.1:
            self.motor_current *= 0.6
        
        if is_overshooting and phase == 'landing':
            self.motor_current = min(self.motor_current, 2.0)
        
        if (not self.is_first_landing_entry and 
            not (phase == 'cruise' and self.commanded_angle > 0.02) and
            random.random() < 0.005):
            self.motor_current = 0.0
            print(f"[FAULT] Motor current fault simulated")
        
        self.motor_current = round(self.motor_current, 2)

    def _reset_takeoff(self, now):
        self.commanded_angle = 0.0
        self.actual_angle = 0.0
        self.motor_current = 0.0
        self.deployment_lag = 0
        self.stable_command = 0.0
        self.command_applied_time = 0.0
        self.fault_flags = {key: False for key in self.fault_flags}
        self.landing_entry_count = 0
        self.is_first_landing_entry = False
        self.cruise_suppression_active = False
        return self._log_and_publish('takeoff', now, "locked")

    def _handle_first_landing_entry(self, now):
        if self.command_applied_time > now:
            lag_remaining = self.command_applied_time - now
            
            if self.commanded_angle > 0 and self.actual_angle == 0:
                initial_movement = random.uniform(2.0, 8.0)
                self.actual_angle = round(initial_movement, 2)
                
                self.motor_current = round(random.uniform(1.5, 3.5), 2)
                
                print(f"[FIRST LANDING] Initial movement: {self.actual_angle:.2f}°, Motor: {self.motor_current:.2f}A")
            
            return self._log_and_publish('landing', now, f"first_entry_lag ({int(lag_remaining*1000)} ms)")
        
        error = self.commanded_angle - self.actual_angle
        max_rate = 50.0
        max_step = max_rate * 1.0
        
        noise = random.gauss(0, 0.3)
        
        if abs(error) > 0.05:
            step = max(-max_step, min(error, max_step))
            self.actual_angle += step + noise
            self._update_motor_current('landing', abs(error))
        else:
            self.motor_current = 0.0
        
        self.actual_angle = round(max(0.0, min(self.actual_angle, 60.0)), 2)
        
        return self._log_and_publish('landing', now, "first_entry_actuating")

    def _log_and_publish(self, phase, timestamp, status=None):
        tracking_error = round(self.commanded_angle - self.actual_angle, 2)
        
        status_str = status if status else "ready"
        fault_indicators = [k for k, v in self.fault_flags.items() if v]
        if fault_indicators:
            status_str += f" [FAULTS: {', '.join(fault_indicators)}]"
        
        if self.cruise_suppression_active and phase == 'cruise':
            status_str += " [SUPPRESSED]"
        
        print(
            f"[{phase.upper()}] Command: {self.commanded_angle:.2f}°, "
            f"Actual: {self.actual_angle:.2f}°, "
            f"Error: {tracking_error:.2f}°, "
            f"Motor: {self.motor_current:.2f} A, {status_str}"
        )

        current_lag = 0
        if hasattr(self, 'command_applied_time') and self.command_applied_time > timestamp:
            current_lag = int((self.command_applied_time - timestamp) * 1000)

        self.bus.publish("wing/spoiler", {
            "sensor": "spoiler",
            "timestamp": timestamp,
            "phase": phase,
            "commanded_angle": self.commanded_angle,
            "actual_angle": self.actual_angle,
            "error": tracking_error,
            "motor_current": self.motor_current,
            "lag_ms": current_lag,
        })

        return {
            "sensor": "spoiler",
            "phase": phase,
            "commanded_angle_deg": self.commanded_angle,
            "actual_angle_deg": self.actual_angle,
            "tracking_error_deg": tracking_error,
            "motor_current_A": self.motor_current,
            "lag_ms": current_lag,
            "status": status_str
        }

    def get_health_status(self):
        if any(self.fault_flags.values()):
            return {
                "status": "degraded",
                "faults": [k for k, v in self.fault_flags.items() if v],
                "recommendation": "Schedule maintenance check"
            }
        return {"status": "normal", "faults": [], "recommendation": "Continue normal operation"}

    def force_command(self, angle):
        self.commanded_angle = angle
        self.deployment_lag = self.simulate_deployment_lag(self.phase_controller.get_current_phase())
        self.command_applied_time = time.time() + self.deployment_lag / 1000.0
        print(f"[FORCE COMMAND] Forced command to {angle:.2f}°")

    def inject_fault(self, fault_type):
        if fault_type in self.fault_flags:
            self.fault_flags[fault_type] = True
            print(f"[FAULT INJECTED] {fault_type}")
        else:
            print(f"[ERROR] Unknown fault type: {fault_type}")

    def clear_faults(self):
        self.fault_flags = {key: False for key in self.fault_flags}
        print("[CLEARED] All faults cleared")

    def enable_cruise_actuation(self, enabled=True):
        self.cruise_actuation_enabled = enabled
        print(f"[CRUISE] Actuation {'enabled' if enabled else 'disabled'}")

    def set_cruise_suppression(self, active=True):
        self.cruise_suppression_active = active
        print(f"[CRUISE] Suppression {'active' if active else 'inactive'}")