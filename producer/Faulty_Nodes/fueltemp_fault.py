import time
import random
import math

class FuelTempFaultInjector:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.fault_active = False
        self.fault_start_time = None
        self.fault_trigger_time = None
        self.flight_start_time = time.time()
        self.fault_was_triggered = False
        
        self.should_stop_publishing = False
        self.stop_publishing_triggered = False
        self.cruise_phase_ended = False
        self.last_known_phase = None
        
        self.temperature_offset = 0.0
        self.thermal_runaway_completed = False
        self.runaway_duration = 8.0
        self.plateau_duration = 0.0
        self.decay_start_time = None
        
        self.gradient_multiplier = 1.0
        self.jitter_multiplier = 1.0
        self.base_jitter_offset = 0.0
        
        self.phase_targets = {
            'takeoff': {'peak': 0.0, 'plateau': 0.0, 'decay_rate': 0.0},
            'cruise': {'peak': 0.0, 'plateau': 0.0, 'decay_rate': 0.0},
            'landing': {'peak': 0.0, 'plateau': 0.0, 'decay_rate': 0.0}
        }
        
        self.fault_configured = False
        self.heating_phase = 'inactive'
        
    def configure_fault(self):
        if self.fault_configured:
            return
            
        current_phase = self.phase_controller.get_current_phase()
        
        if current_phase == 'takeoff':
            self.phase_targets['takeoff'] = {
                'peak': random.uniform(45.0, 55.0),
                'plateau': random.uniform(40.0, 50.0),
                'decay_rate': random.uniform(0.8, 1.2)
            }
            self.phase_targets['cruise'] = {
                'peak': random.uniform(50.0, 60.0),
                'plateau': random.uniform(45.0, 55.0),
                'decay_rate': random.uniform(0.3, 0.5)
            }
            self.phase_targets['landing'] = {
                'peak': random.uniform(35.0, 45.0),
                'plateau': random.uniform(35.0, 42.0),
                'decay_rate': random.uniform(1.0, 1.5)
            }
            
        elif current_phase == 'cruise':
            self.phase_targets['cruise'] = {
                'peak': random.uniform(55.0, 65.0),
                'plateau': random.uniform(50.0, 60.0),
                'decay_rate': random.uniform(0.2, 0.4)
            }
            self.phase_targets['landing'] = {
                'peak': random.uniform(40.0, 50.0),
                'plateau': random.uniform(38.0, 45.0),
                'decay_rate': random.uniform(0.8, 1.2)
            }
            
        elif current_phase == 'landing':
            self.phase_targets['landing'] = {
                'peak': random.uniform(48.0, 58.0),
                'plateau': random.uniform(45.0, 55.0),
                'decay_rate': random.uniform(0.5, 0.8)
            }
        
        self.fault_configured = True
    
    def trigger_fault(self):
        if self.fault_active:
            return
            
        current_phase = self.phase_controller.get_current_phase()
        
        if current_phase == 'takeoff':
            delay = random.uniform(3.0, 7.0)
        elif current_phase == 'cruise':
            delay = random.uniform(1.0, 3.0)
        elif current_phase == 'landing':
            delay = 0.0
        else:
            delay = random.uniform(2.0, 5.0)
        
        self.fault_trigger_time = time.time() + delay
        self.fault_active = True
        self.fault_was_triggered = True
        
        print(f"🔥 [THERMAL FAULT] Thermal runaway will begin in {delay:.1f}s during {current_phase}")
    
    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()
        
        if self.last_known_phase != current_phase:
            
            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print(f"🔥 [FUELTEMP_FAULT] Cruise phase ended, sensor will fail during landing")
                
                self.stop_publishing_trigger_time = time.time() + random.uniform(2.0, 4.0)
                
            self.last_known_phase = current_phase
        
        if (self.cruise_phase_ended and 
            not self.stop_publishing_triggered and 
            hasattr(self, 'stop_publishing_trigger_time') and 
            time.time() >= self.stop_publishing_trigger_time):
            
            self.should_stop_publishing = False
            self.stop_publishing_triggered = True
            print(f"🔥 [FUELTEMP_FAULT] 🚨 CRITICAL: Fuel temperature sensor failing now!")
    
    def get_current_phase_target(self, phase):
        if phase in self.phase_targets:
            return self.phase_targets[phase]
        return {'peak': 35.0, 'plateau': 30.0, 'decay_rate': 1.0}
    
    def update_thermal_runaway(self):
        if not self.fault_active or self.fault_trigger_time is None:
            return
            
        current_time = time.time()
        
        if current_time < self.fault_trigger_time:
            return
            
        if self.fault_start_time is None:
            self.fault_start_time = current_time
            self.configure_fault()
            self.heating_phase = 'heating'
            print(f"🔥 [THERMAL FAULT] Thermal runaway INITIATED!")
        
        elapsed_time = current_time - self.fault_start_time
        current_phase = self.phase_controller.get_current_phase()
        phase_target = self.get_current_phase_target(current_phase)
        
        if self.heating_phase == 'heating':
            if elapsed_time < self.runaway_duration:
                progress = elapsed_time / self.runaway_duration
                exponential_progress = 1 - math.exp(-3 * progress)
                
                target_temp = phase_target['peak']
                self.temperature_offset = target_temp * exponential_progress
                
                if current_phase == 'takeoff':
                    self.gradient_multiplier = 2.0 + (18.0 * exponential_progress)
                    self.jitter_multiplier = 1.5 + (3.5 * exponential_progress)
                    
                elif current_phase == 'cruise':
                    self.gradient_multiplier = 1.5 + (23.5 * exponential_progress)
                    self.jitter_multiplier = 1.2 + (2.8 * exponential_progress)
                    
                elif current_phase == 'landing':
                    self.gradient_multiplier = 3.0 + (17.0 * exponential_progress)
                    self.jitter_multiplier = 1.8 + (2.7 * exponential_progress)
                      
            else:
                self.heating_phase = 'plateau'
                self.plateau_duration = random.uniform(10.0, 20.0)
                self.temperature_offset = phase_target['plateau']
                
        elif self.heating_phase == 'plateau':
            plateau_elapsed = elapsed_time - self.runaway_duration
            
            if plateau_elapsed < self.plateau_duration:
                self.temperature_offset = phase_target['plateau'] + random.uniform(-1.0, 1.0)
                
                self.gradient_multiplier = random.uniform(0.1, 0.3)
                self.jitter_multiplier = random.uniform(1.2, 1.8)
                          
            else:
                self.heating_phase = 'decay'
                self.decay_start_time = current_time
                print(f"🔥 [THERMAL FAULT] Entering DECAY phase")
                
        elif self.heating_phase == 'decay':
            decay_elapsed = current_time - self.decay_start_time
            decay_rate = phase_target['decay_rate']
            
            decay_factor = math.exp(-decay_rate * decay_elapsed / 10.0)
            self.temperature_offset = phase_target['plateau'] * decay_factor
            
            self.gradient_multiplier = random.uniform(-0.5, -0.1)
            self.jitter_multiplier = random.uniform(1.0, 1.5)
        
        self.base_jitter_offset = random.uniform(0.02, 0.05) * (1 + self.temperature_offset / 50.0)
    
    def inject_fault(self, temperature, gradient, jitter, phase):
        self.check_phase_transition()
        
        if self.should_stop_publishing:
            pass
        
        if not self.fault_active:
            return temperature, gradient, jitter
        
        self.update_thermal_runaway()
        
        faulty_temperature = temperature + self.temperature_offset
        
        if self.heating_phase == 'heating':
            faulty_gradient = gradient * self.gradient_multiplier
            faulty_gradient += random.uniform(8.0, 15.0)
            
        elif self.heating_phase == 'plateau':
            faulty_gradient = gradient * self.gradient_multiplier
            faulty_gradient += random.uniform(-0.5, 0.5)
            
        elif self.heating_phase == 'decay':
            faulty_gradient = gradient * self.gradient_multiplier
            faulty_gradient += random.uniform(-2.0, -0.5)
            
        else:
            faulty_gradient = gradient * self.gradient_multiplier
        
        faulty_jitter = jitter * self.jitter_multiplier + self.base_jitter_offset
        
        if phase == 'takeoff':
            faulty_temperature = max(min(faulty_temperature, 60.0), 8.0)
            faulty_gradient = max(min(faulty_gradient, 120.0), -60.0)
            
        elif phase == 'cruise':
            faulty_temperature = max(min(faulty_temperature, 70.0), -25.0)
            faulty_gradient = max(min(faulty_gradient, 150.0), -40.0)
            
        elif phase == 'landing':
            faulty_temperature = max(min(faulty_temperature, 65.0), -5.0)
            faulty_gradient = max(min(faulty_gradient, 80.0), -30.0)
        
        faulty_jitter = max(min(faulty_jitter, 0.5), 0.01)
        
        faulty_temperature = round(faulty_temperature, 2)
        faulty_gradient = round(faulty_gradient, 2)
        faulty_jitter = round(faulty_jitter, 3)
        
        return faulty_temperature, faulty_gradient, faulty_jitter
    
    def is_fault_active(self):
        return (self.fault_active and 
                self.fault_trigger_time is not None and 
                time.time() >= self.fault_trigger_time)
    
    def is_publishing_stopped(self):
        return self.should_stop_publishing
    
    def get_fault_status(self):
        if not self.fault_active:
            return {
                'fault_type': 'thermal_runaway',
                'active': False,
                'heating_phase': 'inactive',
                'temperature_offset': 0.0,
                'gradient_multiplier': 1.0,
                'jitter_multiplier': 1.0,
                'thermal_runaway_completed': False,
                'elapsed_time': 0.0,
                'flight_time': time.time() - self.flight_start_time,
                'sensor_name': 'fueltemp',
                'publishing_stopped': self.should_stop_publishing,
                'cruise_phase_ended': self.cruise_phase_ended
            }
        
        elapsed_time = 0.0
        if self.fault_start_time is not None:
            elapsed_time = time.time() - self.fault_start_time
        
        return {
            'fault_type': 'thermal_runaway',
            'active': self.is_fault_active(),
            'heating_phase': self.heating_phase,
            'temperature_offset': self.temperature_offset,
            'gradient_multiplier': self.gradient_multiplier,
            'jitter_multiplier': self.jitter_multiplier,
            'thermal_runaway_completed': self.thermal_runaway_completed,
            'phase_targets': self.phase_targets,
            'elapsed_time': elapsed_time,
            'flight_time': time.time() - self.flight_start_time,
            'runaway_duration': self.runaway_duration,
            'plateau_duration': self.plateau_duration,
            'sensor_name': 'fueltemp',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


class FuelTempFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'thermal_runaway': FuelTempFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False
    
    def trigger_random_fault(self):
        fault_type = 'thermal_runaway'
        
        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"🔥 [FAULT MANAGER] Thermal runaway fault triggered!")
    
    def inject_fault(self, temperature, gradient, jitter, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(temperature, gradient, jitter, phase)
            
            return result
        else:
            return temperature, gradient, jitter
    
    def is_fault_active(self):
        return self.current_fault is not None and self.current_fault.is_fault_active()
    
    def is_publishing_stopped(self):
        return self.current_fault is not None and self.current_fault.is_publishing_stopped()
    
    def get_fault_status(self):
        if self.current_fault is not None:
            return self.current_fault.get_fault_status()
        else:
            return {
                'fault_type': 'none',
                'active': False,
                'sensor_name': 'fueltemp',
                'publishing_stopped': False,
                'cruise_phase_ended': False
            }

def update_fueltemp_with_fault_injection(fueltemp_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()
    
    if fault_manager.is_fault_active():
        phase = phase_controller.get_current_phase()
        now = time.time()
        elapsed_time = now - fueltemp_sensor.last_time
        fueltemp_sensor.last_time = now

        if phase != fueltemp_sensor.current_phase:
            next_temp = fueltemp_sensor.init_temperature(phase)
            transition_steps = 25
            
            if fueltemp_sensor.smoothed_temp is not None:
                fueltemp_sensor.transition_buffer = fueltemp_sensor.interpolate_transition(
                    fueltemp_sensor.smoothed_temp, next_temp, phase, transition_steps)
            
            fueltemp_sensor.reading_counter = 0
            fueltemp_sensor.temp_log = []
            fueltemp_sensor.current_phase = phase
            
            if not fueltemp_sensor.transition_buffer:
                fueltemp_sensor.smoothed_temp = next_temp
            fueltemp_sensor.prev_temp = fueltemp_sensor.smoothed_temp

        fueltemp_sensor.reading_counter += 1

        if fueltemp_sensor.transition_buffer:
            temp = fueltemp_sensor.transition_buffer.pop(0)
        else:
            target_min, target_max = fueltemp_sensor.phase_ranges[phase]
            mid_target = (target_min + target_max) / 2
            direction = 1 if mid_target > fueltemp_sensor.smoothed_temp else -1
            
            if phase == 'cruise':
                base_trend = random.uniform(0.0005, 0.002) * direction
                stddev = 0.008
            elif phase == 'takeoff':
                base_trend = random.uniform(0.003, 0.008) * direction
                stddev = 0.02
            else:
                base_trend = random.uniform(0.001, 0.005) * direction
                stddev = 0.015
            
            env_influence = fueltemp_sensor.get_environmental_influence(phase)
            new_temp = fueltemp_sensor.smoothed_temp + base_trend + env_influence
            noisy_temp = fueltemp_sensor.apply_noise(new_temp, stddev)
            inertia_temp = fueltemp_sensor.apply_thermal_inertia(noisy_temp, phase)
            adjusted_temp, gradient = fueltemp_sensor.validate_gradient(
                inertia_temp, fueltemp_sensor.prev_temp, elapsed_time, phase)
            
            temp = round(fueltemp_sensor.alpha * adjusted_temp + (1 - fueltemp_sensor.alpha) * fueltemp_sensor.smoothed_temp, 2)
            temp = fueltemp_sensor.clamp_to_phase_range(temp, phase)

        fueltemp_sensor.thermal_history.append(temp)
        if len(fueltemp_sensor.thermal_history) > fueltemp_sensor.thermal_history_size:
            fueltemp_sensor.thermal_history.pop(0)

        if fueltemp_sensor.temp_log and abs(temp - fueltemp_sensor.temp_log[-1]) < 0.005:
            temp += random.uniform(-0.01, 0.01)
            temp = round(temp, 2)
            temp = fueltemp_sensor.clamp_to_phase_range(temp, phase)

        gradient = 0.0
        if fueltemp_sensor.reading_counter > 1 and fueltemp_sensor.prev_temp is not None:
            gradient = fueltemp_sensor.calculate_gradient(temp, fueltemp_sensor.prev_temp, 1.0, phase)

        fueltemp_sensor.smoothed_temp = temp
        fueltemp_sensor.temp_log.append(temp)
        if len(fueltemp_sensor.temp_log) > 10:
            fueltemp_sensor.temp_log.pop(0)
        fueltemp_sensor.prev_temp = temp

        temp_delta = temp - (fueltemp_sensor.temp_log[-2] if len(fueltemp_sensor.temp_log) > 1 else temp)
        jitter = fueltemp_sensor.get_dynamic_jitter(phase, fueltemp_sensor.reading_counter, temp_delta)

        fault_result = fault_manager.inject_fault(temp, gradient, jitter, phase)
        
        # Handle normal fault injection result
        if fault_result and fault_result[0] is not None:
            faulty_temperature, faulty_gradient, faulty_jitter = fault_result
            
            # Update sensor state with faulty values for consistency
            fueltemp_sensor.smoothed_temp = faulty_temperature
            if fueltemp_sensor.temp_log:
                fueltemp_sensor.temp_log[-1] = faulty_temperature
                
            # Publish ONLY faulty data (single publication)
            bus.publish("wing/fueltemp", {
                "sensor": "fueltemp",
                "phase": current_phase,
                "temperature (Degree C)": faulty_temperature,
                "gradient (Degree C/min)": faulty_gradient,
                "jitter (Degree C)": faulty_jitter,
                "timestamp": time.time()
            })
            
            # Enhanced fault logging
            fault_status = fault_manager.get_fault_status()
            if fault_status['active']:
                phase_info = fault_status['heating_phase'].upper()
                print(f"🔥 [FAULT ACTIVE] {phase_info} - "
                      f"Temp: {faulty_temperature:.1f}°C (+{fault_status['temperature_offset']:.1f}°C), "
                      f"Gradient: {faulty_gradient:.1f}°C/min ({fault_status['gradient_multiplier']:.1f}x)")
        else:
            pass
        
    else:
        # Normal operation - let the sensor update and publish normally
        fueltemp_sensor.update_fueltemp_sensor()
    
    return {
        "sensor": "fueltemp",
        "phase": current_phase,
        "temperature (Degree C)": fueltemp_sensor.smoothed_temp,
        "gradient (Degree C/min)": gradient if 'gradient' in locals() else 0.0,
        "jitter (Degree C)": jitter if 'jitter' in locals() else 0.0
    }   
