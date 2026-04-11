import time
import random
import math

class AccelerometerFaultInjector:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.fault_active = False
        self.fault_start_time = None
        self.fault_trigger_time = None
        self.flight_start_time = time.time()
        self.fault_was_triggered = False
        
        # New attributes for stopping publication
        self.should_stop_publishing = False
        self.stop_publishing_triggered = False
        self.cruise_phase_ended = False
        self.last_known_phase = None
        self.disable_shutdown = True
        
        self.bias_x_target = 0.0
        self.bias_y_target = 0.0
        self.bias_z_target = 0.0
        
        self.bias_x_current = 0.0
        self.bias_y_current = 0.0
        self.bias_z_current = 0.0
        
        self.drift_rate_x = 0.0
        self.drift_rate_y = 0.0
        self.drift_rate_z = 0.0
        
        self.progressive_drift_x = 0.0
        self.progressive_drift_y = 0.0
        self.progressive_drift_z = 0.0
        
        self.drift_duration = 15.0
        self.fault_configured = False
        
        pass
    
    def configure_fault(self):
        if self.fault_configured:
            return
            
        axes_to_affect = ['x', 'y', 'z']
        
        current_phase = self.phase_controller.get_current_phase()
        
        if current_phase == 'takeoff':
            self.bias_x_target = random.uniform(0.10, 0.20)
            self.bias_y_target = random.uniform(0.05, 0.10)
            self.bias_z_target = random.uniform(0.15, 0.30)
            
        elif current_phase == 'cruise':
            self.bias_x_target = random.uniform(0.12, 0.22)
            self.bias_y_target = random.uniform(0.05, 0.10)
            self.bias_z_target = random.uniform(-0.06, -0.09)
            
        elif current_phase == 'landing':
            self.bias_x_target = random.uniform(0.10, 0.25)
            self.bias_y_target = random.uniform(0.30, 0.60)
            self.bias_z_target = random.uniform(0.30, 0.30)
            
        else:
            self.bias_x_target = random.uniform(0.08, 0.18)
            self.bias_y_target = random.uniform(0.05, 0.12)
            self.bias_z_target = random.uniform(0.10, 0.20)
        
        self.drift_rate_x = self.bias_x_target / self.drift_duration
        self.drift_rate_y = self.bias_y_target / self.drift_duration
        self.drift_rate_z = self.bias_z_target / self.drift_duration
        
        self.progressive_drift_x = random.uniform(0.008, 0.015)
        self.progressive_drift_y = random.uniform(0.005, 0.010)
        self.progressive_drift_z = random.uniform(0.006, 0.012)
        
        self.fault_configured = True
    
    def trigger_fault(self):
        if self.fault_active:
            return
            
        current_phase = self.phase_controller.get_current_phase()
        
        if current_phase == 'takeoff':
            delay = random.uniform(1.0, 2.0)
        elif current_phase == 'cruise':
            delay = random.uniform(0.0, 2.0)
        elif current_phase == 'landing':
            delay = 0.0
        else:
            delay = random.uniform(0.5, 1.5)
        
        self.fault_trigger_time = time.time() + delay
        self.fault_active = True
        self.fault_was_triggered = True
        print(f"[ACCELEROMETER_FAULT] Fault triggered, will activate in {delay:.1f}s")
    
    def check_phase_transition(self):
        """Check if cruise phase has ended and trigger stop publishing"""
        current_phase = self.phase_controller.get_current_phase()
        
        # Track phase transitions
        if self.last_known_phase != current_phase:
            print(f"[ACCELEROMETER_FAULT] Phase transition: {self.last_known_phase} -> {current_phase}")
            
            # If we just left cruise phase, prepare to stop publishing
            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print(f"[ACCELEROMETER_FAULT] Cruise phase ended, will stop publishing soon")
                
                # Add a small delay before stopping (1-2 seconds into landing phase)
                self.stop_publishing_trigger_time = time.time() + random.uniform(1.0, 2.0)
                
            self.last_known_phase = current_phase
        
        # Check if it's time to stop publishing
        if (self.cruise_phase_ended and 
            not self.stop_publishing_triggered and 
            hasattr(self, 'stop_publishing_trigger_time') and 
            time.time() >= self.stop_publishing_trigger_time):
            
            self.should_stop_publishing = False
            self.stop_publishing_triggered = True
            print(f"[ACCELEROMETER_FAULT] 🚨 SENSOR FAILURE: Accelerometer stopped publishing!")
    
    def update_bias_drift(self):
        if not self.fault_active or self.fault_trigger_time is None:
            return
            
        current_time = time.time()
        
        if current_time < self.fault_trigger_time:
            return
            
        if self.fault_start_time is None:
            self.fault_start_time = current_time
            self.configure_fault()
        
        elapsed_time = current_time - self.fault_start_time
        flight_elapsed = current_time - self.flight_start_time
        
        progressive_x = self.progressive_drift_x * flight_elapsed
        progressive_y = self.progressive_drift_y * flight_elapsed
        progressive_z = self.progressive_drift_z * flight_elapsed
        
        if elapsed_time < self.drift_duration:
            immediate_x = self.drift_rate_x * elapsed_time
            immediate_y = self.drift_rate_y * elapsed_time
            immediate_z = self.drift_rate_z * elapsed_time
        else:
            immediate_x = self.bias_x_target
            immediate_y = self.bias_y_target
            immediate_z = self.bias_z_target
        
        self.bias_x_current = immediate_x + progressive_x
        self.bias_y_current = immediate_y + progressive_y
        self.bias_z_current = immediate_z + progressive_z
        
        self.bias_x_current = max(-0.5, min(0.5, self.bias_x_current))
        self.bias_y_current = max(-0.5, min(0.5, self.bias_y_current))
        self.bias_z_current = max(-0.5, min(0.5, self.bias_z_current))
    
    def inject_fault(self, ax, ay, az, freq, rms, phase):
        # Check for phase transitions and potential stop publishing
        self.check_phase_transition()
        
        # If we should stop publishing, return None to indicate no data
        if self.should_stop_publishing and not self.disable_shutdown:
            pass
        
        if not self.fault_active:
            return ax, ay, az, freq, rms
        
        self.update_bias_drift()
        
        faulty_ax = ax + self.bias_x_current
        faulty_ay = ay + self.bias_y_current
        faulty_az = az + self.bias_z_current
        
        original_rms_squared = rms ** 2
        bias_magnitude = math.sqrt(self.bias_x_current ** 2 + 
                                 self.bias_y_current ** 2 + 
                                 self.bias_z_current ** 2)
        
        if phase == 'takeoff':
            rms_multiplier = 0.5
        elif phase == 'cruise':
            rms_multiplier = 0.8
        elif phase == 'landing':
            rms_multiplier = 0.6
        else:
            rms_multiplier = 0.4
        
        bias_contribution = bias_magnitude * rms_multiplier
        faulty_rms = math.sqrt(original_rms_squared + bias_contribution ** 2)
        
        if phase == 'takeoff':
            faulty_rms = max(faulty_rms, rms + 0.06)
        elif phase == 'cruise':
            faulty_rms = max(faulty_rms, rms + 0.05)
        elif phase == 'landing':
            faulty_rms = max(faulty_rms, rms + 0.15)
        
        faulty_freq = freq
        
        faulty_ax = max(-2.0, min(2.0, faulty_ax))
        faulty_ay = max(-2.0, min(2.0, faulty_ay))
        faulty_az = max(-2.5, min(2.5, faulty_az))
        faulty_rms = max(0.001, min(1.5, faulty_rms))
        
        faulty_ax = round(faulty_ax, 3)
        faulty_ay = round(faulty_ay, 3)
        faulty_az = round(faulty_az, 3)
        faulty_freq = round(faulty_freq, 2)
        faulty_rms = round(faulty_rms, 3)
        
        return faulty_ax, faulty_ay, faulty_az, faulty_freq, faulty_rms
    
    def is_fault_active(self):
        return self.fault_active and self.fault_trigger_time is not None and time.time() >= self.fault_trigger_time
    
    def is_publishing_stopped(self):
        """Check if the sensor should stop publishing"""
        return self.should_stop_publishing
    
    def get_fault_status(self):
        if not self.fault_active:
            return {
                'fault_type': 'enhanced_bias_drift',
                'active': False,
                'bias_x': 0.0,
                'bias_y': 0.0,
                'bias_z': 0.0,
                'elapsed_time': 0.0,
                'flight_time': time.time() - self.flight_start_time,
                'sensor_name': 'accelerometer',
                'publishing_stopped': self.should_stop_publishing,
                'cruise_phase_ended': self.cruise_phase_ended
            }
        
        elapsed_time = 0.0
        if self.fault_start_time is not None:
            elapsed_time = time.time() - self.fault_start_time
        
        return {
            'fault_type': 'enhanced_bias_drift',
            'active': self.is_fault_active(),
            'bias_x': self.bias_x_current,
            'bias_y': self.bias_y_current,
            'bias_z': self.bias_z_current,
            'bias_x_target': self.bias_x_target,
            'bias_y_target': self.bias_y_target,
            'bias_z_target': self.bias_z_target,
            'progressive_drift_x': self.progressive_drift_x,
            'progressive_drift_y': self.progressive_drift_y,
            'progressive_drift_z': self.progressive_drift_z,
            'elapsed_time': elapsed_time,
            'flight_time': time.time() - self.flight_start_time,
            'drift_duration': self.drift_duration,
            'sensor_name': 'accelerometer',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


class AccelerometerFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'enhanced_bias_drift': AccelerometerFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False
        
        pass
    
    def trigger_random_fault(self):
        fault_type = 'enhanced_bias_drift'
        
        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
        else:
            pass
    
    def inject_fault(self, ax, ay, az, freq, rms, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(ax, ay, az, freq, rms, phase)            
            return result
        else:
            return ax, ay, az, freq, rms
    
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
                'sensor_name': 'accelerometer',
                'publishing_stopped': False,
                'cruise_phase_ended': False
            }


def update_accelerometer_with_fault_injection(accel, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()
    accel.reading_counter += 1

    ax, ay, az, freq, rms = accel.generate_readings(current_phase)
    
    fault_result = fault_manager.inject_fault(ax, ay, az, freq, rms, current_phase)
    
    # Handle normal fault injection result
    if fault_result and fault_result[0] is not None:
        faulty_ax, faulty_ay, faulty_az, faulty_freq, faulty_rms = fault_result
        
        if fault_manager.is_fault_active():
            output_ax, output_ay, output_az = faulty_ax, faulty_ay, faulty_az
            output_freq, output_rms = faulty_freq, faulty_rms
        else:
            output_ax, output_ay, output_az = ax, ay, az
            output_freq, output_rms = freq, rms
    else:
        output_ax, output_ay, output_az = ax, ay, az
        output_freq, output_rms = freq, rms
    # Normal publishing
    bus.publish("wing/accelerometer", {
        "sensor": "accelerometer",
        "phase": current_phase,
        "accel_x (g)": output_ax,
        "accel_y (g)": output_ay,
        "accel_z (g)": output_az,
        "dominant_freq (Hz)": output_freq,
        "rms (g)": output_rms,
        "timestamp": time.time()
    })
    
    return {
    "sensor": "accelerometer",
    "phase": current_phase,
    "accel_x (g)": output_ax,
    "accel_y (g)": output_ay,
    "accel_z (g)": output_az,
    "dominant_freq (Hz)": output_freq,
    "rms (g)": output_rms,
}