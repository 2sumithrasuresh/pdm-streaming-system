import time
import random
import math

class FlutterFaultInjector:
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
        
        self.damping_target_increase = 0.0
        self.damping_current_bias = 0.0
        self.damping_rise_rate = 0.0
        
        self.amplitude_instability_factor = 1.0
        self.frequency_jitter_amplitude = 0.0
        self.frequency_base_drift = 0.0
        
        self.progressive_amplitude_growth = 0.0
        self.progressive_frequency_instability = 0.0
        self.instability_duration = 20.0
        
        self.oscillation_frequency = 0.0
        self.oscillation_amplitude = 0.0
        
        self.fault_configured = False
        
    def configure_fault(self):
        if self.fault_configured:
            return
            
        current_phase = self.phase_controller.get_current_phase()
        
        if current_phase == 'takeoff':
            self.damping_target_increase = random.uniform(0.04, 0.08)
            self.amplitude_instability_factor = random.uniform(1.15, 1.35)
            self.frequency_jitter_amplitude = random.uniform(0.3, 0.6)
            self.frequency_base_drift = random.uniform(0.5, 1.0)
            self.progressive_amplitude_growth = random.uniform(0.08, 0.12)
            self.progressive_frequency_instability = random.uniform(0.02, 0.04)
            
        elif current_phase == 'cruise':
            self.damping_target_increase = random.uniform(0.05, 0.10)
            self.amplitude_instability_factor = random.uniform(1.25, 1.60)
            self.frequency_jitter_amplitude = random.uniform(0.8, 1.3)
            self.frequency_base_drift = random.uniform(0.3, 0.7)
            self.progressive_amplitude_growth = random.uniform(0.06, 0.10)
            self.progressive_frequency_instability = random.uniform(0.03, 0.05)
            
        elif current_phase == 'landing':
            self.damping_target_increase = random.uniform(0.06, 0.12)
            self.amplitude_instability_factor = random.uniform(1.20, 1.50)
            self.frequency_jitter_amplitude = random.uniform(0.4, 0.7)
            self.frequency_base_drift = random.uniform(-0.2, 0.3)
            self.progressive_amplitude_growth = random.uniform(0.10, 0.15)
            self.progressive_frequency_instability = random.uniform(0.025, 0.035)
            
        else:
            self.damping_target_increase = random.uniform(0.03, 0.06)
            self.amplitude_instability_factor = random.uniform(1.10, 1.30)
            self.frequency_jitter_amplitude = random.uniform(0.2, 0.5)
            self.frequency_base_drift = random.uniform(0.1, 0.4)
            self.progressive_amplitude_growth = random.uniform(0.05, 0.08)
            self.progressive_frequency_instability = random.uniform(0.015, 0.025)
        
        self.damping_rise_rate = self.damping_target_increase / self.instability_duration
        
        self.oscillation_frequency = random.uniform(0.1, 0.3)
        self.oscillation_amplitude = random.uniform(0.5, 1.0)
        
        self.fault_configured = True
    
    def trigger_fault(self):
        if self.fault_active:
            return
            
        current_phase = self.phase_controller.get_current_phase()
        
        if current_phase == 'takeoff':
            delay = random.uniform(1.0, 2.0)
        elif current_phase == 'cruise':
            delay = random.uniform(2.0, 6.0)
        elif current_phase == 'landing':
            delay = random.uniform(0.5, 2.0)
        else:
            delay = random.uniform(1.0, 3.0)
        
        self.fault_trigger_time = time.time() + delay
        self.fault_active = True
        self.fault_was_triggered = True
        
        print(f"[FLUTTER FAULT] Fault configured for {current_phase} phase")
        print(f"[FLUTTER FAULT] Will trigger in {delay:.1f} seconds")
    
    def check_phase_transition(self):
        current_phase = self.phase_controller.get_current_phase()
        
        if self.last_known_phase != current_phase:
            
            if self.last_known_phase == 'cruise' and current_phase == 'landing':
                self.cruise_phase_ended = True
                print(f"[FLUTTER_FAULT] Cruise phase ended, will stop publishing soon")
                
                self.stop_publishing_trigger_time = time.time() + random.uniform(1.0, 2.0)
                
            self.last_known_phase = current_phase
        
        if (self.cruise_phase_ended and 
            not self.stop_publishing_triggered and 
            hasattr(self, 'stop_publishing_trigger_time') and 
            time.time() >= self.stop_publishing_trigger_time):
            
            self.should_stop_publishing = False
            self.stop_publishing_triggered = True
            print(f"[FLUTTER_FAULT] 🚨 SENSOR FAILURE: Flutter sensor stopped publishing!")
    
    def update_fault_progression(self):
        if not self.fault_active or self.fault_trigger_time is None:
            return
            
        current_time = time.time()
        
        if current_time < self.fault_trigger_time:
            return
            
        if self.fault_start_time is None:
            self.fault_start_time = current_time
            self.configure_fault()
            print(f"[FLUTTER FAULT] False damping ratio increase fault activated!")
        
        elapsed_time = current_time - self.fault_start_time
        flight_elapsed = current_time - self.flight_start_time
        
        if elapsed_time < self.instability_duration:
            self.damping_current_bias = self.damping_rise_rate * elapsed_time
        else:
            self.damping_current_bias = self.damping_target_increase
        
        time_factor = min(elapsed_time / self.instability_duration, 1.0)
        self.amplitude_instability_factor = 1.0 + (self.amplitude_instability_factor - 1.0) * time_factor
        
        self.progressive_frequency_instability = self.progressive_frequency_instability * time_factor
        
        self.damping_current_bias = max(0.0, min(0.15, self.damping_current_bias))
        self.amplitude_instability_factor = max(1.0, min(3.0, self.amplitude_instability_factor))
    
    def inject_fault(self, frequency, amplitude, damping_ratio, phase):
        self.check_phase_transition()
        
        if self.should_stop_publishing:
            pass
        
        if not self.fault_active:
            return frequency, amplitude, damping_ratio
        
        self.update_fault_progression()
        
        if self.fault_trigger_time is None or time.time() < self.fault_trigger_time:
            return frequency, amplitude, damping_ratio
        
        elapsed_time = time.time() - self.fault_start_time if self.fault_start_time else 0
        
        faulty_damping = damping_ratio + self.damping_current_bias
        
        oscillation_time = elapsed_time * self.oscillation_frequency * 2 * math.pi
        damping_oscillation = 0.01 * math.sin(oscillation_time)
        faulty_damping += damping_oscillation
        
        faulty_amplitude = amplitude * self.amplitude_instability_factor
        
        progressive_growth = self.progressive_amplitude_growth * elapsed_time
        faulty_amplitude += progressive_growth
        
        amplitude_oscillation = self.oscillation_amplitude * math.sin(oscillation_time * 1.5)
        faulty_amplitude += amplitude_oscillation * 0.1
        
        faulty_frequency = frequency + self.frequency_base_drift * (elapsed_time / 10.0)
        
        jitter_time = elapsed_time * 3.0
        frequency_jitter = self.frequency_jitter_amplitude * math.sin(jitter_time) * 0.3
        faulty_frequency += frequency_jitter
        
        progressive_freq_instability = self.progressive_frequency_instability * elapsed_time
        freq_noise = random.uniform(-progressive_freq_instability, progressive_freq_instability)
        faulty_frequency += freq_noise
        
        if phase == 'takeoff':
            faulty_frequency = max(3.5, min(faulty_frequency, 8.0))
            faulty_amplitude = max(0.1, min(faulty_amplitude, 5.0))
            faulty_damping = max(0.030, min(faulty_damping, 0.200))
        elif phase == 'cruise':
            faulty_frequency = max(7.0, min(faulty_frequency, 12.0))
            faulty_amplitude = max(0.1, min(faulty_amplitude, 3.0))
            faulty_damping = max(0.070, min(faulty_damping, 0.250))
        elif phase == 'landing':
            faulty_frequency = max(4.0, min(faulty_frequency, 8.0))
            faulty_amplitude = max(0.1, min(faulty_amplitude, 6.0))
            faulty_damping = max(0.040, min(faulty_damping, 0.220))
        else:
            faulty_frequency = max(3.0, min(faulty_frequency, 15.0))
            faulty_amplitude = max(0.1, min(faulty_amplitude, 5.0))
            faulty_damping = max(0.020, min(faulty_damping, 0.200))
        
        faulty_frequency = round(faulty_frequency, 2)
        faulty_amplitude = round(faulty_amplitude, 2)
        faulty_damping = round(faulty_damping, 3)
        
        return faulty_frequency, faulty_amplitude, faulty_damping
    
    def is_fault_active(self):
        return self.fault_active and self.fault_trigger_time is not None and time.time() >= self.fault_trigger_time
    
    def is_publishing_stopped(self):
        return self.should_stop_publishing
    
    def get_fault_status(self):
        if not self.fault_active:
            return {
                'fault_type': 'false_damping_ratio_increase',
                'active': False,
                'damping_bias': 0.0,
                'amplitude_factor': 1.0,
                'frequency_jitter': 0.0,
                'elapsed_time': 0.0,
                'flight_time': time.time() - self.flight_start_time,
                'sensor_name': 'flutter',
                'publishing_stopped': self.should_stop_publishing,
                'cruise_phase_ended': self.cruise_phase_ended
            }
        
        elapsed_time = 0.0
        if self.fault_start_time is not None:
            elapsed_time = time.time() - self.fault_start_time
        
        return {
            'fault_type': 'false_damping_ratio_increase',
            'active': self.is_fault_active(),
            'damping_bias': self.damping_current_bias,
            'damping_target_increase': self.damping_target_increase,
            'amplitude_factor': self.amplitude_instability_factor,
            'frequency_jitter': self.frequency_jitter_amplitude,
            'frequency_base_drift': self.frequency_base_drift,
            'progressive_amplitude_growth': self.progressive_amplitude_growth,
            'progressive_frequency_instability': self.progressive_frequency_instability,
            'elapsed_time': elapsed_time,
            'flight_time': time.time() - self.flight_start_time,
            'instability_duration': self.instability_duration,
            'sensor_name': 'flutter',
            'publishing_stopped': self.should_stop_publishing,
            'cruise_phase_ended': self.cruise_phase_ended
        }


class FlutterFaultManager:
    def __init__(self, phase_controller):
        self.phase_controller = phase_controller
        self.current_fault = None
        self.fault_injectors = {
            'false_damping_ratio_increase': FlutterFaultInjector(phase_controller)
        }
        self.fault_was_triggered = False
        self.fault_notification_sent = False
        
    def trigger_random_fault(self):
        fault_type = 'false_damping_ratio_increase'
        
        if fault_type in self.fault_injectors:
            self.current_fault = self.fault_injectors[fault_type]
            self.current_fault.trigger_fault()
            self.fault_was_triggered = True
            print(f"[FLUTTER FAULT] Fault manager activated: {fault_type}")
        else:
            print(f"[FLUTTER FAULT] Error: Unknown fault type: {fault_type}")
    
    def inject_fault(self, frequency, amplitude, damping_ratio, phase):
        if self.current_fault is not None:
            result = self.current_fault.inject_fault(frequency, amplitude, damping_ratio, phase)
            return result
        else:
            return frequency, amplitude, damping_ratio
    
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
                'sensor_name': 'flutter',
                'publishing_stopped': False,
                'cruise_phase_ended': False
            }


def update_flutter_with_fault_injection(flutter_sensor, fault_manager, bus, phase_controller):
    current_phase = phase_controller.get_current_phase()
    
    flutter_sensor.update_flutter_sensor()
    
    original_frequency = flutter_sensor.freq
    original_amplitude = flutter_sensor.amp
    original_damping = flutter_sensor.damp
    
    fault_result = fault_manager.inject_fault(
        original_frequency, original_amplitude, original_damping, current_phase
    )
    
    if fault_result and fault_result[0] is not None:
        faulty_frequency, faulty_amplitude, faulty_damping = fault_result
        
        if fault_manager.is_fault_active():
            output_frequency = faulty_frequency
            output_amplitude = faulty_amplitude
            output_damping = faulty_damping
            
            if fault_manager.current_fault and fault_manager.current_fault.fault_start_time:
                elapsed = time.time() - fault_manager.current_fault.fault_start_time
                if elapsed < 5.0:
                    print(f"[FLUTTER FAULT] Original: f={original_frequency:.2f}Hz, a={original_amplitude:.2f}mm, d={original_damping:.3f}")
                    print(f"[FLUTTER FAULT] Faulty:   f={output_frequency:.2f}Hz, a={output_amplitude:.2f}mm, d={output_damping:.3f}")
        else:
            output_frequency = original_frequency
            output_amplitude = original_amplitude
            output_damping = original_damping
    
    bus.publish("wing/flutter", {
        "sensor": "flutter",
        "phase": current_phase,
        "frequency (Hz)": output_frequency,
        "amplitude (mm)": output_amplitude,
        "damping_ratio": output_damping,
        "timestamp": time.time()
    })
    
    return {
    "sensor": "flutter",
    "phase": current_phase,
    "frequency (Hz)": output_frequency,
    "amplitude (mm)": output_amplitude,
    "damping_ratio": output_damping,
}