import time
import random
import threading

class FlightPhaseController:
    def __init__(self, stop_event, num_sensors=10, logger=None):
        self.ready_sensors = 0
        self.start_barrier = threading.Condition()
        self.phases = ['takeoff', 'cruise', 'landing']
        self.phase_durations = {
            'takeoff': (6, 8),
            'cruise': (18, 24),
            'landing': (10, 14)
        }

        self.current_phase_index = 0
        self.current_phase = self.phases[self.current_phase_index]
        self.remaining_time = self._random_duration(self.current_phase)

        self.lock = threading.Lock()
        self.stop_event = stop_event
        self.logger = logger

        self.num_sensors = num_sensors
        self.condition = threading.Condition()
        self.tick = 0

        self.completed_phases = 0

        self.acks_this_tick = 0
        self.ack_lock = threading.Lock()
        self.ack_barrier = threading.Condition()
        
        self.tick_barrier = threading.Condition()
        self.sensors_ready_for_tick = 0
        
        self.dynamic_sensor_count_lock = threading.Lock()

    def update_sensor_count(self, new_count):
        with self.dynamic_sensor_count_lock:
            old_count = self.num_sensors
            self.num_sensors = new_count
            print(f"[PHASE] Sensor count updated: {old_count} -> {new_count}")

    def get_current_sensor_count(self):
        with self.dynamic_sensor_count_lock:
            return self.num_sensors

    def _random_duration(self, phase):
        return random.randint(*self.phase_durations[phase])

    def get_current_phase(self):
        with self.lock:
            return self.current_phase

    def sync_tick(self):
        with self.condition:
            self.condition.wait()

    def acknowledge_tick(self):
        with self.ack_lock:
            self.acks_this_tick += 1
            current_sensor_count = self.get_current_sensor_count()
            if self.acks_this_tick >= current_sensor_count:
                with self.ack_barrier:
                    self.ack_barrier.notify_all()

    def _update_phase(self):
        if self.logger:
            self.logger.update_phase(self.current_phase)

    def tick_sync_loop(self):
        self.completed_phases = 0
        self.sim_start_time = time.time()
        self.last_tick_real = self.sim_start_time
        
        print(f"[PHASE] === NEW PHASE: {self.current_phase.upper()} | Duration: {self.remaining_time}s ===")
        self._update_phase()

        while not self.stop_event.is_set():
            tick_start = time.time()
            current_sensor_count = self.get_current_sensor_count()

            with self.ack_lock:
                self.acks_this_tick = 0

            with self.condition:
                self.tick += 1
                self.condition.notify_all()

            with self.ack_barrier:
                while self.acks_this_tick < current_sensor_count and not self.stop_event.is_set():
                    self.ack_barrier.wait(timeout=2.0)
                    if self.acks_this_tick < current_sensor_count:
                        print(f"[DEBUG] Only {self.acks_this_tick}/{current_sensor_count} sensors acknowledged tick {self.tick}")

            if self.stop_event.is_set():
                break

            with self.lock:
                self.remaining_time -= 1
                print(f"[TICK {self.tick}] Phase: {self.current_phase}, Remaining: {self.remaining_time}s ({self.acks_this_tick}/{current_sensor_count} sensors)")

                if self.remaining_time <= 0:
                    print(f"\n[PHASE] === {self.current_phase.upper()} COMPLETED ===")
                    self.completed_phases += 1

                    if self.completed_phases >= len(self.phases):
                        duration = time.time() - self.sim_start_time
                        print(f"\n[INFO] All phases completed. Total real time: {duration:.2f}s")
                        self.stop_event.set()
                        break

                    self.current_phase_index += 1
                    self.current_phase = self.phases[self.current_phase_index]
                    self.remaining_time = self._random_duration(self.current_phase)
                    print(f"[PHASE] === NEW PHASE: {self.current_phase.upper()} | Duration: {self.remaining_time}s ===")
                    self._update_phase()

            now = time.time()
            real_elapsed = now - self.last_tick_real
            print(f"[DEBUG] Real time since last tick: {real_elapsed:.3f}s")
            self.last_tick_real = now

            elapsed = time.time() - tick_start
            sleep_remaining = max(0.0, 1.0 - elapsed)
            if sleep_remaining > 0:
                time.sleep(sleep_remaining)

        with self.condition:
            self.condition.notify_all()