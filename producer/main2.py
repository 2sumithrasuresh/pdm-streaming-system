import threading
import time
import json
import os

from Common.message_bus import MessageBus

# Healthy sensors only
from Healthy_Nodes.accelerometer_sensor_sim import WingtipAccelerometer
from Healthy_Nodes.flutter_sensor_sim import FlutterSensor
from Healthy_Nodes.fueltemp_sensor_sim import FuelTempSensor
from Healthy_Nodes.wingtipStrain_sensor_sim import WingtipStrainSensor
from Healthy_Nodes.wingtip_sensor_sim import WingtipNodeSensor
from Healthy_Nodes.pressure_sensor import WingSurfacePressureSensor
from Healthy_Nodes.flap_sensor import FlapSensor
from Healthy_Nodes.slat_sensor import SlatSensor
from Healthy_Nodes.spoiler_sensor import SpoilerSensor
from Healthy_Nodes.wingsparload_sensor import WingSparLoadSensor

from Common.flight_phase import FlightPhaseController


# -----------------------------
# JSON LOGGING UTILITY
# -----------------------------
def append_json(filename, entry):
    if not os.path.exists(filename):
        with open(filename, "w") as f:
            json.dump([], f)

    with open(filename, "r") as f:
        data = json.load(f)

    data.append(entry)

    with open(filename, "w") as f:
        json.dump(data, f, indent=4)


# -----------------------------
# SENSOR THREAD FUNCTION
# -----------------------------
def run_sensor(update_method, stop_event, phase_controller, file_name):
    with phase_controller.start_barrier:
        phase_controller.ready_sensors += 1
        phase_controller.start_barrier.notify_all()

    while not stop_event.is_set():
        with phase_controller.condition:
            phase_controller.condition.wait(timeout=1.0)

        if stop_event.is_set():
            break

        try:
            result = update_method()
            entry = {
                "timestamp": time.time(),
                "value": result
            }
            append_json(file_name, entry)

            phase_controller.acknowledge_tick()

        except Exception as e:
            print(f"[ERROR] Sensor update failed: {e}")
            phase_controller.acknowledge_tick()


# -----------------------------
# MAIN SIMULATION LOOP
# -----------------------------
def main():
    while True:
        try:
            total_cycles = int(input("Enter the number of simulation cycles to run: "))
            if total_cycles < 1:
                print("Please enter a positive integer.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter a whole number.")

    for cycle in range(1, total_cycles + 1):
        print("\n" + "="*70)
        print(f"🚀 STARTING SIMULATION CYCLE {cycle}/{total_cycles}")
        print("="*70)

        stop_event = threading.Event()
        bus = MessageBus()

        SENSOR_COUNT = 10
        phase_controller = FlightPhaseController(stop_event, num_sensors=SENSOR_COUNT)
        bus.current_phase = phase_controller.get_current_phase()

        # Create healthy sensor objects
        accel = WingtipAccelerometer(bus, phase_controller)
        flutter = FlutterSensor(bus, phase_controller)
        fueltemp = FuelTempSensor(bus, phase_controller)
        wingtip_strain = WingtipStrainSensor(bus, phase_controller)
        wingtip_node = WingtipNodeSensor(bus, phase_controller)
        wing_surface_pressure = WingSurfacePressureSensor(bus, phase_controller)
        flap = FlapSensor(bus, phase_controller)
        slat = SlatSensor(bus, phase_controller)
        spoiler = SpoilerSensor(bus, phase_controller)
        wing_spar = WingSparLoadSensor(bus, phase_controller)

        sensor_configs = [
            (accel.update_accelerometer_sensor, "accelerometer.json"),
            (flutter.update_flutter_sensor, "flutter.json"),
            (fueltemp.update_fueltemp_sensor, "fueltemp.json"),
            (wingtip_strain.update_wingtip_strain, "wingtip_strain.json"),
            (wingtip_node.update_wingtip_node, "wingtip_node.json"),
            (wing_surface_pressure.update_wing_surface_pressure, "wing_surface_pressure.json"),
            (flap.update_flap_sensor, "flap.json"),
            (slat.update_slats, "slat.json"),
            (spoiler.update_spoiler, "spoiler.json"),
            (wing_spar.update_sensor, "spar.json")
        ]

        threads = []

        # Start sensor threads
        for sensor_update, json_file in sensor_configs:
            thread = threading.Thread(
                target=run_sensor,
                args=(sensor_update, stop_event, phase_controller, json_file)
            )
            threads.append(thread)
            thread.start()

        # Wait for all sensors to be ready
        with phase_controller.start_barrier:
            while phase_controller.ready_sensors < SENSOR_COUNT:
                phase_controller.start_barrier.wait()

        print("[INFO] All sensors ready. Beginning flight sequence...")

        # Start flight phase controller
        phase_thread = threading.Thread(target=phase_controller.tick_sync_loop)
        phase_thread.start()

        # Wait for simulation to finish
        phase_thread.join()

        print("\n[INFO] Ending cycle, shutting down sensors...")

        stop_event.set()
        with phase_controller.condition:
            phase_controller.condition.notify_all()

        for t in threads:
            t.join(timeout=2.0)

        print(f"✅ COMPLETED SIMULATION CYCLE {cycle}/{total_cycles}")

        time.sleep(2)  # Small pause before next cycle

    print("\n" + "="*70)
    print(f"🎉 ALL {total_cycles} SIMULATION CYCLES COMPLETED 🎉")
    print("="*70)


if __name__ == "__main__":
    main()