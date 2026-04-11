import threading
import time
import json
import os
import random

from Common.message_bus import MessageBus
from Common.flight_phase import FlightPhaseController

# -----------------------------------------------------------------------
# HEALTHY SENSOR IMPORTS
# -----------------------------------------------------------------------
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

# -----------------------------------------------------------------------
# FAULTY SENSOR IMPORTS
# -----------------------------------------------------------------------
from Faulty_Nodes.accelerometer_fault import (
    AccelerometerFaultManager,
    update_accelerometer_with_fault_injection
)
from Faulty_Nodes.flutter_fault import (
    FlutterFaultManager,
    update_flutter_with_fault_injection
)
from Faulty_Nodes.fueltemp_fault import (
    FuelTempFaultManager,
    update_fueltemp_with_fault_injection
)
from Faulty_Nodes.flap_fault import (
    FlapFaultManager,
    update_flap_with_fault_injection
)
from Faulty_Nodes.pressure_fault import (
    PressureFaultManager,
    update_pressure_with_fault_injection
)
from Faulty_Nodes.slat_fault import (
    SlatFaultManager,
    update_slat_with_fault_injection
)
from Faulty_Nodes.spoiler_fault import (
    SpoilerFaultManager,
    update_spoiler_with_fault_injection
)
from Faulty_Nodes.wingsparload_fault import (
    WingSparLoadFaultManager,
    update_wingsparload_with_fault_injection
)
from Faulty_Nodes.wingtip_fault import (
    WingtipNodeFaultManager,
    update_wingtip_node_with_fault_injection
)
from Faulty_Nodes.wingtipSensor_fault import (
    WingtipStrainFaultManager,
    update_wingtip_strain_with_fault_injection
)


# -----------------------------------------------------------------------
# JSON LOGGING UTILITY
# -----------------------------------------------------------------------
def append_json(filename, entry):
    if not os.path.exists(filename):
        with open(filename, "w") as f:
            json.dump([], f)

    with open(filename, "r") as f:
        data = json.load(f)

    data.append(entry)

    with open(filename, "w") as f:
        json.dump(data, f, indent=4)


# -----------------------------------------------------------------------
# SENSOR THREAD FUNCTION (shared by healthy and faulty sensors)
# -----------------------------------------------------------------------
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


# -----------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------
def main():
    print("\n" + "=" * 70)
    print("🛫  FLIGHT SENSOR SIMULATION — MIXED FAULT MODE")
    print("=" * 70)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    stop_event = threading.Event()
    bus = MessageBus()

    SENSOR_COUNT = 10
    phase_controller = FlightPhaseController(stop_event, num_sensors=SENSOR_COUNT)
    bus.current_phase = phase_controller.get_current_phase()

    # ------------------------------------------------------------------
    # Instantiate all sensor objects (healthy base for both modes)
    # ------------------------------------------------------------------
    accel          = WingtipAccelerometer(bus, phase_controller)
    flutter        = FlutterSensor(bus, phase_controller)
    fueltemp       = FuelTempSensor(bus, phase_controller)
    wingtip_strain = WingtipStrainSensor(bus, phase_controller)
    wingtip_node   = WingtipNodeSensor(bus, phase_controller)
    pressure       = WingSurfacePressureSensor(bus, phase_controller)
    flap           = FlapSensor(bus, phase_controller)
    slat           = SlatSensor(bus, phase_controller)
    spoiler        = SpoilerSensor(bus, phase_controller)
    spar           = WingSparLoadSensor(bus, phase_controller)

    # ------------------------------------------------------------------
    # Instantiate all fault managers
    # ------------------------------------------------------------------
    accel_fault          = AccelerometerFaultManager(phase_controller)
    flutter_fault        = FlutterFaultManager(phase_controller)
    fuel_fault           = FuelTempFaultManager(phase_controller)
    wingtip_strain_fault = WingtipStrainFaultManager(phase_controller)
    wingtip_node_fault   = WingtipNodeFaultManager(phase_controller)
    pressure_fault       = PressureFaultManager(phase_controller)
    flap_fault           = FlapFaultManager(phase_controller)
    slat_fault           = SlatFaultManager(phase_controller)
    spoiler_fault        = SpoilerFaultManager(phase_controller)
    spar_fault           = WingSparLoadFaultManager(phase_controller)

    # ------------------------------------------------------------------
    # Sensor pool: (label, healthy_fn, faulty_fn, json_file, fault_manager)
    # ------------------------------------------------------------------
    sensor_pool = [
        (
            "Accelerometer",
            accel.update_accelerometer_sensor,
            lambda: update_accelerometer_with_fault_injection(accel, accel_fault, bus, phase_controller),
            "accelerometer.json",
            accel_fault,
        ),
        (
            "Flutter",
            flutter.update_flutter_sensor,
            lambda: update_flutter_with_fault_injection(flutter, flutter_fault, bus, phase_controller),
            "flutter.json",
            flutter_fault,
        ),
        (
            "Fuel Temperature",
            fueltemp.update_fueltemp_sensor,
            lambda: update_fueltemp_with_fault_injection(fueltemp, fuel_fault, bus, phase_controller),
            "fueltemp.json",
            fuel_fault,
        ),
        (
            "Wingtip Strain",
            wingtip_strain.update_wingtip_strain,
            lambda: update_wingtip_strain_with_fault_injection(wingtip_strain, wingtip_strain_fault, bus, phase_controller),
            "wingtip_strain.json",
            wingtip_strain_fault,
        ),
        (
            "Wingtip Node",
            wingtip_node.update_wingtip_node,
            lambda: update_wingtip_node_with_fault_injection(wingtip_node, wingtip_node_fault, bus, phase_controller),
            "wingtip_node.json",
            wingtip_node_fault,
        ),
        (
            "Wing Surface Pressure",
            pressure.update_wing_surface_pressure,
            lambda: update_pressure_with_fault_injection(pressure, pressure_fault, bus, phase_controller),
            "wing_surface_pressure.json",
            pressure_fault,
        ),
        (
            "Flap",
            flap.update_flap_sensor,
            lambda: update_flap_with_fault_injection(flap, flap_fault, bus, phase_controller),
            "flap.json",
            flap_fault,
        ),
        (
            "Slat",
            slat.update_slats,
            lambda: update_slat_with_fault_injection(slat, slat_fault, bus, phase_controller),
            "slat.json",
            slat_fault,
        ),
        (
            "Spoiler",
            spoiler.update_spoiler,
            lambda: update_spoiler_with_fault_injection(spoiler, spoiler_fault, bus, phase_controller),
            "spoiler.json",
            spoiler_fault,
        ),
        (
            "Wing Spar Load",
            spar.update_sensor,
            lambda: update_wingsparload_with_fault_injection(spar, spar_fault, bus, phase_controller),
            "spar.json",
            spar_fault,
        ),
    ]

    # ------------------------------------------------------------------
    # Randomly assign healthy / faulty — guarantee at least one of each
    # ------------------------------------------------------------------
    indices = list(range(SENSOR_COUNT))
    random.shuffle(indices)
    forced_healthy_idx = indices[0]
    forced_faulty_idx  = indices[1]

    healthy_sensors = []
    faulty_sensors  = []
    sensor_configs  = []   # (update_fn, json_file) in same order as sensor_pool

    for i, (label, healthy_fn, faulty_fn, json_file, fault_mgr) in enumerate(sensor_pool):
        if i == forced_healthy_idx:
            mode = "healthy"
        elif i == forced_faulty_idx:
            mode = "faulty"
        else:
            mode = random.choice(["healthy", "faulty"])

        if mode == "healthy":
            healthy_sensors.append(label)
            sensor_configs.append((healthy_fn, json_file))
        else:
            faulty_sensors.append(label)
            fault_mgr.trigger_random_fault()          # arm the fault
            sensor_configs.append((faulty_fn, json_file))

    # ------------------------------------------------------------------
    # Print pre-flight assignment
    # ------------------------------------------------------------------
    print("\n📋 PRE-FLIGHT SENSOR ASSIGNMENT (randomly determined):\n")
    print(f"  ✅ HEALTHY ({len(healthy_sensors)}):")
    for s in healthy_sensors:
        print(f"      • {s}")
    print(f"\n  ⚠️  FAULTY  ({len(faulty_sensors)}):")
    for s in faulty_sensors:
        print(f"      • {s}")
    print()

    # ------------------------------------------------------------------
    # Launch one thread per sensor
    # ------------------------------------------------------------------
    threads = []
    for update_fn, json_file in sensor_configs:
        t = threading.Thread(
            target=run_sensor,
            args=(update_fn, stop_event, phase_controller, json_file)
        )
        threads.append(t)
        t.start()

    # Wait until every sensor thread has checked in
    with phase_controller.start_barrier:
        while phase_controller.ready_sensors < SENSOR_COUNT:
            phase_controller.start_barrier.wait()

    print("[INFO] All sensors ready. Beginning flight sequence...")

    # ------------------------------------------------------------------
    # Run flight phase controller (blocks until all phases complete)
    # ------------------------------------------------------------------
    phase_thread = threading.Thread(target=phase_controller.tick_sync_loop)
    phase_thread.start()
    phase_thread.join()

    print("\n[INFO] Flight complete. Shutting down sensors...")

    stop_event.set()
    with phase_controller.condition:
        phase_controller.condition.notify_all()

    for t in threads:
        t.join(timeout=2.0)

    # ------------------------------------------------------------------
    # Final summary
    # ------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📊  SIMULATION COMPLETE — SENSOR STATUS REPORT")
    print("=" * 70)

    print(f"\n  ✅ HEALTHY SENSORS ({len(healthy_sensors)}/{SENSOR_COUNT}):")
    if healthy_sensors:
        for s in healthy_sensors:
            print(f"      • {s}")
    else:
        print("      (none)")

    print(f"\n  ❌ FAULTY SENSORS  ({len(faulty_sensors)}/{SENSOR_COUNT}):")
    if faulty_sensors:
        for s in faulty_sensors:
            print(f"      • {s}")
    else:
        print("      (none)")

    print("\n" + "=" * 70)
    print("🏁  END OF SIMULATION")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()