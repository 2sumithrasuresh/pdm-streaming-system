import time
import json
import os

from Common.message_bus import MessageBus
from Common.flight_phase import FlightPhaseController

# -----------------------------------------------------------------------
# HEALTHY SENSOR CLASSES
# -----------------------------------------------------------------------
from Healthy_Nodes.accelerometer_sensor_sim import WingtipAccelerometer
from Healthy_Nodes.flutter_sensor_sim import FlutterSensor
from Healthy_Nodes.fueltemp_sensor_sim import FuelTempSensor
from Healthy_Nodes.flap_sensor import FlapSensor
from Healthy_Nodes.pressure_sensor import WingSurfacePressureSensor
from Healthy_Nodes.slat_sensor import SlatSensor
from Healthy_Nodes.spoiler_sensor import SpoilerSensor
from Healthy_Nodes.wingsparload_sensor import WingSparLoadSensor
from Healthy_Nodes.wingtip_sensor_sim import WingtipNodeSensor
from Healthy_Nodes.wingtipStrain_sensor_sim import WingtipStrainSensor

# -----------------------------------------------------------------------
# FAULT MANAGERS + INJECTION FUNCTIONS
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
# JSON LOGGER
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
# RUN FAULTY SENSOR
# -----------------------------------------------------------------------
def run_faulty_sensor(update_func, file_name):
    result = update_func()

    entry = {
        "timestamp": time.time(),
        "value": result
    }

    append_json(file_name, entry)


# -----------------------------------------------------------------------
# MAIN SIMULATION
# -----------------------------------------------------------------------
def main():

    runs = int(input("Enter number of simulation runs: "))

    for run in range(1, runs + 1):
        print("\n" + "=" * 60)
        print(f"🔥 FAULTY SIMULATION RUN {run}/{runs}")
        print("=" * 60)

        bus = MessageBus()
        stop_event = type('', (), {"is_set": lambda self: False})()  # dummy

        # 10 faulty sensors
        phase_controller = FlightPhaseController(stop_event, num_sensors=10, logger=None)

        # -------------------------------------------------------------------
        # CREATE HEALTHY SENSOR OBJECTS
        # -------------------------------------------------------------------
        accel          = WingtipAccelerometer(bus, phase_controller)
        flutter        = FlutterSensor(bus, phase_controller)
        fueltemp       = FuelTempSensor(bus, phase_controller)
        flap           = FlapSensor(bus, phase_controller)
        pressure       = WingSurfacePressureSensor(bus, phase_controller)
        slat           = SlatSensor(bus, phase_controller)
        spoiler        = SpoilerSensor(bus, phase_controller)
        spar           = WingSparLoadSensor(bus, phase_controller)
        wingtip_node   = WingtipNodeSensor(bus, phase_controller)
        wingtip_strain = WingtipStrainSensor(bus, phase_controller)

        # -------------------------------------------------------------------
        # CREATE FAULT MANAGERS
        # -------------------------------------------------------------------
        accel_fault          = AccelerometerFaultManager(phase_controller)
        flutter_fault        = FlutterFaultManager(phase_controller)
        fuel_fault           = FuelTempFaultManager(phase_controller)
        flap_fault           = FlapFaultManager(phase_controller)
        pressure_fault       = PressureFaultManager(phase_controller)
        slat_fault           = SlatFaultManager(phase_controller)
        spoiler_fault        = SpoilerFaultManager(phase_controller)
        spar_fault           = WingSparLoadFaultManager(phase_controller)
        wingtip_node_fault   = WingtipNodeFaultManager(phase_controller)
        wingtip_strain_fault = WingtipStrainFaultManager(phase_controller)

        # Arm all faults
        accel_fault.trigger_random_fault()
        flutter_fault.trigger_random_fault()
        fuel_fault.trigger_random_fault()
        flap_fault.trigger_random_fault()
        pressure_fault.trigger_random_fault()
        slat_fault.trigger_random_fault()
        spoiler_fault.trigger_random_fault()
        spar_fault.trigger_random_fault()
        wingtip_node_fault.trigger_random_fault()
        wingtip_strain_fault.trigger_random_fault()

        print("⚠️  All 10 sensors running in FAULTY mode\n")

        # -------------------------------------------------------------------
        # SIMULATION LOOP
        # -------------------------------------------------------------------
        completed_phases = 0
        last_phase = None

        while completed_phases < 3:

            current_phase = phase_controller.get_current_phase()

            if current_phase != last_phase:
                print(f"\n✈️  PHASE: {current_phase.upper()}\n")
                last_phase = current_phase

            print(f"\n--- Tick {phase_controller.tick + 1} ---")

            # --- Accelerometer ---
            run_faulty_sensor(
                lambda: update_accelerometer_with_fault_injection(
                    accel, accel_fault, bus, phase_controller
                ),
                "accelerometer_faulty.json"
            )

            # --- Flutter ---
            run_faulty_sensor(
                lambda: update_flutter_with_fault_injection(
                    flutter, flutter_fault, bus, phase_controller
                ),
                "flutter_faulty.json"
            )

            # --- Fuel Temperature ---
            run_faulty_sensor(
                lambda: update_fueltemp_with_fault_injection(
                    fueltemp, fuel_fault, bus, phase_controller
                ),
                "fueltemp_faulty.json"
            )

            # --- Flap ---
            run_faulty_sensor(
                lambda: update_flap_with_fault_injection(
                    flap, flap_fault, bus, phase_controller
                ),
                "flap_faulty.json"
            )

            # --- Wing Surface Pressure ---
            run_faulty_sensor(
                lambda: update_pressure_with_fault_injection(
                    pressure, pressure_fault, bus, phase_controller
                ),
                "pressure_faulty.json"
            )

            # --- Slat ---
            run_faulty_sensor(
                lambda: update_slat_with_fault_injection(
                    slat, slat_fault, bus, phase_controller
                ),
                "slat_faulty.json"
            )

            # --- Spoiler ---
            run_faulty_sensor(
                lambda: update_spoiler_with_fault_injection(
                    spoiler, spoiler_fault, bus, phase_controller
                ),
                "spoiler_faulty.json"
            )

            # --- Wing Spar Load ---
            run_faulty_sensor(
                lambda: update_wingsparload_with_fault_injection(
                    spar, spar_fault, bus, phase_controller
                ),
                "spar_faulty.json"
            )

            # --- Wingtip Node ---
            run_faulty_sensor(
                lambda: update_wingtip_node_with_fault_injection(
                    wingtip_node, wingtip_node_fault, bus, phase_controller
                ),
                "wingtip_node_faulty.json"
            )

            # --- Wingtip Strain ---
            run_faulty_sensor(
                lambda: update_wingtip_strain_with_fault_injection(
                    wingtip_strain, wingtip_strain_fault, bus, phase_controller
                ),
                "wingtip_strain_faulty.json"
            )

            # Advance controller manually (no threading)
            phase_controller.tick += 1
            phase_controller.remaining_time -= 1

            if phase_controller.remaining_time <= 0:
                print(f"\n[PHASE COMPLETE] {current_phase.upper()}")

                phase_controller.completed_phases += 1
                completed_phases += 1

                if completed_phases >= 3:
                    break

                phase_controller.current_phase_index += 1
                phase_controller.current_phase = phase_controller.phases[
                    phase_controller.current_phase_index
                ]
                phase_controller.remaining_time = phase_controller._random_duration(
                    phase_controller.current_phase
                )

            time.sleep(0.5)

        print(f"\n✅ Completed Run {run}/{runs}\n")

    print("\n🎉 ALL FAULTY SIMULATIONS COMPLETE 🎉")


if __name__ == "__main__":
    main()