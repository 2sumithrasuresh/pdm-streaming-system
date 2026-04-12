import time
import json
import os
import threading

from kafka import KafkaProducer, KafkaConsumer
from ml_model import EWMAAnomalyModel

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
# RUN FAULTY SENSOR (KAFKA ONLY)
# -----------------------------------------------------------------------
def run_faulty_sensor(update_func, sensor_name, producer, model):

    result = update_func()

    # -------- Feature Extraction --------
    features = {}

    if isinstance(result, dict):
        for k, v in result.items():
            try:
                features[k] = float(v)
            except:
                continue
    else:
        try:
            features["value"] = float(result)
        except:
            features = {}

    # -------- ML Model --------
    phase = phase_controller.get_current_phase()
    prediction = model.predict(features, phase)
    is_anomaly = model.is_anomaly(prediction, sensor_name)

    model.update(features, phase, is_anomaly=is_anomaly)

    entry = {
        "timestamp": time.time(),
        "sensor": sensor_name,
        "value": result,
        "prediction": float(prediction),
        "is_anomaly": bool(is_anomaly),
        "phase": phase
    }

    # -------- Kafka Send --------
    producer.send("sensor-data", entry)
    print("Sent:", entry)

def model_update_listener(models):
    consumer = KafkaConsumer(
        "model-updates",
        bootstrap_servers='localhost:29092',
        value_deserializer=lambda m: json.loads(m.decode('utf-8'))
    )

    print("📡 Listening for backend model updates...")

    for msg in consumer:
        data = msg.value
        sensor = data.get("sensor")

        if sensor in models:
            phase = data.get("phase")

            print("📥 RECEIVED BACKEND UPDATE:", data)

            models[sensor].apply_backend_update(
                data.get("features", {}),
                phase
            )
            print(f"🔄 Model updated for {sensor}")

# -----------------------------------------------------------------------
# MAIN SIMULATION
# -----------------------------------------------------------------------
def main():

    # Kafka Producer (same as File 2 :contentReference[oaicite:1]{index=1})
    producer = KafkaProducer(
        bootstrap_servers='localhost:29092',
        value_serializer=lambda v: json.dumps(v).encode('utf-8')
    )

    # One model per sensor
    models = {
        "accelerometer": EWMAAnomalyModel(),
        "flutter": EWMAAnomalyModel(),
        "fueltemp": EWMAAnomalyModel(),
        "flap": EWMAAnomalyModel(),
        "pressure": EWMAAnomalyModel(),
        "slat": EWMAAnomalyModel(),
        "spoiler": EWMAAnomalyModel(),
        "spar": EWMAAnomalyModel(),
        "wingtip_node": EWMAAnomalyModel(),
        "wingtip_strain": EWMAAnomalyModel()
    }

    # 🔥 Start backend update listener
    threading.Thread(
        target=model_update_listener,
        args=(models,),
        daemon=True
    ).start()

    runs = int(input("Enter number of simulation runs: "))

    for run in range(1, runs + 1):
        print("\n" + "=" * 60)
        print(f"🔥 FAULTY SIMULATION RUN {run}/{runs}")
        print("=" * 60)

        bus = MessageBus()
        stop_event = type('', (), {"is_set": lambda self: False})()

        phase_controller = FlightPhaseController(stop_event, num_sensors=10, logger=None)

        # Sensors
        accel = WingtipAccelerometer(bus, phase_controller)
        flutter = FlutterSensor(bus, phase_controller)
        fueltemp = FuelTempSensor(bus, phase_controller)
        flap = FlapSensor(bus, phase_controller)
        pressure = WingSurfacePressureSensor(bus, phase_controller)
        slat = SlatSensor(bus, phase_controller)
        spoiler = SpoilerSensor(bus, phase_controller)
        spar = WingSparLoadSensor(bus, phase_controller)
        wingtip_node = WingtipNodeSensor(bus, phase_controller)
        wingtip_strain = WingtipStrainSensor(bus, phase_controller)

        # Fault managers
        accel_fault = AccelerometerFaultManager(phase_controller)
        flutter_fault = FlutterFaultManager(phase_controller)
        fuel_fault = FuelTempFaultManager(phase_controller)
        flap_fault = FlapFaultManager(phase_controller)
        pressure_fault = PressureFaultManager(phase_controller)
        slat_fault = SlatFaultManager(phase_controller)
        spoiler_fault = SpoilerFaultManager(phase_controller)
        spar_fault = WingSparLoadFaultManager(phase_controller)
        wingtip_node_fault = WingtipNodeFaultManager(phase_controller)
        wingtip_strain_fault = WingtipStrainFaultManager(phase_controller)

        # Trigger faults
        for f in [
            accel_fault, flutter_fault, fuel_fault, flap_fault,
            pressure_fault, slat_fault, spoiler_fault,
            spar_fault, wingtip_node_fault, wingtip_strain_fault
        ]:
            f.trigger_random_fault()

        print("⚠️  All 10 sensors running in FAULTY mode\n")

        completed_phases = 0
        last_phase = None

        while completed_phases < 3:

            current_phase = phase_controller.get_current_phase()

            if current_phase != last_phase:
                print(f"\n✈️  PHASE: {current_phase.upper()}\n")
                last_phase = current_phase

            print(f"\n--- Tick {phase_controller.tick + 1} ---")

            run_faulty_sensor(lambda: update_accelerometer_with_fault_injection(accel, accel_fault, bus, phase_controller),
                              "accelerometer", producer, models["accelerometer"])

            run_faulty_sensor(lambda: update_flutter_with_fault_injection(flutter, flutter_fault, bus, phase_controller),
                              "flutter", producer, models["flutter"])

            run_faulty_sensor(lambda: update_fueltemp_with_fault_injection(fueltemp, fuel_fault, bus, phase_controller),
                              "fueltemp", producer, models["fueltemp"])

            run_faulty_sensor(lambda: update_flap_with_fault_injection(flap, flap_fault, bus, phase_controller),
                              "flap", producer, models["flap"])

            run_faulty_sensor(lambda: update_pressure_with_fault_injection(pressure, pressure_fault, bus, phase_controller),
                              "pressure", producer, models["pressure"])

            run_faulty_sensor(lambda: update_slat_with_fault_injection(slat, slat_fault, bus, phase_controller),
                              "slat", producer, models["slat"])

            run_faulty_sensor(lambda: update_spoiler_with_fault_injection(spoiler, spoiler_fault, bus, phase_controller),
                              "spoiler", producer, models["spoiler"])

            run_faulty_sensor(lambda: update_wingsparload_with_fault_injection(spar, spar_fault, bus, phase_controller),
                              "spar", producer, models["spar"])

            run_faulty_sensor(lambda: update_wingtip_node_with_fault_injection(wingtip_node, wingtip_node_fault, bus, phase_controller),
                              "wingtip_node", producer, models["wingtip_node"])

            run_faulty_sensor(lambda: update_wingtip_strain_with_fault_injection(wingtip_strain, wingtip_strain_fault, bus, phase_controller),
                              "wingtip_strain", producer, models["wingtip_strain"])

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
