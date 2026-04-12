import threading
import time
import json
import random

from kafka import KafkaProducer, KafkaConsumer
from ml_model import EWMAAnomalyModel

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
# SENSOR THREAD FUNCTION (NOW WITH KAFKA)
# -----------------------------------------------------------------------
def run_sensor(update_method, stop_event, phase_controller, producer, sensor_name, model):

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

            # -------- Feature Extraction --------
            KEY_MAP = {
                "accel_x (g)":              "accel_x",
                "accel_y (g)":              "accel_y",
                "accel_z (g)":              "accel_z",
                "strain_microstrain":       "strain",
                "dynamic_pressure_Pa":      "pressure",
                "temperature_C":            "fuel_temp",
                "frequency (Hz)":           "flutter_freq",
                "flap_angle":               "flap_angle",
                "actual_angle_deg":         "slat_actual",
                "lag_ms":                   "spoiler_lag",
                "bending_moment_Nm":        "spar_bending",
                "deflection_cm":            "node_deflection",
            }       

            features = {}
            if isinstance(result, dict):
                for k, v in result.items():
                    try:
                        clean_key = KEY_MAP.get(k, k)
                        features[clean_key] = float(v)
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
            
            # -------- ALERT MECHANISM (EDGE) --------
            if is_anomaly:
                alert = {
                    "timestamp": time.time(),
                    "sensor": sensor_name,
                    "prediction": float(prediction),
                    "value": json.dumps(result),
                    "message": f"Anomaly detected in {sensor_name}"
                }

                # 1. Print (for debugging/demo)
                print(f"🚨 ALERT [{sensor_name}] → {prediction}")

                # 2. Send to Kafka alert topic
                producer.send("alerts", alert)
                producer.flush()

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

            phase_controller.acknowledge_tick()

        except Exception as e:
            print(f"[ERROR] Sensor update failed: {e}")
            phase_controller.acknowledge_tick()

def consume_model_updates(models):
    consumer = KafkaConsumer(
        "model-updates",
        bootstrap_servers="localhost:29092",
        value_deserializer=lambda m: json.loads(m.decode('utf-8'))
    )

    print("🔄 Listening for backend model updates...")

    for msg in consumer:
        data = msg.value
        
        print("📥 RECEIVED UPDATE:", data)   # 🔥 ADD HERE

        # Only learn from NORMAL data
        if not data.get("is_anomaly"):
            sensor = data.get("sensor")
            features = data.get("features", {})
            phase = data.get("phase", "unknown")

            if sensor in models:
                print(f"🔁 Updating model for {sensor}")
                print("Features:", features, "Phase:", phase)
                models[sensor].apply_backend_update(features, phase)

# -----------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------
def main():
    print("\n" + "=" * 70)
    print("🛫  FLIGHT SENSOR SIMULATION — MIXED MODE + KAFKA")
    print("=" * 70)

    stop_event = threading.Event()
    bus = MessageBus()

    # ✅ Kafka Producer
    producer = KafkaProducer(
        bootstrap_servers='localhost:29092',
        value_serializer=lambda v: json.dumps(v).encode('utf-8')
    )

    SENSOR_COUNT = 10
    phase_controller = FlightPhaseController(stop_event, num_sensors=SENSOR_COUNT)
    bus.current_phase = phase_controller.get_current_phase()

    # ------------------------------------------------------------------
    # ML Models (one per sensor)
    # ------------------------------------------------------------------
    models = {
        "accelerometer": EWMAAnomalyModel(),
        "flutter": EWMAAnomalyModel(),
        "fueltemp": EWMAAnomalyModel(),
        "wingtip_strain": EWMAAnomalyModel(),
        "wingtip_node": EWMAAnomalyModel(),
        "pressure": EWMAAnomalyModel(),
        "flap": EWMAAnomalyModel(),
        "slat": EWMAAnomalyModel(),
        "spoiler": EWMAAnomalyModel(),
        "spar": EWMAAnomalyModel()
    }

    # 🔥 START BACKEND RETRAINING LISTENER
    threading.Thread(
        target=consume_model_updates,
        args=(models,),
        daemon=True
    ).start()

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------
    accel = WingtipAccelerometer(bus, phase_controller)
    flutter = FlutterSensor(bus, phase_controller)
    fueltemp = FuelTempSensor(bus, phase_controller)
    wingtip_strain = WingtipStrainSensor(bus, phase_controller)
    wingtip_node = WingtipNodeSensor(bus, phase_controller)
    pressure = WingSurfacePressureSensor(bus, phase_controller)
    flap = FlapSensor(bus, phase_controller)
    slat = SlatSensor(bus, phase_controller)
    spoiler = SpoilerSensor(bus, phase_controller)
    spar = WingSparLoadSensor(bus, phase_controller)

    # ------------------------------------------------------------------
    # Fault managers
    # ------------------------------------------------------------------
    accel_fault = AccelerometerFaultManager(phase_controller)
    flutter_fault = FlutterFaultManager(phase_controller)
    fuel_fault = FuelTempFaultManager(phase_controller)
    wingtip_strain_fault = WingtipStrainFaultManager(phase_controller)
    wingtip_node_fault = WingtipNodeFaultManager(phase_controller)
    pressure_fault = PressureFaultManager(phase_controller)
    flap_fault = FlapFaultManager(phase_controller)
    slat_fault = SlatFaultManager(phase_controller)
    spoiler_fault = SpoilerFaultManager(phase_controller)
    spar_fault = WingSparLoadFaultManager(phase_controller)

    # ------------------------------------------------------------------
    # Sensor Pool
    # ------------------------------------------------------------------
    sensor_pool = [
        ("accelerometer", accel.update_accelerometer_sensor,
         lambda: update_accelerometer_with_fault_injection(accel, accel_fault, bus, phase_controller), accel_fault),

        ("flutter", flutter.update_flutter_sensor,
         lambda: update_flutter_with_fault_injection(flutter, flutter_fault, bus, phase_controller), flutter_fault),

        ("fueltemp", fueltemp.update_fueltemp_sensor,
         lambda: update_fueltemp_with_fault_injection(fueltemp, fuel_fault, bus, phase_controller), fuel_fault),

        ("wingtip_strain", wingtip_strain.update_wingtip_strain,
         lambda: update_wingtip_strain_with_fault_injection(wingtip_strain, wingtip_strain_fault, bus, phase_controller), wingtip_strain_fault),

        ("wingtip_node", wingtip_node.update_wingtip_node,
         lambda: update_wingtip_node_with_fault_injection(wingtip_node, wingtip_node_fault, bus, phase_controller), wingtip_node_fault),

        ("pressure", pressure.update_wing_surface_pressure,
         lambda: update_pressure_with_fault_injection(pressure, pressure_fault, bus, phase_controller), pressure_fault),

        ("flap", flap.update_flap_sensor,
         lambda: update_flap_with_fault_injection(flap, flap_fault, bus, phase_controller), flap_fault),

        ("slat", slat.update_slats,
         lambda: update_slat_with_fault_injection(slat, slat_fault, bus, phase_controller), slat_fault),

        ("spoiler", spoiler.update_spoiler,
         lambda: update_spoiler_with_fault_injection(spoiler, spoiler_fault, bus, phase_controller), spoiler_fault),

        ("spar", spar.update_sensor,
         lambda: update_wingsparload_with_fault_injection(spar, spar_fault, bus, phase_controller), spar_fault),
    ]

    # ------------------------------------------------------------------
    # Random Assignment
    # ------------------------------------------------------------------
    indices = list(range(SENSOR_COUNT))
    random.shuffle(indices)

    forced_healthy_idx = indices[0]
    forced_faulty_idx = indices[1]

    sensor_configs = []

    for i, (name, healthy_fn, faulty_fn, fault_mgr) in enumerate(sensor_pool):

        if i == forced_healthy_idx:
            mode = "healthy"
        elif i == forced_faulty_idx:
            mode = "faulty"
        else:
            mode = random.choice(["healthy", "faulty"])

        if mode == "faulty":
            fault_mgr.trigger_random_fault()
            sensor_configs.append((name, faulty_fn))
        else:
            sensor_configs.append((name, healthy_fn))

    # ------------------------------------------------------------------
    # Launch Threads
    # ------------------------------------------------------------------
    threads = []
    for name, fn in sensor_configs:
        t = threading.Thread(
            target=run_sensor,
            args=(fn, stop_event, phase_controller, producer, name, models[name])
        )
        threads.append(t)
        t.start()

    # Sync start
    with phase_controller.start_barrier:
        while phase_controller.ready_sensors < SENSOR_COUNT:
            phase_controller.start_barrier.wait()

    print("[INFO] All sensors ready. Starting flight...")

    phase_thread = threading.Thread(target=phase_controller.tick_sync_loop)
    phase_thread.start()
    phase_thread.join()

    stop_event.set()

    with phase_controller.condition:
        phase_controller.condition.notify_all()

    for t in threads:
        t.join(timeout=2.0)

    print("\n🏁 Simulation complete.\n")


if __name__ == "__main__":
    main()
