import threading
import time
import json

from kafka import KafkaProducer
from ml_model import OnlineAnomalyModel

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
# SENSOR THREAD FUNCTION
# -----------------------------
def run_sensor(update_method, stop_event, phase_controller, sensor_name, producer):

    model = OnlineAnomalyModel()

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

            features = {}

            if isinstance(result, dict):
                for k, v in result.items():
                    try:
                        features[k] = float(v)
                    except (ValueError, TypeError):
                        # skip non-numeric fields like "status", "locked", etc.
                        continue
            else:
                try:
                    features["value"] = float(result)
                except (ValueError, TypeError):
                    features = {}

            model.update(features)

            prediction = model.predict(features)

            entry = {
                "timestamp": time.time(),
                "sensor": sensor_name,
                "value": result,
                "prediction": float(prediction)
            }

            # Send to Kafka
            producer.send("sensor-data", entry)
            print("Sent:", entry)

            phase_controller.acknowledge_tick()

        except Exception as e:
            print(f"[ERROR] Sensor update failed: {e}")
            phase_controller.acknowledge_tick()


# -----------------------------
# MAIN SIMULATION LOOP
# -----------------------------
def main():

    # Kafka Producer
    producer = KafkaProducer(
        bootstrap_servers='localhost:29092',
        value_serializer=lambda v: json.dumps(v).encode('utf-8')
    )

    for cycle in range(1, 1001):
        print("\n" + "="*70)
        print(f"🚀 STARTING SIMULATION CYCLE {cycle}/1000")
        print("="*70)

        stop_event = threading.Event()
        bus = MessageBus()
        logger = None

        SENSOR_COUNT = 10
        phase_controller = FlightPhaseController(stop_event, num_sensors=SENSOR_COUNT, logger=logger)
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
            (accel.update_accelerometer_sensor, "accelerometer"),
            (flutter.update_flutter_sensor, "flutter"),
            (fueltemp.update_fueltemp_sensor, "fueltemp"),
            (wingtip_strain.update_wingtip_strain, "wingtip_strain"),
            (wingtip_node.update_wingtip_node, "wingtip_node"),
            (wing_surface_pressure.update_wing_surface_pressure, "wing_surface_pressure"),
            (flap.update_flap_sensor, "flap"),
            (slat.update_slats, "slat"),
            (spoiler.update_spoiler, "spoiler"),
            (wing_spar.update_sensor, "spar")
        ]

        threads = []

        # Start sensor threads
        for sensor_update, sensor_name in sensor_configs:
            thread = threading.Thread(
                target=run_sensor,
                args=(sensor_update, stop_event, phase_controller, sensor_name, producer)
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

        print(f"✅ COMPLETED SIMULATION CYCLE {cycle}/1000")

        time.sleep(2)

    print("\n" + "="*70)
    print("🎉 ALL 1000 SIMULATION CYCLES COMPLETED 🎉")
    print("="*70)


if __name__ == "__main__":
    main() 
