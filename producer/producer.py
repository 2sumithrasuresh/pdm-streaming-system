from kafka import KafkaProducer
import json
import time
import random

producer = KafkaProducer(
    bootstrap_servers='localhost:29092',
    value_serializer=lambda v: json.dumps(v).encode('utf-8')
)

sensors = ["accelerometer", "flap", "flutter"]

def generate_data():
    sensor = random.choice(sensors)

    if sensor == "accelerometer":
        return {
            "timestamp": time.time(),
            "sensor": "accelerometer",
            "value": {
                "accel_x": random.uniform(-1, 1),
                "accel_y": random.uniform(-1, 1),
                "accel_z": random.uniform(0.8, 1.5)
            }
        }

    elif sensor == "flap":
        return {
            "timestamp": time.time(),
            "sensor": "flap",
            "value": {
                "flap_angle": random.uniform(0, 15),
                "motor_current": random.uniform(0, 3)
            }
        }

    elif sensor == "flutter":
        return {
            "timestamp": time.time(),
            "sensor": "flutter",
            "value": {
                "frequency": random.uniform(4, 10),
                "amplitude": random.uniform(1, 5)
            }
        }


while True:
    data = generate_data()
    producer.send("sensor-data", data)
    print("Sent:", data)
    time.sleep(1)
