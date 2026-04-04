from pyspark.sql import SparkSession
from pyspark.sql.functions import col, from_json
from pyspark.sql.types import *
import requests

spark = SparkSession.builder \
    .appName("PDM Pipeline") \
    .getOrCreate()

spark.sparkContext.setLogLevel("WARN")

def send_to_model(row):
    try:
        response = requests.post(
            "http://model-api:8000/predict",
            json=row.asDict()
        )
        prediction = response.json()["prediction"]

        print("Prediction:", prediction)

        if prediction > 0.8:
            print("⚠️ ALERT!")

    except Exception as e:
        print("Error:", e)

# Read Kafka
df = spark.readStream \
    .format("kafka") \
    .option("kafka.bootstrap.servers", "kafka:9092") \
    .option("subscribe", "sensor-data") \
    .load()

# Define schema
schema = StructType([
    StructField("timestamp", DoubleType()),
    StructField("sensor", StringType()),
    StructField("value", StringType())
])


raw = df.selectExpr("CAST(value AS STRING) as json_str")

parsed = raw.select(
    from_json(col("json_str"), schema).alias("data")
).select("data.*")

value_parsed = parsed.withColumn(
    "value_map",
    from_json(col("value"), MapType(StringType(), StringType()))
)

# Flatten map
flattened = value_parsed.select(
    "timestamp",
    "sensor",
    col("value_map").getItem("strain_microstrain").cast("double").alias("strain"),
    col("value_map").getItem("temperature_C").cast("double").alias("temp"),
    col("value_map").getItem("snr_dB").cast("double").alias("snr"),
    col("value_map").getItem("accel_x (g)").cast("double").alias("accel_x"),
    col("value_map").getItem("accel_y (g)").cast("double").alias("accel_y"),
    col("value_map").getItem("accel_z (g)").cast("double").alias("accel_z")
)

# Debug print stream
flattened.writeStream \
    .format("console") \
    .outputMode("append") \
    .option("truncate", False) \
    .start()

# Print stream
query = flattened.writeStream \
    .foreach(send_to_model) \
    .start()

query.awaitTermination()
