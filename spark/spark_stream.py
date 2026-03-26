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
    StructField("value", MapType(StringType(), DoubleType()))
])

# Parse JSON
parsed = df.selectExpr("CAST(value AS STRING)") \
    .select(from_json(col("value"), schema).alias("data")) \
    .select("data.*")

# Print stream
query = parsed.writeStream \
    .foreach(send_to_model) \
    .start()

query.awaitTermination()
