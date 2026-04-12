from pyspark.sql import SparkSession
from pyspark.sql.functions import *
from pyspark.sql.types import *

# -----------------------------
# SPARK SESSION
# -----------------------------
spark = SparkSession.builder \
    .appName("PDM Final Pipeline") \
    .getOrCreate()

spark.sparkContext.setLogLevel("WARN")
spark.conf.set("spark.sql.shuffle.partitions", "2")

# -----------------------------
# READ FROM KAFKA
# -----------------------------
df = spark.readStream \
    .format("kafka") \
    .option("kafka.bootstrap.servers", "kafka:9092") \
    .option("subscribe", "sensor-data") \
    .option("startingOffsets", "latest") \
    .option("failOnDataLoss", "false") \
    .load()

# -----------------------------
# SCHEMA (FIXED)
# -----------------------------
schema = StructType([
    StructField("timestamp", DoubleType()),
    StructField("sensor", StringType()),
    StructField("value", StringType()),
    StructField("prediction", DoubleType()),
    StructField("is_anomaly", BooleanType()),
    StructField("phase", StringType())
])

# -----------------------------
# PARSE JSON
# -----------------------------
raw = df.selectExpr("CAST(value AS STRING) as json_str")

parsed = raw.select(
    from_json(col("json_str"), schema).alias("data")
).select("data.*")

# 🔥 FIX: parse inner JSON properly
value_parsed = parsed.withColumn(
    "value_map",
    from_json(col("value"), MapType(StringType(), StringType()))
)

# -----------------------------
# 🔥 FLATTEN (FIXED KEYS)
# -----------------------------
flattened = value_parsed.select(
    "timestamp",
    "sensor",
    "prediction",
    "is_anomaly",
    "phase",

    # accelerometer
    col("value_map").getItem("accel_x (g)").cast("double").alias("accel_x"),
    col("value_map").getItem("accel_y (g)").cast("double").alias("accel_y"),
    col("value_map").getItem("accel_z (g)").cast("double").alias("accel_z"),

    # strain sensors
    col("value_map").getItem("strain_microstrain").cast("double").alias("strain"),

    # pressure sensor
    col("value_map").getItem("dynamic_pressure_Pa").cast("double").alias("pressure"),

    # temperature
    col("value_map").getItem("temperature_C").cast("double").alias("fuel_temp"),
    col("value_map").getItem("temperature_C").cast("double").alias("strain_temp"),    # wingtip_strain sensor
    col("value_map").getItem("snr_dB").cast("double").alias("snr_db"),

    # flutter
    col("value_map").getItem("frequency (Hz)").cast("double").alias("flutter_freq"),

    # control surfaces
    col("value_map").getItem("flap_angle").cast("double").alias("flap_angle"),
    col("value_map").getItem("actual_angle_deg").cast("double").alias("slat_actual"),
    col("value_map").getItem("lag_ms").cast("double").alias("spoiler_lag"),

    # structural
    col("value_map").getItem("bending_moment_Nm").cast("double").alias("spar_bending"),
    col("value_map").getItem("deflection_cm").cast("double").alias("node_deflection")
)

# -----------------------------
# EVENT TIME
# -----------------------------
with_time = flattened.withColumn(
    "event_time",
    to_timestamp(col("timestamp"))
)

# -----------------------------
# GROUP PER SENSOR
# -----------------------------
bucketed = with_time.groupBy(
    window(col("event_time"), "5 second"),
    col("sensor"),
    col("phase")
).agg(
    max("accel_x").alias("accel_x"),
    max("accel_y").alias("accel_y"),
    max("accel_z").alias("accel_z"),
    max("strain").alias("strain"),
    max("pressure").alias("pressure"),
    max("fuel_temp").alias("fuel_temp"),
    max("flutter_freq").alias("flutter_freq"),
    max("flap_angle").alias("flap_angle"),
    max("slat_actual").alias("slat_actual"),
    max("spoiler_lag").alias("spoiler_lag"),
    max("spar_bending").alias("spar_bending"),
    max("node_deflection").alias("node_deflection"),
    max("prediction").alias("score"),
    max("is_anomaly").alias("is_anomaly")
)

# -----------------------------
# FINAL DF
# -----------------------------
final_df = bucketed.select(
    col("window.start").alias("timestamp"),
    col("sensor"),
    col("phase"),
    "accel_x", "accel_y", "accel_z",
    "strain", "pressure", "fuel_temp",
    "flutter_freq", "flap_angle",
    "slat_actual", "spoiler_lag",
    "spar_bending", "node_deflection",
    "score", "is_anomaly"
)

# -----------------------------
# KAFKA SINK FUNCTION
# -----------------------------
def send_model_updates(df, epoch_id):
    from kafka import KafkaProducer
    import json
    
    print("🚀 NEW BATCH TRIGGERED:", epoch_id)
    df = df.limit(50)

    SENSOR_FEATURE_MAP = {
        "accelerometer":  ["accel_x", "accel_y", "accel_z"],
        "flutter":        ["flutter_freq"],
        "fueltemp":       ["fuel_temp"],
        "wingtip_strain": ["strain", "spoiler_lag"],   
        "wingtip_node":   ["node_deflection", "strain"],
        "pressure":       ["pressure"],
        "flap":           ["flap_angle"],
        "slat":           ["slat_actual"],
        "spoiler":        ["spoiler_lag"],
        "spar":           ["spar_bending"],
    }
    
    producer = KafkaProducer(
        bootstrap_servers='kafka:9092',
        value_serializer=lambda v: json.dumps(v).encode('utf-8')
    )
    
    rows = df.collect()
    for row in rows:
        data = row.asDict()
        sensor = data.get("sensor")

        relevant_cols = SENSOR_FEATURE_MAP.get(sensor, [])
        features = {
            k: data[k] for k in relevant_cols
            if data.get(k) is not None
        }

        if not features:
            continue

        message = {
            "timestamp": str(data.get("timestamp")),
            "sensor": data.get("sensor"),
            "score": float(data.get("score", 0)),
            "is_anomaly": bool(data.get("is_anomaly")),
            "phase": data.get("phase"),
            "features": features
        }

        print("PHASE:", message["phase"])
        
        print("📦 Sending to edge:", message)

        producer.send("model-updates", message)

    producer.flush()

# -----------------------------
# OUTPUTS
# -----------------------------
console_query = final_df.writeStream \
    .format("console") \
    .option("truncate", False) \
    .outputMode("update") \
    .trigger(processingTime="5 second") \
    .start()

kafka_query = final_df.writeStream \
    .foreachBatch(send_model_updates) \
    .outputMode("update") \
    .trigger(processingTime="5 second") \
    .start()

spark.streams.awaitAnyTermination()
