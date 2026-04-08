from pyspark.sql import SparkSession
from pyspark.sql.functions import (
    col, from_json, window, to_timestamp,
    when, max
)
from pyspark.sql.types import *
import requests

# -----------------------------
# SPARK SESSION
# -----------------------------
spark = SparkSession.builder \
    .appName("PDM Pipeline - Full Sensors") \
    .getOrCreate()

spark.sparkContext.setLogLevel("WARN")

# -----------------------------
# MODEL CALL
# -----------------------------
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

# -----------------------------
# READ FROM KAFKA
# -----------------------------
df = spark.readStream \
    .format("kafka") \
    .option("kafka.bootstrap.servers", "kafka:9092") \
    .option("subscribe", "sensor-data") \
    .load()

# -----------------------------
# SCHEMA
# -----------------------------
schema = StructType([
    StructField("timestamp", DoubleType()),
    StructField("sensor", StringType()),
    StructField("value", StringType()),
    StructField("prediction", DoubleType())
])

# -----------------------------
# PARSE JSON
# -----------------------------
raw = df.selectExpr("CAST(value AS STRING) as json_str")

parsed = raw.select(
    from_json(col("json_str"), schema).alias("data")
).select("data.*")

value_parsed = parsed.withColumn(
    "value_map",
    from_json(col("value"), MapType(StringType(), StringType()))
)

# -----------------------------
# FLATTEN ALL FEATURES
# -----------------------------
flattened = value_parsed.select(
    "timestamp",
    "sensor",
    "prediction",

    # Accelerometer
    col("value_map").getItem("accel_x (g)").cast("double").alias("accel_x"),
    col("value_map").getItem("accel_y (g)").cast("double").alias("accel_y"),
    col("value_map").getItem("accel_z (g)").cast("double").alias("accel_z"),
    col("value_map").getItem("dominant_freq (Hz)").cast("double").alias("accel_freq"),
    col("value_map").getItem("rms (g)").cast("double").alias("accel_rms"),

    # Strain
    col("value_map").getItem("strain_microstrain").cast("double").alias("strain"),
    col("value_map").getItem("temperature_C").cast("double").alias("strain_temp"),
    col("value_map").getItem("snr_dB").cast("double").alias("snr"),

    # Pressure
    col("value_map").getItem("dynamic_pressure_Pa").cast("double").alias("pressure"),
    col("value_map").getItem("pressure_gradient_Pa_per_cm").cast("double").alias("pressure_gradient"),
    col("value_map").getItem("asymmetry_index_percent").cast("double").alias("asymmetry"),

    # Fuel
    col("value_map").getItem("temperature_C").cast("double").alias("fuel_temp"),
    col("value_map").getItem("gradient_C_per_min").cast("double").alias("fuel_gradient"),
    col("value_map").getItem("jitter_C").cast("double").alias("fuel_jitter"),

    # Flutter
    col("value_map").getItem("frequency_Hz").cast("double").alias("flutter_freq"),
    col("value_map").getItem("amplitude_mm").cast("double").alias("flutter_amp"),
    col("value_map").getItem("damping_ratio").cast("double").alias("flutter_damping"),

    # Flap
    col("value_map").getItem("flap_angle").cast("double").alias("flap_angle"),
    col("value_map").getItem("motor_current").cast("double").alias("flap_motor"),
    col("value_map").getItem("hydraulic_pressure").cast("double").alias("flap_pressure"),

    # Slat
    col("value_map").getItem("commanded_angle_deg").cast("double").alias("slat_cmd"),
    col("value_map").getItem("actual_angle_deg").cast("double").alias("slat_actual"),
    col("value_map").getItem("tracking_error_deg").cast("double").alias("slat_error"),
    col("value_map").getItem("motor_current_A").cast("double").alias("slat_motor"),

    # Spoiler
    col("value_map").getItem("lag_ms").cast("double").alias("spoiler_lag"),

    # Spar
    col("value_map").getItem("bending_moment_Nm").cast("double").alias("spar_bending"),
    col("value_map").getItem("shear_force_N").cast("double").alias("spar_shear"),
    col("value_map").getItem("axial_load_N").cast("double").alias("spar_axial"),

    # Wingtip Node
    col("value_map").getItem("acceleration_g").cast("double").alias("node_accel"),
    col("value_map").getItem("deflection_cm").cast("double").alias("node_deflection"),
    col("value_map").getItem("strain_microstrain").cast("double").alias("node_strain")
)

# -----------------------------
# EVENT TIME + WATERMARK
# -----------------------------
with_time = flattened.withColumn(
    "event_time",
    to_timestamp(col("timestamp"))
).withWatermark("event_time", "2 seconds")

# -----------------------------
# BUCKET + NORMALIZE
# -----------------------------
bucketed = with_time.groupBy(
    window(col("event_time"), "1 second")
).agg(

    # Accelerometer
    max(when(col("sensor") == "accelerometer", col("accel_x"))).alias("accel_x"),
    max(when(col("sensor") == "accelerometer", col("accel_y"))).alias("accel_y"),
    max(when(col("sensor") == "accelerometer", col("accel_z"))).alias("accel_z"),
    max(when(col("sensor") == "accelerometer", col("accel_freq"))).alias("accel_freq"),
    max(when(col("sensor") == "accelerometer", col("accel_rms"))).alias("accel_rms"),

    # Strain
    max(when(col("sensor") == "wingtip_strain", col("strain"))).alias("strain"),
    max(when(col("sensor") == "wingtip_strain", col("strain_temp"))).alias("strain_temp"),
    max(when(col("sensor") == "wingtip_strain", col("snr"))).alias("snr"),

    # Pressure
    max(when(col("sensor") == "wing_surface_pressure", col("pressure"))).alias("pressure"),
    max(when(col("sensor") == "wing_surface_pressure", col("pressure_gradient"))).alias("pressure_gradient"),
    max(when(col("sensor") == "wing_surface_pressure", col("asymmetry"))).alias("asymmetry"),

    # Fuel
    max(when(col("sensor") == "fueltemp", col("fuel_temp"))).alias("fuel_temp"),
    max(when(col("sensor") == "fueltemp", col("fuel_gradient"))).alias("fuel_gradient"),
    max(when(col("sensor") == "fueltemp", col("fuel_jitter"))).alias("fuel_jitter"),

    # Flutter
    max(when(col("sensor") == "flutter", col("flutter_freq"))).alias("flutter_freq"),
    max(when(col("sensor") == "flutter", col("flutter_amp"))).alias("flutter_amp"),
    max(when(col("sensor") == "flutter", col("flutter_damping"))).alias("flutter_damping"),

    # Flap
    max(when(col("sensor") == "flap", col("flap_angle"))).alias("flap_angle"),
    max(when(col("sensor") == "flap", col("flap_motor"))).alias("flap_motor"),
    max(when(col("sensor") == "flap", col("flap_pressure"))).alias("flap_pressure"),

    # Slat
    max(when(col("sensor") == "slat", col("slat_cmd"))).alias("slat_cmd"),
    max(when(col("sensor") == "slat", col("slat_actual"))).alias("slat_actual"),
    max(when(col("sensor") == "slat", col("slat_error"))).alias("slat_error"),
    max(when(col("sensor") == "slat", col("slat_motor"))).alias("slat_motor"),

    # Spoiler
    max(when(col("sensor") == "spoiler", col("spoiler_lag"))).alias("spoiler_lag"),

    # Spar
    max(when(col("sensor") == "spar", col("spar_bending"))).alias("spar_bending"),
    max(when(col("sensor") == "spar", col("spar_shear"))).alias("spar_shear"),
    max(when(col("sensor") == "spar", col("spar_axial"))).alias("spar_axial"),

    # Wingtip Node
    max(when(col("sensor") == "wingtip_node", col("node_accel"))).alias("node_accel"),
    max(when(col("sensor") == "wingtip_node", col("node_deflection"))).alias("node_deflection"),
    max(when(col("sensor") == "wingtip_node", col("node_strain"))).alias("node_strain"),
    
    # Prediction
    max("prediction").alias("prediction")
)

# -----------------------------
# FINAL OUTPUT
# -----------------------------
final_df = bucketed.select(
    col("window.start").alias("timestamp"),
    "*"
).drop("window")

# -----------------------------
# DEBUG OUTPUT
# -----------------------------
final_df.writeStream \
    .format("console") \
    .option("truncate", False) \
    .outputMode("append") \
    .start()

# -----------------------------
# SEND TO MODEL
# -----------------------------
query = final_df.writeStream \
    .foreach(send_to_model) \
    .start()

query.awaitTermination()
