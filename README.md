# Commands to run

docker compose up -d

python3 producer/producer.py

docker exec -it spark-master \       
/opt/spark/bin/spark-submit \
--conf spark.jars.ivy=/tmp/.ivy \
--packages org.apache.spark:spark-sql-kafka-0-10_2.12:3.5.0 \
/opt/spark-apps/spark_stream.py

