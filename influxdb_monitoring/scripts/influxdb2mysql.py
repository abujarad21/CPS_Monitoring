import influxdb_client
import mysql.connector
import numpy as np
import pandas as pd
from datetime import datetime

#establishing the connection
conn = mysql.connector.connect(user='root', password='123AsD!@#', host='127.0.0.1', database='rosdb')
cursor= conn.cursor()
# Connect to InfluxDB
bucket = "rosdata"
org = "UTM"
token = "UEtQFAPc3cpITlis5JpR0zNjMYIwahBvUp-RIvqJyFjq-qMT5JOgrP716S9C4dcuevYWFqGBoiLKSniCuq37zg=="
url="https://us-east-1-1.aws.cloud2.influxdata.com"

client = influxdb_client.InfluxDBClient(
    url=url,
    token=token,
    org=org
)

query_api = client.query_api()

query = '''from(bucket: "rosdata")
 |> range(start: -15m)
 |> filter(fn: (r) => r["_measurement"] == "my_laser_scan" or r["_measurement"] == "odometry_pose")
  |> filter(fn: (r) => r["name"] == "/scan" or r["name"] == "/odom")
  |> filter(fn: (r) => r["robot"] == "my_robot")
  |> filter(fn: (r) => r["_field"] == "a_max" or r["_field"] == "a_min" or r["_field"] == "x" or r["_field"] == "y" or r["_field"] == "z")
  |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
  |> yield(name: "mean")'''

 
# Execute InfluxDB query
result = query_api.query(org=org, query=query)

# Convert the result to a dictionary of dictionaries
data_dict = {}
for table in result:
    for record in table.records:
        timestamp = record.values['_time'].strftime('%Y-%m-%d %H:%M:%S')  # format the timestamp
        robot_name = record.values['robot']
            
        # Ensure robot exists in `robots` table and get its id
        cursor.execute("SELECT id FROM robots WHERE name = %s", (robot_name,))
        robot_id = cursor.fetchone()
        if robot_id is None:
            cursor.execute("INSERT INTO robots (name) VALUES (%s)", (robot_name,))
            conn.commit()
            robot_id = cursor.lastrowid
        else:
            robot_id = robot_id[0]

        measurement = record.values['_measurement']
        if (timestamp, measurement) not in data_dict:
            data_dict[(timestamp, measurement)] = {'robot_id': robot_id, 'timestamp': timestamp}
        # Ensure the _field value exists in the valid_columns list
        valid_columns = {'robot_id', 'timestamp', 'a_min', 'a_max', 'x', 'y', 'z'}
        if record.values['_field'] in valid_columns:
            data_dict[(timestamp, measurement)][record.values['_field']] = record.values['_value']
for timestamp, data in data_dict.items():
    table_name = 'scan_table' if 'a_min' in data or 'a_max' in data else 'odom_table'  # determine the table name
    columns = ', '.join(data.keys())
    placeholders = ', '.join(['%s'] * len(data))
    query = f"INSERT INTO {table_name} ({columns}) VALUES ({placeholders})"
    cursor.execute(query, list(data.values()))

conn.commit()

# Clean up
cursor.close()
conn.close()
client.__del__()
