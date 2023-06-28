from django.core.management.base import BaseCommand, CommandError
from influxdb2mysqldb_app.models import Robots,OdomTable, ScanTable  # assuming your models' names and the app they are in
import influxdb_client
from django.utils import timezone
from datetime import datetime, timedelta
import threading

class Command(BaseCommand):
    help = 'Transfers data from InfluxDB to MySQL'

    def __init__(self, *args, **kwargs):
        super(Command, self).__init__(*args, **kwargs)
        self.client = None
        self.query_api = None

    @staticmethod
    def get_last_processed_timestamp_from_mysql():
        try:
            result = ScanTable.objects.latest('timestamp')
            return result.timestamp
        except ScanTable.DoesNotExist:
            return datetime.utcnow()

    def handle_robot_data(self, robot_name):
        last_processed_timestamp = Command.get_last_processed_timestamp_from_mysql()
        if last_processed_timestamp is None:
            start_timestamp = datetime.utcnow() - timedelta(minutes=15)
        else:
            start_timestamp = last_processed_timestamp
        start_timestamp = start_timestamp.strftime('%Y-%m-%dT%H:%M:%SZ')
        
        query = 'from(bucket: "rosdata")' \
                '    |> range(start: ' + start_timestamp + ', stop: now())' \
                '    |> filter(fn: (r) => r["_measurement"] == "my_laser_scan" or r["_measurement"] == "odometry_pose")' \
                '    |> filter(fn: (r) => r["name"] == "' + robot_name + '")' \
                '    |> filter(fn: (r) => r["_field"] == "a_max" or r["_field"] == "a_min" or r["_field"] == "x" or r["_field"] == "y" or r["_field"] == "z")' \
                '    |> aggregateWindow(every: 5s, fn: mean, createEmpty: false)' \
                '    |> yield(name: "mean")'

        try:
            result = self.query_api.query(org="UTM", query=query)
        except influxdb_client.rest.ApiException as e:
            self.stdout.write(f"Error executing InfluxDB query: {e}")
            return

        data_dict = {}
        for table in result:
            for record in table.records:
                timestamp = record.values['_time']
                timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S')
                try:
                    robot = Robots.objects.get(name=robot_name)
                except Robots.DoesNotExist:
                    self.stdout.write(f"No entry for robot {robot_name} in Robots table. Please make sure it exists.")
                    continue

                measurement = record.values['_measurement']
                if (timestamp_str, measurement) not in data_dict:
                    data_dict[(timestamp_str, measurement)] = {'robot_id': robot.id, 'timestamp': timestamp}

                valid_columns = {'robot', 'timestamp', 'a_min', 'a_max', 'x', 'y', 'z'}
                if record.values['_field'] in valid_columns:
                    data_dict[(timestamp_str, measurement)][record.values['_field']] = record.values['_value']

        for timestamp_str, data in data_dict.items():
            if 'a_min' in data or 'a_max' in data:
                ScanTable.objects.create(**data)
            else:
                OdomTable.objects.create(**data)

    def handle(self, *args, **options):
        bucket = "rosdata"
        org = "UTM"
        token = "UEtQFAPc3cpITlis5JpR0zNjMYIwahBvUp-RIvqJyFjq-qMT5JOgrP716S9C4dcuevYWFqGBoiLKSniCuq37zg=="
        url="https://us-east-1-1.aws.cloud2.influxdata.com"
        self.client = influxdb_client.InfluxDBClient(url=url, token=token, org=org, debug=False)
        self.query_api = self.client.query_api()

        robot_names = ['tb3_0', 'tb3_1']
        threads = []

        for robot_name in robot_names:
            thread = threading.Thread(target=self.handle_robot_data, args=(robot_name,))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()

        self.client.__del__()

# class Command(BaseCommand):
#     help = 'Transfers data from InfluxDB to MySQL'

#     @staticmethod
#     def get_last_processed_timestamp_from_mysql():
#         try:
#             result = ScanTable.objects.latest('timestamp')
#             return result.timestamp
#         except ScanTable.DoesNotExist:
#             return datetime.utcnow()

#     def handle(self, *args, **options):

#         bucket = "rosdata"
#         org = "UTM"
#         token = "UEtQFAPc3cpITlis5JpR0zNjMYIwahBvUp-RIvqJyFjq-qMT5JOgrP716S9C4dcuevYWFqGBoiLKSniCuq37zg=="
#         url="https://us-east-1-1.aws.cloud2.influxdata.com"

#         client = influxdb_client.InfluxDBClient(
#             url=url,
#             token=token,
#             org=org
#         )
#         last_processed_timestamp = Command.get_last_processed_timestamp_from_mysql()

#         if last_processed_timestamp is None:
#             start_timestamp = datetime.utcnow() - timedelta(minutes=15)
#         else:
#             start_timestamp = last_processed_timestamp

#         start_timestamp = start_timestamp.strftime('%Y-%m-%dT%H:%M:%SZ')
#         query_api = client.query_api()


#         query = 'from(bucket: "rosdata")' \
#                 '    |> range(start: ' + start_timestamp + ', stop: now())' \
#                 '    |> filter(fn: (r) => r["_measurement"] == "my_laser_scan" or r["_measurement"] == "odometry_pose")' \
#                 '    |> filter(fn: (r) => r["name"] == "tb3_0" or r["name"] == "tb3_1")' \
#                 '    |> filter(fn: (r) => r["_field"] == "a_max" or r["_field"] == "a_min" or r["_field"] == "x" or r["_field"] == "y" or r["_field"] == "z")' \
#                 '    |> aggregateWindow(every: 5s, fn: mean, createEmpty: false)' \
#                 '    |> yield(name: "mean")'

#         # Execute InfluxDB query
#         # print(query)
#         try:
#             result = query_api.query(org=org, query=query)
#         except influxdb_client.rest.ApiException as e:
#             self.stdout.write(f"Error executing InfluxDB query: {e}")
#             return


#         data_dict = {}
#         for table in result:
#             for record in table.records:
#                 timestamp = record.values['_time']
#                 timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S')  # format the timestamp
#                 robot_name = record.values['name']
#                 # print(robot_name)
#                 # Ensure robot exists in `robots` table and get its id
#             # Get robot id from `robots` table
#                 try:
#                     robot = Robots.objects.get(name=robot_name)
#                 except Robots.DoesNotExist:
#                     self.stdout.write(f"No entry for robot {robot_name} in Robots table. Please make sure it exists.")
#                     continue

#                 measurement = record.values['_measurement']
#                 if (timestamp_str, measurement) not in data_dict:
#                         data_dict[(timestamp_str, measurement)] = {'robot_id': robot.id, 'timestamp': timestamp}
#                   # Ensure the _field value exists in the valid_columns list
#                 valid_columns = {'robot', 'timestamp', 'a_min', 'a_max', 'x', 'y', 'z'}
#                 if record.values['_field'] in valid_columns:
#                     data_dict[(timestamp_str, measurement)][record.values['_field']] = record.values['_value']

#         # ... omitted for brevity ...

#         for timestamp_str, data in data_dict.items():
#             if 'a_min' in data or 'a_max' in data:
#                 ScanTable.objects.create(**data)  # Add data to the scan_table
#             else:
#                 OdomTable.objects.create(**data)  # Add data to the odom_table


#         client.__del__()
# from django.core.management.base import BaseCommand, CommandError
# import influxdb_client
# import mysql.connector
# import numpy as np
# import pandas as pd
# from datetime import datetime


# class Command(BaseCommand):
#     help = 'Transfers data from InfluxDB to MySQL'
#     @staticmethod
#     def get_last_processed_timestamp_from_mysql():
#         # Establish a connection to MySQL
#         conn = mysql.connector.connect(user='root', password='123AsD!@#', host='127.0.0.1', database='rosdb')
#         cursor = conn.cursor()

#         # Execute a query to retrieve the last processed timestamp
#         query = "SELECT MAX(timestamp) FROM odom_table"  # Replace 'your_mysql_table' with the actual table name
#         cursor.execute(query)
#         result = cursor.fetchone()

#         # Close the connection
#         cursor.close()
#         conn.close()

#         # Extract and return the last processed timestamp from the result
#         return result[0] if result else None
    
#     def handle(self, *args, **options):

#         #establishing the connection
#         conn = mysql.connector.connect(user='root', password='123AsD!@#', host='127.0.0.1', database='rosdb')
#         cursor= conn.cursor()
#         # Connect to InfluxDB
#         bucket = "rosdata"
#         org = "UTM"
#         token = "UEtQFAPc3cpITlis5JpR0zNjMYIwahBvUp-RIvqJyFjq-qMT5JOgrP716S9C4dcuevYWFqGBoiLKSniCuq37zg=="
#         url="https://us-east-1-1.aws.cloud2.influxdata.com"

#         client = influxdb_client.InfluxDBClient(
#             url=url,
#             token=token,
#             org=org
#         )
#         last_processed_timestamp = Command.get_last_processed_timestamp_from_mysql()  # Retrieve the last processed timestamp from MySQL
#         start_timestamp = last_processed_timestamp.strftime('%Y-%m-%dT%H:%M:%SZ')
#         query_api = client.query_api()


#         query = 'from(bucket: "rosdata")' \
#                 '    |> range(start: ' + start_timestamp + ', stop: now())' \
#                 '    |> filter(fn: (r) => r["_measurement"] == "my_laser_scan" or r["_measurement"] == "odometry_pose")' \
#                 '    |> filter(fn: (r) => r["name"] == "/scan" or r["name"] == "/odom")' \
#                 '    |> filter(fn: (r) => r["robot"] == "my_robot")' \
#                 '    |> filter(fn: (r) => r["_field"] == "a_max" or r["_field"] == "a_min" or r["_field"] == "x" or r["_field"] == "y" or r["_field"] == "z")' \
#                 '    |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)' \
#                 '    |> yield(name: "mean")'

#         # Execute InfluxDB query
#         try:
#             result = query_api.query(org=org, query=query)
#         except influxdb_client.rest.ApiException as e:
#             self.stdout.write(f"Error executing InfluxDB query: {e}")
#             return
#         # Convert the result to a dictionary of dictionaries
#         data_dict = {}
#         for table in result:
#             for record in table.records:
#                 timestamp = record.values['_time'].strftime('%Y-%m-%d %H:%M:%S')  # format the timestamp
#                 robot_name = record.values['robot']
                    
#                 # Ensure robot exists in `robots` table and get its id
#                 cursor.execute("SELECT id FROM robots WHERE name = %s", (robot_name,))
#                 robot_id = cursor.fetchone()
#                 if robot_id is None:
#                     cursor.execute("INSERT INTO robots (name) VALUES (%s)", (robot_name,))
#                     conn.commit()
#                     robot_id = cursor.lastrowid
#                 else:
#                     robot_id = robot_id[0]

#                 measurement = record.values['_measurement']
#                 if (timestamp, measurement) not in data_dict:
#                     data_dict[(timestamp, measurement)] = {'robot_id': robot_id, 'timestamp': timestamp}
#                 # Ensure the _field value exists in the valid_columns list
#                 valid_columns = {'robot_id', 'timestamp', 'a_min', 'a_max', 'x', 'y', 'z'}
#                 if record.values['_field'] in valid_columns:
#                     data_dict[(timestamp, measurement)][record.values['_field']] = record.values['_value']
#         for timestamp, data in data_dict.items():
#             table_name = 'scan_table' if 'a_min' in data or 'a_max' in data else 'odom_table'  # determine the table name
#             columns = ', '.join(data.keys())
#             placeholders = ', '.join(['%s'] * len(data))
#             query = f"INSERT INTO {table_name} ({columns}) VALUES ({placeholders})"
#             cursor.execute(query, list(data.values()))

#         conn.commit()

#         # Clean up
#         cursor.close()
#         conn.close()
#         client.__del__()
