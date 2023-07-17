#!/bin/bash

rosrun influxdb_monitoring start_by_json.py --startConfig "config/node_config_example.json" --filePkg "influxdb_monitoring"

# FOR ROS free watcher
rosrun influxdb_monitoring ros_free_node_watcher.py --startConfig "config/node_config_example.json" --startScript start_monitoring_example.sh --filePkg influxdb_monitoring --sessionName robot_session