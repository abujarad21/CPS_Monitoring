#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import rospy
import influxdb_monitoring.MessageMonitor as MessageMonitor
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs
import rospkg


# on Ryzen 5600X with the given settings at ~100 Hz
# ~4% cpu usage (~55% of a core)


if __name__ == '__main__':

    rospy.init_node("msgs2influxdb_high_frequency")

    # can set robot name tag. is not required
    MessageMonitor.setRobotName("my_robot")

    # all three methods are possible to specify the influxdb settings:
    #MessageMonitor.setConfig(token="MYTOKEN564651CAFG52AST6FS", org="myOrg", url="http://127.0.0.1:8086", bucket="webdata")
    #MessageMonitor.setConfig(filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/config/influxdb_example.json")
    MessageMonitor.setConfig(docker_compose_filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/docker/docker-compose.yml")

    topic_pre = "/test/high_freq/no"
    subscribers = []
    for i in range(0, 50):
        subscribers.append(MessageMonitor.std_msgs_float(topic_pre+str(i), std_msgs.Float64, on_change=True, update_period=-1, measurement_name="high_freq_msgs"))
    for i in range(50, 100):
        subscribers.append(MessageMonitor.std_msgs_float(topic_pre+str(i), std_msgs.Float64, update_period=5, measurement_name="high_freq_msgs"))
    for i in range(100, 150):
        subscribers.append(MessageMonitor.std_msgs_float(topic_pre+str(i), std_msgs.Float64, update_period=15, measurement_name="high_freq_msgs"))
    rospy.spin()
