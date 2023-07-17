#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import rospy, rospkg

import influxdb_monitoring.TopicMonitor as TopicMonitor
from influxdb_monitoring.TopicMonitor import TopicEntry


write_every_x_seconds = 15

topic_names = [TopicEntry('/test/float0', estimated_frequency=5),
               TopicEntry('/test/pose', estimated_frequency=20),
               TopicEntry('/test/quaternion'), 
               TopicEntry('/test/int3', estimated_frequency=8),
               TopicEntry('/test/point', window_size=300), 
               TopicEntry('/test/node/node0', estimated_frequency=50), 
               TopicEntry('/test/node/node1', estimated_frequency=50), 
               TopicEntry('/test/node/node2', estimated_frequency=50), 
               TopicEntry('/test/node/node3', estimated_frequency=50), 
               TopicEntry('/test/node/node4', estimated_frequency=50), 
               TopicEntry('/test/node/node5', estimated_frequency=50), 
               TopicEntry('/test/node/node6', estimated_frequency=50), 
               TopicEntry('/test/node/node7', estimated_frequency=50), 
               TopicEntry('/test/node/node8', estimated_frequency=50), 
               TopicEntry('/test/node/node9', estimated_frequency=50)]


if __name__ == '__main__':
    rospy.init_node("topicHz2influxdb2")

    # all three methods are possible to specify the influxdb settings:
    #TopicMonitor.setConfig(token="MYTOKEN564651CAFG52AST6FS", org="myOrg", url="http://127.0.0.1:8086", bucket="webdata")
    #TopicMonitor.setConfig(filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/config/influxdb_example.json")
    TopicMonitor.setConfig(docker_compose_filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/docker/docker-compose.yml")

    # can set robot name tag. is not required
    #topic_frequency_to_influxdb.setRobotName("my_robot")
    TopicMonitor.startMonitoring(topic_names, 1.0/write_every_x_seconds)
