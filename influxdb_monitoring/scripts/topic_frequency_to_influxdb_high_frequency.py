#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import rospy, rospkg

import influxdb_monitoring.TopicMonitor as TopicMonitor
from influxdb_monitoring.TopicMonitor import TopicEntry


# on Ryzen 5600X with 200 simple topics with ~200Hz
# ~11% cpu usage (~140% of a core)
# in comparison, the pseudo_publisher_high_frequency used ~9% (110%)

# with 500 simple topics with ~50Hz
# ~9% (110%) cpu usage topic monitor
# ~8% ( 94%) cpu usage pseudo pub

write_every_x_seconds = 15

if __name__ == '__main__':
    rospy.init_node("topicHz2influxdb2_high_frequency")

    # all three methods are possible to specify the influxdb settings:
    #TopicMonitor.setConfig(token="MYTOKEN564651CAFG52AST6FS", org="myOrg", url="http://127.0.0.1:8086", bucket="webdata")
    #TopicMonitor.setConfig(filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/config/influxdb_example.json")
    TopicMonitor.setConfig(docker_compose_filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/docker/docker-compose.yml")

    topic_pre = "/test/high_freq/no"
    topic_names = []
    for i in range(0, 400):
        topic_names.append(TopicEntry(topic_pre+str(i), estimated_frequency=100))

    # can set robot name tag. is not required
    #topic_frequency_to_influxdb.setRobotName("my_robot")
    TopicMonitor.startMonitoring(topic_names, 1.0/write_every_x_seconds, measurement_name="topic_high_frequency")
