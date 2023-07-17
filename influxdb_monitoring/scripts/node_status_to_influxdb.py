#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip


import rospy, rospkg
import rosnode
import influxdb_monitoring.NodeMonitor as NodeMonitor


if __name__ == '__main__':
    rospy.init_node("node2influxdb")

    # all three methods are possible to specify the influxdb settings:
    #NodeMonitor.setConfig(token="MYTOKEN564651CAFG52AST6FS", org="myOrg", url="http://127.0.0.1:8086", bucket="webdata")
    #NodeMonitor.setConfig(filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/config/influxdb_example.json")
    NodeMonitor.setConfig(docker_compose_filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/docker/docker-compose.yml")

    # can set robot name tag. is not required
    #NodeMonitor.setRobotName("my_robot")
    
    # it's possible to just monitor with "restart_failed=False"
    # this allows simplified config, see "node_config_example_just_monitoring.json"

    NodeMonitor.startMainMonitor(restart_failed=True)


