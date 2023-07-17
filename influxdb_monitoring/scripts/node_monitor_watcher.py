#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip


import rospy, rospkg
import rosnode
import influxdb_monitoring.NodeMonitor as NodeMonitor
import os




if __name__ == '__main__':
    rospy.init_node("nodeMonitorWatcher")

    # all three methods are possible to specify the influxdb settings:
    #NodeMonitor.setConfig(token="MYTOKEN564651CAFG52AST6FS", org="myOrg", url="http://127.0.0.1:8086", bucket="webdata")
    #NodeMonitor.setConfig(filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/config/influxdb_example.json")
    NodeMonitor.setConfig(docker_compose_filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/docker/docker-compose.yml")


    # can set robot name tag. is not required
    #NodeMonitor.setRobotName("my_robot")

    # This PID is watched by the ros-free node watcher
    pid = os.getpid()
    f = open(os.path.expanduser("~/.node_watcher_pid"), "w")
    f.write(str(pid))
    f.close()

    NodeMonitor.startSecondaryMonitor(main_monitor_node_name="/node2influxdb", check_every_x_seconds_=5, write_state_every_x_seconds_=60, restart_after_x_cycles_=4)


