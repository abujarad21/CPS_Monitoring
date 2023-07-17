#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import rospy
import std_msgs.msg as std_msgs
import random
import geometry_msgs.msg as geometry_msgs
import tf.transformations
import numpy as np
import sensor_msgs.msg as sensor_msgs
import sys


# set name with
# rosrun influxdb_monitoring pseudo_single_node.py __name:=node_name

if __name__ == '__main__':

    rospy.init_node("pseudo_node")
    topic_name = "/test/node" + rospy.get_name()
    print(topic_name)
    start = rospy.Time.now()


    frequency = round(random.random()*50.0 + 1.0)

    die_percentage = 0.001 / frequency

    pub = rospy.Publisher(topic_name, std_msgs.Float32, queue_size=1)

    rate = rospy.Rate(frequency)
    last_val = 0
    running = True
    while not rospy.is_shutdown() and running:
        try:
            if random.random() < die_percentage:
                running = False
            last_val += random.random()
            pub.publish(last_val)
            rate.sleep()
        except KeyboardInterrupt as e:
            rospy.logerr(e)
            sys.exit()
        except Exception as e:
            rospy.logerr(e)
            sys.exit()
    rospy.loginfo("(Stopped publishing)")
    rospy.sleep(abs(random.gauss(8, 5)))
    duration = (rospy.Time.now() - start).to_sec()
    rospy.logwarn("Node died after %.1f seconds." % duration)