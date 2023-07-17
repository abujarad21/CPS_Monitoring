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


if __name__ == '__main__':

    rospy.init_node("pseudo_pub_high_frequency")

    topic_pre = "/test/high_freq/no"
    publishers = []
    val = []
    for i in range(0, 400):
        publishers.append(rospy.Publisher(topic_pre+str(i), std_msgs.Float64, queue_size=1))
        val.append(0)


    rate = rospy.Rate(105)
    while not rospy.is_shutdown():
        try:
            for i in range(0, len(publishers)):
                rand = random.random()
                if (rand > (0.01*(i%10))):
                    if (rand > 0.993):
                        val[i] += random.random() - 0.5
                    publishers[i].publish(val[i])

            rate.sleep()
        except Exception as e:
            rospy.logerr(e)
            sys.exit()