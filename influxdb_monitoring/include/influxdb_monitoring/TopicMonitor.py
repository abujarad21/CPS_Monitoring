#!/usr/bin/env python
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import typing

import rostopic
import rospy

import influxdb_client
from influxdb_monitoring.InfluxDBConfig import setConfig
import influxdb_monitoring.InfluxDBConfig as InfluxDBConfig

robot_name = None
measurement_name_global = "ros_topics"

def setRobotName(robot_name_):
    """
    Specify a robot name. All InfluxDB updates have this name as a tag.
    """
    global robot_name
    robot_name = robot_name_

class TopicEntry:
    """
    Class storing topics to monitor, as well as an custom windown_size parameter,
    either defined directly or proportional to an estimated frequency."""
    def __init__(self, topic_name, estimated_frequency = None, window_size = 1000):
        self.topic_name = topic_name
        self.estimated_frequency = estimated_frequency
        self.window_size = window_size
    
    def getTopicName(self):
        return self.topic_name
    
    def getWindowSize(self, publish_frequency = None, ):
        if not self.estimated_frequency is None and not publish_frequency is None:
            # design the window size, so that it covers two times the ppublish frequency
            return 2.0 / publish_frequency * self.estimated_frequency
        return self.window_size


def writeToInfluxDB(frequency_data, topic_name, publish_std_dev = True, publish_min_max = True, publish_num  = False):
    try:
        point = (
            influxdb_client.Point(measurement_name_global)
            .tag("topic", topic_name)
            .field("rate", float(frequency_data[0]))
        )
        if publish_std_dev:
            point.field("sd", float(frequency_data[3]))
        if publish_min_max:
            point.field("min", float(frequency_data[1]))
            point.field("max", float(frequency_data[2]))
        if publish_num:
            point.field("num", float(frequency_data[4]))
        if not robot_name is None:
            point.tag("robot", robot_name)
        InfluxDBConfig.write_api.write(bucket=InfluxDBConfig.bucket_g, org=InfluxDBConfig.org_g, record=point)
    except Exception as e:
        rospy.loginfo("Write to influxdb failed: %s"%e)

def startMonitoring(topic_names : typing.List[TopicEntry], publish_frequency : float, publish_std_dev = True, publish_min_max = True, publish_num  = False, measurement_name = "ros_topics"):
    """
    writes topic frequency to InfluxDB. Also allows for standard deviation, min/max and number of messages.
    """
    global measurement_name_global
    if type(measurement_name) == str:
        measurement_name_global = measurement_name
    hz_reporters = []
    hz_subscribers = []
    for i in range(len(topic_names)):
        topic = topic_names[i].getTopicName()
        r = rostopic.ROSTopicHz(topic_names[i].getWindowSize(publish_frequency=publish_frequency))
        hz_reporters.append(r)
        s = rospy.Subscriber(topic, rospy.AnyMsg, r.callback_hz, callback_args=topic)
        hz_subscribers.append(s)
    rospy.loginfo("Started listening to %d topics. Will publish every %.2f seconds." % (len(topic_names), 1.0/publish_frequency))
    rospy.sleep(3)
    rate = rospy.Rate(publish_frequency)
    check_rate = rospy.Rate(max([publish_frequency, 5]))
    while not rospy.is_shutdown():
        time_remaining = max([0.0, rate.remaining().to_sec()])
        updating_string = "       "
        if time_remaining > 0.9 * 1.0/publish_frequency or time_remaining > (1.0/publish_frequency - 2):
            updating_string = "UPDATED"
        print("  remaining: %3.1fs %s " % (time_remaining,  updating_string), end="\r")
        if rate.remaining().to_sec() < 0.1:
            try:
                #report frequencies!
                for i in range(len(topic_names)):
                    #print(topic_names[i][0])
                    topic_feedback = hz_reporters[i].get_hz(topic_names[i].getTopicName())
                    try:
                        topic_feedback[2]
                    except KeyboardInterrupt:
                        raise KeyboardInterrupt
                    except:
                        topic_feedback = [0, 0, 0, 0, 0]
                    #print("  r: %.2f, min/max:[%.4f; %.4f], stddev: %.4f   n: %d" % (topic_feedback[0], topic_feedback[1], topic_feedback[2], topic_feedback[3], topic_feedback[4]))
                    writeToInfluxDB(topic_feedback, topic_names[i].getTopicName(), publish_std_dev = publish_std_dev, publish_min_max = publish_min_max, publish_num  = publish_num)
                    # write to influxdb2
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                rospy.logerr("problem during frequency reporting")
                rospy.logerr(e)
                return
            #rospy.loginfo("New update written to InfluxDB.")
            rate.sleep()
        check_rate.sleep()
    
