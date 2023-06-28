#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# need to install influxdb_client via pip

import rospy
import influxdb_monitoring.MessageMonitor as MessageMonitor
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs
import nav_msgs.msg as nav_msgs
import rospkg
import numpy as np

# needed to define class for custom message type
import influxdb_client

class my_laser_scan_class(MessageMonitor.DB_Update):
    """
    Example of how a custom message type can be implemented. Here a laser scan message is used.

    The maximum and minimum range is calculated, and the ranges as well as the directions of those are written to the database.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `my_laser_scan_class` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Stores the max and min range as well as their directions
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "my_laser_scan"
        if message_type != sensor_msgs.LaserScan:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : sensor_msgs.LaserScan):

        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                
                curr_max_range_index = np.argmax(data.ranges)
                curr_max_range = data.ranges[curr_max_range_index]
                curr_min_range_index = np.argmin(data.ranges)
                curr_min_range = data.ranges[curr_min_range_index]

                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("r_max", float(curr_max_range))
                    .field("a_max", float(self.indexToAngle(curr_max_range_index, data)))
                    .field("r_min", float(curr_min_range))
                    .field("a_min", float(self.indexToAngle(curr_min_range_index, data)))
                )
                
                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def indexToAngle(self, index : int, data : sensor_msgs.LaserScan) -> float:
        """
        Helper function to convert an index to a laser scan angle
        """
        return data.angle_min + data.angle_increment * index

    def checkEqual(self, data : sensor_msgs.LaserScan) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return True # TODO: add better equality check.. has to be defined to make sense to your usecase
    

if __name__ == '__main__':

    rospy.init_node("msgs2influxdb")

    # can set robot name tag. is not required
    # MessageMonitor.setRobotName("my_robot")

    # all three methods are possible to specify the influxdb settings:
    # MessageMonitor.setConfig(token="nEE39NAeg8Exa9AQa_yjs02OQMzrDKVUqUZKnrbnXF-f7MesVy1-bY040yNSDupiSRMj_SrRGKAyT5UhEXUCOA==", org="UTM", url="http://127.0.0.1:8086", bucket="FYP")
    MessageMonitor.setConfig(token="UEtQFAPc3cpITlis5JpR0zNjMYIwahBvUp-RIvqJyFjq-qMT5JOgrP716S9C4dcuevYWFqGBoiLKSniCuq37zg==", org="UTM", url="https://us-east-1-1.aws.cloud2.influxdata.com", bucket="rosdata")
    #MessageMonitor.setConfig(filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/config/influxdb_example.json")
    # MessageMonitor.setConfig(docker_compose_filename=rospkg.RosPack().get_path("influxdb_monitoring") + "/docker/docker-compose.yml")


    # a = MessageMonitor.std_msgs_float("/test/float0", std_msgs.Float32, tag_name="some_name", update_period=5)
    # b = MessageMonitor.std_msgs_float("/test/float1", std_msgs.Float64,  update_period=-1, on_change=True)
    # c = MessageMonitor.std_msgs_int("/test/int0", std_msgs.Int8,  update_period=-1, on_change=True)
    # d = MessageMonitor.std_msgs_int("/test/int1", std_msgs.Int16,  update_period=10, on_change=True)
    # e = MessageMonitor.std_msgs_int("/test/int2", std_msgs.Int32,  update_period=10)
    # f = MessageMonitor.std_msgs_int("/test/int3", std_msgs.Int64,  update_period=10)
    # g = MessageMonitor.std_msgs_int("/test/uint0", std_msgs.UInt8,  update_period=-1, on_change=True)
    # h = MessageMonitor.std_msgs_int("/test/uint1", std_msgs.UInt16,  update_period=10, on_change=True)
    # i = MessageMonitor.std_msgs_int("/test/uint2", std_msgs.UInt32,  update_period=10)
    # j = MessageMonitor.std_msgs_int("/test/uint3", std_msgs.UInt64, skip_name=True, measurement_name="uint64_individual", update_period=10)
    
    # k = MessageMonitor.geometry_msgs_pose2d("/test/pose2d", geometry_msgs.Pose2D, on_change=True, publish_increment_distance=True)
    # l = MessageMonitor.geometry_msgs_pose2d("/test/pose2d_2", geometry_msgs.Pose2D, on_change=True, publish_increment_distance=True, publish_cumulative_distance=True, tag_name="another_point")

    # m = MessageMonitor.geometry_msgs_pose2d_from3d("/test/posepseudo2d", geometry_msgs.Pose, update_period=10, publish_cumulative_distance=True)
    # n = MessageMonitor.geometry_msgs_pose("/test/posepseudo2d", geometry_msgs.Pose, update_period=10, publish_cumulative_distance=True)
    # o = MessageMonitor.geometry_msgs_pose("/test/pose", geometry_msgs.Pose, update_period=10, publish_cumulative_distance=True)
    # p = MessageMonitor.geometry_msgs_pose("/test/posestamped", geometry_msgs.PoseStamped, update_period=10, publish_cumulative_distance=True)

    # q = MessageMonitor.geometry_msgs_quaternion_to_rpy("/test/quaternion", geometry_msgs.Quaternion)
    # r = MessageMonitor.geometry_msgs_quaternion_to_rpy("/test/quaternion2d", geometry_msgs.Quaternion, on_change=True)
    # s = MessageMonitor.geometry_msgs_quaternion("/test/quaternion", geometry_msgs.Quaternion, on_change=True)

    # t = MessageMonitor.geometry_msgs_point("/test/point", geometry_msgs.Point, on_change=True)
    # u = MessageMonitor.geometry_msgs_vector3("/test/vec3", geometry_msgs.Vector3, skip_name=True, skip_robot_name_tag=True)

  #  v = MessageMonitor.geometry_msgs_twist("/odom", nav_msgs.Odometry, update_period=5, on_change=True)

    # w = MessageMonitor.std_msgs_bool("/test/bool", std_msgs.Bool, on_change=True)
    # x = MessageMonitor.std_msgs_bool("/test/bool1", std_msgs.Bool, on_change=True)

    # y = MessageMonitor.geometry_msgs_pose_with_covariance("/test/poseWithCovariance", geometry_msgs.PoseWithCovariance, publish_cumulative_distance=True, on_change=True)
    # z = MessageMonitor.geometry_msgs_pose("/test/poseWithCovariance", geometry_msgs.PoseWithCovariance, publish_increment_distance=True)

    # aa = MessageMonitor.sensor_msgs_battery_state("/test/battery", sensor_msgs.BatteryState, on_change=True)


    # using the custom class to subscribe to a sensor_msgs.LaserScan message
    ab = my_laser_scan_class("/tb3_0/scan", sensor_msgs.LaserScan, update_period=1, on_change=True, tag_name="tb3_0")

    ad = MessageMonitor.nav_msgs_odometry("/tb3_0/odom", nav_msgs.Odometry, update_period=1, on_change=True,tag_name="tb3_0")

    ae = MessageMonitor.nav_msgs_odometry("/tb3_1/odom", nav_msgs.Odometry, update_period=1, on_change=True,tag_name="tb3_1")

    ac = my_laser_scan_class("/tb3_1/scan", sensor_msgs.LaserScan, update_period=1, on_change=True, tag_name="tb3_1")


    # ac = MessageMonitor.sensor_msgs_jointstates("/test/jointstates", sensor_msgs.JointState, update_period = 1, on_change=False)



    
    # needed to keep the node alive.
    rospy.spin()
