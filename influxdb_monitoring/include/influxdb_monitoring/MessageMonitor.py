#!/usr/bin/env python
# -*- coding: utf-8 -*-

import influxdb_client
import geometry_msgs.msg as geometry_msgs
import tf.transformations
import rospy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import numpy as np
import sensor_msgs.msg as sensor_msgs
import nav_msgs.msg as nav_msgs

from typing import Union

from influxdb_monitoring.InfluxDBConfig import setConfig
import influxdb_monitoring.InfluxDBConfig as InfluxDBConfig


robot_name = None

def setRobotName(robot_name_):
    """
    Specify a robot name. All InfluxDB updates have this name as a tag.
    """
    global robot_name
    robot_name = robot_name_

class DB_Update:
    """
    A class used to inherit from by all needed ROS message types in order to 
    save a ROS message to InfluxDB.
    @param topic_name 
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False, epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `DB_Update` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
        
        self.topic_name = topic_name
        self.message_type = message_type
        if measurement_name is None:
            self.measurement_name = topic_name
        else:
            self.measurement_name = measurement_name
        if tag_name is None:
            self.tag_name = topic_name
        else:
            self.tag_name = tag_name
        self.update_period = update_period
        self.on_change = on_change
        if (self.update_period < 0):
            self.on_change = True
        self.last_check = rospy.Time(0)
        self.last_data = None
        self.epsilon = epsilon
        self.skip_name = skip_name
        self.skip_robot_name_tag = skip_robot_name_tag
        try:
            self.subscriber = rospy.Subscriber(topic_name, message_type, self.callbackCheck, queue_size=1)
            rospy.loginfo("Created subscriber to topic %s of type %r" % (topic_name, message_type.__name__))
        except:
            rospy.logerr("Could not subscibe to topic %s of type %r" % (topic_name, message_type.__name__))




    def callbackCheck(self, data):
        """
        The callback function passed to the subscriber. Specifies how the ROS message is represented in the database,
          i.e. which fields are stored with what tag.
        Has to be overwritten for every child class.

        Args:
        - data: ROS message data

        Returns: None
        """
        raise NotImplementedError
    
    def checkEqual(self, data) -> bool:
        """
        Function to check if the data is equal (possibly within bounds for float types) to the last data.
        Has to be overwritten for every child class.

        Args:
        - data: ROS message data

        Returns: boolean, `True` if data is equal to last data, `False` otherwise
        """
        raise NotImplementedError
    
    def writeDBPoint(self, db_point : influxdb_client.Point):
        """
        Writes a db_point to the influx database.

        Args:
        - db_point: InfluxDB point to write to the database

        Returns: None
        """
        if not robot_name is None and not self.skip_robot_name_tag:
            db_point.tag("robot", robot_name)
        if (not self.skip_name):
            db_point.tag("name", self.tag_name)
        InfluxDBConfig.write_api.write(bucket=InfluxDBConfig.bucket_g, org=InfluxDBConfig.org_g, record=db_point)
        self.on_write_point()

    def floatEqual(self, a, b) -> bool:
        """
        Check if two floating point numbers are equal within a tolerance.

        Args:
        - a: the first floating point number
        - b: the second floating point number

        Returns:
        - True if the two numbers are equal within the tolerance, False otherwise
        """
        return abs(a - b) < self.epsilon
    
    def quaternionEqual(self, q1_msg : geometry_msgs.Quaternion, q2_msg : geometry_msgs.Quaternion) -> bool:
        """
        Check if two quaternions are equal within a tolerance.

        Args:
        - q1_msg: the first quaternion as a ROS message
        - q2_msg: the second quaternion as a ROS message

        Returns:
        - True if the two quaternions are equal within the tolerance, False otherwise
        """
        q1 = [q1_msg.x, q1_msg.y, q1_msg.z, q1_msg.w]
        q2 = [q2_msg.x, q2_msg.y, q2_msg.z, q2_msg.w]
        angles = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(q1, tf.transformations.quaternion_inverse(q2)))
        return abs(angles[0]) < self.epsilon and abs(angles[1]) < self.epsilon and abs(angles[2]) < self.epsilon
    
    def yawEqual(self, yaw1, yaw2) -> bool:
        """
        Check if two yaw angles are equal within a tolerance.

        Args:
        - yaw1: the first yaw angle
        - yaw2: the second yaw angle

        Returns:
        - True if the two yaw angles are equal within the tolerance, False otherwise
        """
        q1 = tf.transformations.quaternion_from_euler(0, 0, yaw1)
        q2 = tf.transformations.quaternion_from_euler(0, 0, yaw2)
        angles = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(q1, tf.transformations.quaternion_inverse(q2)))
        return abs(angles[2]) < self.epsilon
    
    def on_last_data_not_avail(self):
        rospy.logwarn("last data not available (yet).\n%s" % self.topic_name)

    def on_callbackCheck_exception(self, exception):
        rospy.logerr("Problem during callbackCheck of %s" % self.topic_name)
        rospy.logerr(exception)
        # pass

    def on_write_point(self):
        print("Wrote %s to db." %  self.topic_name)



    
class std_msgs_float(DB_Update):
    """
    Derivative class of DB_Update for float type messages"""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False, epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `std_msgs_float` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
            measurement_name = "floats"
        if message_type != std_msgs.Float32 and message_type != std_msgs.Float64:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : std_msgs.Float32):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("value", float(data.data))
                )
                
                    
                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.data, self.last_data.data)
    
class std_msgs_int(DB_Update):
    """
    Derivative class of DB_Update for (unsigned) int type messages"""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False, epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `std_msgs_int` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
            measurement_name = "ints"
        if message_type != std_msgs.Int8 and message_type != std_msgs.Int16 and message_type != std_msgs.Int32 and message_type != std_msgs.Int64 and \
           message_type != std_msgs.UInt8 and message_type != std_msgs.UInt16 and message_type != std_msgs.UInt32 and message_type != std_msgs.UInt64:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : std_msgs.Int32):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("value", int(data.data))
                )
                
                    
                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.data, self.last_data.data)
    
class std_msgs_bool(DB_Update):
    """
    Derivative class of DB_Update for bool type messages"""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False, epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `std_msgs_bool` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
            measurement_name = "bools"
        if message_type != std_msgs.Bool:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : std_msgs.Bool):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("value", bool(data.data))
                )
                
                    
                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data:std_msgs.Bool) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return data.data == self.last_data.data
    
class geometry_msgs_pose2d(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Pose2D type messages.
    Can publish cumulative distance to set precision (default 1e-6 m)."""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, publish_increment_distance = False, publish_cumulative_distance = False, cumulative_distance_precision = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_pose2d` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Can publish cumulative distance to set precision (default 1e-6 m) as well as the incremental distance to the last update.
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - publish_increment_distance (bool): flag to publish the incremental distance (default is `False`)
        - publish_cumulative_distance (bool): flag to publish the cumulative distance (default is `False`)
        - cumulative_distance_precision (float): precision for the cumulative distance (default 1e-6 m)
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "pose2d"
        if message_type != geometry_msgs.Pose2D:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)
        self.publish_increment_distance = publish_increment_distance
        self.publish_cumulative_distance = publish_cumulative_distance
        self.cumulative_distance =  np.uint64(0)
        self.cumulative_distance_precision = cumulative_distance_precision

    def callbackCheck(self, data : geometry_msgs.Pose2D):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.x))
                    .field("y", float(data.y))
                    .field("theta", float(data.theta))
                )
                

                if (self.publish_cumulative_distance or self.publish_increment_distance) and not self.last_data is None:
                    distance = np.sqrt(np.sum(np.power([data.x - self.last_data.x, data.y  - self.last_data.y], [2, 2])))
                    if (self.publish_increment_distance):
                        point.field("inc_dist", float(distance))
                    if (self.publish_cumulative_distance):
                        distance_inc = np.uint64(distance/self.cumulative_distance_precision)
                        self.cumulative_distance += distance_inc
                        point.field("cum_dist", float(self.cumulative_distance * self.cumulative_distance_precision))


                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change or self.publish_cumulative_distance or self.publish_increment_distance:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Pose2D) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.x, self.last_data.x) and self.floatEqual(data.y, self.last_data.y) and self.yawEqual(data.theta, self.last_data.theta)
    
class geometry_msgs_pose2d_from3d(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Pose type messages that contain only 2D information.
    Can publish cumulative distance to set precision (default 1e-6 m)."""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, publish_increment_distance = False, publish_cumulative_distance = False, cumulative_distance_precision = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_pose2d_from3d` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Can publish cumulative distance to set precision (default 1e-6 m) as well as the incremental distance to the last update.
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - publish_increment_distance (bool): flag to publish the incremental distance (default is `False`)
        - publish_cumulative_distance (bool): flag to publish the cumulative distance (default is `False`)
        - cumulative_distance_precision (float): precision for the cumulative distance (default 1e-6 m)
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "pose2d"
        if message_type != geometry_msgs.Pose and message_type != geometry_msgs.PoseStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)
        self.publish_increment_distance = publish_increment_distance
        self.publish_cumulative_distance = publish_cumulative_distance
        self.cumulative_distance =  np.uint64(0)
        self.cumulative_distance_precision = cumulative_distance_precision

    def callbackCheck(self, data : Union[geometry_msgs.Pose, geometry_msgs.PoseStamped]):
        if (data._has_header):
            data = data.pose
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                
                q_orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
                angles = tf.transformations.euler_from_quaternion(q_orientation)
                theta = angles[2]
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.position.x))
                    .field("y", float(data.position.y))
                    .field("theta", float(theta))
                )
                

                if (self.publish_cumulative_distance or self.publish_increment_distance) and not self.last_data is None:
                    distance = np.sqrt(np.sum(np.power([data.position.x - self.last_data.position.x, data.position.y  - self.last_data.position.y], [2, 2])))
                    if (self.publish_increment_distance):
                        point.field("inc_dist", float(distance))
                    if (self.publish_cumulative_distance):
                        distance_inc = np.uint64(distance/self.cumulative_distance_precision)
                        self.cumulative_distance += distance_inc
                        point.field("cum_dist", float(self.cumulative_distance * self.cumulative_distance_precision))


                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change or self.publish_cumulative_distance or self.publish_increment_distance:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Pose) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.position.x, self.last_data.position.x) and self.floatEqual(data.position.y, self.last_data.position.y) and \
               self.floatEqual(data.position.z, self.last_data.position.z) and self.quaternionEqual(data.orientation, self.last_data.orientation)
    
class geometry_msgs_pose(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Pose type messages.
    Can publish cumulative distance to set precision (default 1e-6 m)."""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, publish_increment_distance = False, publish_cumulative_distance = False, cumulative_distance_precision = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_pose` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Can publish cumulative distance to set precision (default 1e-6 m) as well as the incremental distance to the last update.
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - publish_increment_distance (bool): flag to publish the incremental distance (default is `False`)
        - publish_cumulative_distance (bool): flag to publish the cumulative distance (default is `False`)
        - cumulative_distance_precision (float): precision for the cumulative distance (default 1e-6 m)
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "pose"
        if message_type != geometry_msgs.Pose and message_type != geometry_msgs.PoseStamped and message_type != geometry_msgs.PoseWithCovariance and message_type != geometry_msgs.PoseWithCovarianceStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)
        self.publish_increment_distance = publish_increment_distance
        self.publish_cumulative_distance = publish_cumulative_distance
        self.cumulative_distance =  np.uint64(0)
        self.cumulative_distance_precision = cumulative_distance_precision

    def callbackCheck(self, data : Union[geometry_msgs.Pose, geometry_msgs.PoseStamped, geometry_msgs.PoseWithCovariance, geometry_msgs.PoseWithCovarianceStamped]):
        if (data._has_header):
            data = data.pose
        if (type(data)==geometry_msgs.PoseWithCovariance):
            data = data.pose
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                
                q_orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
                angles = tf.transformations.euler_from_quaternion(q_orientation)
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.position.x))
                    .field("y", float(data.position.y))
                    .field("z", float(data.position.y))
                    .field("roll", float(angles[0]))
                    .field("pitch", float(angles[1]))
                    .field("yaw", float(angles[2]))
                )
                

                if (self.publish_cumulative_distance or self.publish_increment_distance) and not self.last_data is None:
                    distance = np.sqrt(np.sum(np.power([data.position.x - self.last_data.position.x, data.position.y  - self.last_data.position.y, data.position.z  - self.last_data.position.z], [2, 2, 2])))
                    if (self.publish_increment_distance):
                        point.field("inc_dist", float(distance))
                    if (self.publish_cumulative_distance):
                        distance_inc = np.uint64(distance/self.cumulative_distance_precision)
                        self.cumulative_distance += distance_inc
                        point.field("cum_dist", float(self.cumulative_distance * self.cumulative_distance_precision))


                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change or self.publish_cumulative_distance or self.publish_increment_distance:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Pose) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.position.x, self.last_data.position.x) and self.floatEqual(data.position.y, self.last_data.position.y) and \
               self.floatEqual(data.position.z, self.last_data.position.z) and self.quaternionEqual(data.orientation, self.last_data.orientation)
    
class geometry_msgs_pose_with_covariance(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Pose type messages.
    Can publish cumulative distance to set precision (default 1e-6 m)."""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, publish_increment_distance = False, publish_cumulative_distance = False, cumulative_distance_precision = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_pose_with_covariance` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Can publish cumulative distance to set precision (default 1e-6 m) as well as the incremental distance to the last update.
        Also publishes the main-diagonal entries of the covariance.
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - publish_increment_distance (bool): flag to publish the incremental distance (default is `False`)
        - publish_cumulative_distance (bool): flag to publish the cumulative distance (default is `False`)
        - cumulative_distance_precision (float): precision for the cumulative distance (default 1e-6 m)
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "pose_cov"
        if message_type != geometry_msgs.PoseWithCovariance and message_type != geometry_msgs.PoseWithCovarianceStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)
        self.publish_increment_distance = publish_increment_distance
        self.publish_cumulative_distance = publish_cumulative_distance
        self.cumulative_distance =  np.uint64(0)
        self.cumulative_distance_precision = cumulative_distance_precision

    def callbackCheck(self, data : Union[geometry_msgs.PoseWithCovariance, geometry_msgs.PoseWithCovarianceStamped]):
        if (data._has_header):
            data = data.pose
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                
                q_orientation = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
                angles = tf.transformations.euler_from_quaternion(q_orientation)
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.pose.position.x))
                    .field("y", float(data.pose.position.y))
                    .field("z", float(data.pose.position.y))
                    .field("roll", float(angles[0]))
                    .field("pitch", float(angles[1]))
                    .field("yaw", float(angles[2]))
                    .field("xx", float(data.covariance[0]))
                    .field("yy", float(data.covariance[7]))
                    .field("zz", float(data.covariance[14]))
                    .field("rollroll", float(data.covariance[21]))
                    .field("pitchpitch", float(data.covariance[28]))
                    .field("yawyaw", float(data.covariance[35]))
                )
                

                if (self.publish_cumulative_distance or self.publish_increment_distance) and not self.last_data is None:
                    distance = np.sqrt(np.sum(np.power([data.pose.position.x - self.last_data.pose.position.x, data.pose.position.y  - self.last_data.pose.position.y, data.pose.position.z  - self.last_data.pose.position.z], [2, 2, 2])))
                    if (self.publish_increment_distance):
                        point.field("inc_dist", float(distance))
                    if (self.publish_cumulative_distance):
                        distance_inc = np.uint64(distance/self.cumulative_distance_precision)
                        self.cumulative_distance += distance_inc
                        point.field("cum_dist", float(self.cumulative_distance * self.cumulative_distance_precision))


                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change or self.publish_cumulative_distance or self.publish_increment_distance:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.PoseWithCovariance) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.pose.position.x, self.last_data.pose.position.x) and self.floatEqual(data.pose.position.y, self.last_data.pose.position.y) and \
               self.floatEqual(data.pose.position.z, self.last_data.pose.position.z) and self.quaternionEqual(data.pose.orientation, self.last_data.pose.orientation)
    
class geometry_msgs_quaternion(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Quaternion type messages, stores it in x, y, z, w.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_quaternion` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Stores the quaternion in x, y, z, w
        
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
            measurement_name = "quaternion"
        if message_type != geometry_msgs.Quaternion and message_type != geometry_msgs.QuaternionStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : Union[geometry_msgs.Quaternion, geometry_msgs.QuaternionStamped]):
        if (data._has_header):
            data = data.quaternion
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.x))
                    .field("y", float(data.y))
                    .field("z", float(data.z))
                    .field("w", float(data.w))
                )
                

                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Quaternion) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.quaternionEqual(data, self.last_data)
    
class geometry_msgs_quaternion_to_rpy(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Quaternion type messages, stores it in roll, pitch, yaw.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_quaternion_to_rpy` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Stores the quaternion in roll, pitch and yaw.
        
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
            measurement_name = "orientation"
        if message_type != geometry_msgs.Quaternion and message_type != geometry_msgs.QuaternionStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : Union[geometry_msgs.Quaternion, geometry_msgs.QuaternionStamped]):
        if (data._has_header):
            data = data.quaternion
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                
                q_orientation = [data.x, data.y, data.z, data.w]
                angles = tf.transformations.euler_from_quaternion(q_orientation)
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("roll", float(angles[0]))
                    .field("pitch", float(angles[1]))
                    .field("yaw", float(angles[2]))
                )
                

                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Quaternion) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.quaternionEqual(data, self.last_data)
    
class geometry_msgs_point(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Point type messages.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_point` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
            measurement_name = "point"
        if message_type != geometry_msgs.Point and message_type != geometry_msgs.Point32 and message_type != geometry_msgs.PointStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : Union[geometry_msgs.Point, geometry_msgs.Point32, geometry_msgs.PointStamped]):
        if (data._has_header):
            data = data.point
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.x))
                    .field("y", float(data.y))
                    .field("z", float(data.z))
                )
                

                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Point) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.x, self.last_data.x) and self.floatEqual(data.y, self.last_data.y) and \
               self.floatEqual(data.z, self.last_data.z)
    
class geometry_msgs_vector3(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Vector3 type messages.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_point` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
            measurement_name = "vector3"
        if message_type != geometry_msgs.Vector3 and message_type != geometry_msgs.Vector3Stamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : Union[geometry_msgs.Vector3, geometry_msgs.Vector3Stamped]):
        if (data._has_header):
            data = data.vector
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.x))
                    .field("y", float(data.y))
                    .field("z", float(data.z))
                )
                

                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Vector3) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.x, self.last_data.x) and self.floatEqual(data.y, self.last_data.y) and \
               self.floatEqual(data.z, self.last_data.z)
    
class geometry_msgs_twist(DB_Update):
    """
    Derivative class of DB_Update for geometry_msgs.Twist type messages.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `geometry_msgs_point` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
            measurement_name = "twist"
        if message_type != geometry_msgs.Twist and message_type != geometry_msgs.TwistStamped and \
           message_type != geometry_msgs.TwistWithCovariance and message_type != geometry_msgs.TwistWithCovarianceStamped:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

    def callbackCheck(self, data : Union[geometry_msgs.Twist, geometry_msgs.TwistStamped, geometry_msgs.TwistWithCovariance, geometry_msgs.TwistWithCovarianceStamped]):
        if (data._has_header):
            data = data.twist
        if (type(data) == geometry_msgs.TwistWithCovariance):
            data = data.twist
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("lin_x", float(data.linear.x))
                    .field("lin_y", float(data.linear.y))
                    .field("lin_z", float(data.linear.z))
                    .field("ang_x", float(data.angular.x))
                    .field("ang_y", float(data.angular.y))
                    .field("ang_z", float(data.angular.z))
                )
                

                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : geometry_msgs.Twist) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.linear.x, self.last_data.linear.x) and self.floatEqual(data.linear.y, self.last_data.linear.y) and \
               self.floatEqual(data.linear.z, self.last_data.linear.z) and self.floatEqual(data.angular.x, self.last_data.angular.x) and \
               self.floatEqual(data.angular.y, self.last_data.angular.y) and self.floatEqual(data.angular.z, self.last_data.angular.z)
    
class sensor_msgs_battery_state(DB_Update):
    """
    Derivative class of DB_Update for sensor_msgs.BatteryState type messages.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, publish_percentage = True, publish_charge = False, publish_current  = False, publish_capacity = False, publish_supply_status = True,
                  publish_supply_health = False, publish_temperature = False, skip_robot_name_tag=False):
        """
        Constructor for the `sensor_msgs_battery_state` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - publish_percentage (bool): Whether to publish the percentage field to the database. Default is True.
        - publish_charge (bool): Whether to publish the charge field to the database. Default is False.
        - publish_current (bool): Whether to publish the current field to the database. Default is False.
        - publish_capacity (bool): Whether to publish the capacity field to the database. Default is False.
        - publish_supply_status (bool): Whether to publish the power supply status field to the database. Default is True.
        - publish_supply_health (bool): Whether to publish the power supply health field to the database. Default is False.
        - publish_temperature (bool): Whether to publish the temperature field to the database. Default is False.
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "battery"
        if message_type != sensor_msgs.BatteryState:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)

        self.publish_percentage = publish_percentage
        self.publish_charge = publish_charge
        self.publish_current = publish_current
        self.publish_capacity = publish_capacity
        self.publish_supply_status = publish_supply_status
        self.publish_supply_health = publish_supply_health
        self.publish_temperature = publish_temperature

    def callbackCheck(self, data : sensor_msgs.BatteryState):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("voltage", float(data.voltage))
                )
                if self.publish_percentage:
                    point.field("percentage", float(data.percentage))
                if self.publish_charge:
                    point.field("charge", float(data.charge))
                if self.publish_current:
                    point.field("current", float(data.current))
                if self.publish_capacity:
                    point.field("capacity", float(data.capacity))
                if self.publish_supply_status:
                    point.field("status", int(data.power_supply_status))
                if self.publish_supply_health:
                    point.field("health", int(data.power_supply_health))
                if self.publish_temperature:
                    point.field("temperature", float(data.temperature))

                

                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : sensor_msgs.BatteryState) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        state = self.floatEqual(data.voltage, self.last_data.voltage)
        if self.publish_percentage:
            state = state and self.floatEqual(data.percentage, self.last_data.percentage)
        if self.publish_charge:
            state = state and self.floatEqual(data.charge, self.last_data.charge)
        if self.publish_current:
            state = state and self.floatEqual(data.current, self.last_data.current)
        if self.publish_capacity:
            state = state and self.floatEqual(data.capacity, self.last_data.capacity)
        if self.publish_supply_status:
            state = state and data.power_supply_status == self.last_data.power_supply_status
        if self.publish_supply_health:
            state = state and data.power_supply_health == self.last_data.power_supply_health
        if self.publish_temperature:
            state = state and self.floatEqual(data.temperature, self.last_data.temperature)
        return state


class sensor_msgs_jointstates(DB_Update):
    """
    Derivative class of DB_Update for sensor_msgs.JointState type messages.
    """
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, skip_robot_name_tag=False, add_velocities = True, add_effort = True):
        """
        Constructor for the `sensor_msgs_jointstates` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        
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
        - add_velocities (bool): whether or not to include the velocity measurements
        - add_effort (bool): whether or not to include the effort measurements

        Returns: None
        """
        self.add_velocities = add_velocities
        self.add_effort = add_effort
        if measurement_name is None:
            measurement_name = "jointstates"
        if message_type != sensor_msgs.JointState:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)
        
        self.last_data_dict = {}
        self.last_check_dict = {}

    def callbackCheck(self, data : sensor_msgs.JointState):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.get_last_check_time(data)).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                for i in range(len(data.position)):
                    point = (
                        influxdb_client.Point(self.measurement_name)
                        .field("position", float(data.position[i]))
                        .tag("joint_name", data.name[i])
                    )
                    if self.add_velocities:
                        try:
                            point.field("velocity", float(data.velocity[i]))
                        except:
                            pass
                    if self.add_effort:
                        try:
                            point.field("effort", float(data.effort[i]))
                        except:
                            pass
                    
                    self.writeDBPoint(point)
                    self.last_check = time_now
                    self.last_check_dict[data.name[0]] = time_now
                    if self.on_change:
                        self.last_data = data
                        # this is needed, as somehow joint state updates from arm and mercator base come in different messages sometimes..
                        self.last_data_dict[data.name[0]] = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def get_last_check_time(self, data : sensor_msgs.JointState) -> rospy.Time:
        try:
            return self.last_check_dict[data.name[0]]
        except:
            return rospy.Time(0)

    def checkEqual(self, data : sensor_msgs.JointState) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        
        status = True
        try:
            # this is needed, as somehow joint state updates from arm and mercator base come in different messages sometimes..
            local_last_data = self.last_data_dict[data.name[0]]

            for i in range(len(data.position)):
                if (not self.floatEqual(data.position[i], local_last_data.position[i])):
                    status = False
        except:
            return False
        return status
    


class nav_msgs_odometry(DB_Update):
    """
    Derivative class of DB_Update for nav_msgs.Odometry type messages.
    Can publish cumulative distance to set precision (default 1e-6 m)."""
    def __init__(self, topic_name, message_type, update_period = 30, measurement_name = None, tag_name = None, skip_name=False, on_change=False,
                  epsilon = 1e-6, publish_increment_distance = False, publish_cumulative_distance = False, cumulative_distance_precision = 1e-6, skip_robot_name_tag=False):
        """
        Constructor for the `nav_msgs_odometry` class.
        Creates the DB_Update object, including the subscriber to the ros topic.
        Can publish cumulative distance to set precision (default 1e-6 m) as well as the incremental distance to the last update.
        Also publishes the main-diagonal entries of the covariance.
        
        Args:
        - topic_name (str): Full name of the ROS topic to subscribe to
        - message_type (ros msg): message type of the ROS topic
        - update_period (float): time interval for checking data changes in seconds (default 30 seconds)
        - measurement_name (str): name of the measurement in the database (default is the same as `topic_name`)
        - tag_name (str): Value of 'name' tag in InfluxDB (default is the same as `topic_name`)
        - skip_name (bool): flag to skip adding the `tag_name` to the database
        - on_change (bool): flag to update the database immediately on change (default is `False`)
        - epsilon (float): tolerance level for floating-point comparisons in checkEqual (default 1e-6)
        - publish_increment_distance (bool): flag to publish the incremental distance (default is `False`)
        - publish_cumulative_distance (bool): flag to publish the cumulative distance (default is `False`)
        - cumulative_distance_precision (float): precision for the cumulative distance (default 1e-6 m)
        - skip_robot_name_tag (bool): flag to skip adding a robot name tag to the database

        Returns: None
        """
        if measurement_name is None:
            measurement_name = "odometry_pose"
        if message_type != nav_msgs.Odometry:
            rospy.logerr("Invalid message type specified for %s", topic_name)
            raise TypeError
        super().__init__(topic_name=topic_name, message_type=message_type, update_period=update_period, measurement_name=measurement_name, tag_name=tag_name, 
                         skip_name=skip_name, on_change=on_change, epsilon=epsilon, skip_robot_name_tag=skip_robot_name_tag)
        self.publish_increment_distance = publish_increment_distance
        self.publish_cumulative_distance = publish_cumulative_distance
        self.cumulative_distance =  np.uint64(0)
        self.cumulative_distance_precision = cumulative_distance_precision

    def callbackCheck(self, data : nav_msgs.Odometry):
        time_now = rospy.Time.now()
        if (self.update_period >= 0 and (time_now - self.last_check).to_sec() > self.update_period) or \
           (self.on_change and not self.checkEqual(data)):
            try:
                
                q_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
                angles = tf.transformations.euler_from_quaternion(q_orientation)
                point = (
                    influxdb_client.Point(self.measurement_name)
                    .field("x", float(data.pose.pose.position.x))
                    .field("y", float(data.pose.pose.position.y))
                    .field("z", float(data.pose.pose.position.z))
                    .field("roll", float(angles[0]))
                    .field("pitch", float(angles[1]))
                    .field("yaw", float(angles[2]))
                    .field("xx", float(data.twist.twist.linear.x))
                    .field("yy", float(data.twist.twist.linear.y))
                    .field("zz", float(data.twist.twist.linear.z))
                    # .field("xx", float(data.pose.covariance[0]))
                    # .field("yy", float(data.pose.covariance[7]))
                    # .field("zz", float(data.pose.covariance[14]))
                    .field("rollroll", float(data.pose.covariance[21]))
                    .field("pitchpitch", float(data.pose.covariance[28]))
                    .field("yawyaw", float(data.pose.covariance[35]))
                )
                

                if (self.publish_cumulative_distance or self.publish_increment_distance) and not self.last_data is None:
                    distance = np.sqrt(np.sum(np.power([data.pose.pose.position.x - self.last_data.pose.pose.position.x, data.pose.pose.position.y  - self.last_data.pose.pose.position.y, data.pose.pose.position.z  - self.last_data.pose.pose.position.z], [2, 2, 2])))
                    if (self.publish_increment_distance):
                        point.field("inc_dist", float(distance))
                    if (self.publish_cumulative_distance):
                        distance_inc = np.uint64(distance/self.cumulative_distance_precision)
                        self.cumulative_distance += distance_inc
                        point.field("cum_dist", float(self.cumulative_distance * self.cumulative_distance_precision))


                self.writeDBPoint(point)
                self.last_check = time_now
                if self.on_change or self.publish_cumulative_distance or self.publish_increment_distance:
                    self.last_data = data

            except Exception as e:
                self.on_callbackCheck_exception(e)

    def checkEqual(self, data : nav_msgs.Odometry) -> bool:
        if self.last_data is None:
            self.on_last_data_not_avail()
            return False
        return self.floatEqual(data.pose.pose.position.x, self.last_data.pose.pose.position.x) and self.floatEqual(data.pose.pose.position.y, self.last_data.pose.pose.position.y) and \
               self.floatEqual(data.pose.pose.position.z, self.last_data.pose.pose.position.z) and self.quaternionEqual(data.pose.pose.orientation, self.last_data.pose.pose.orientation)
    