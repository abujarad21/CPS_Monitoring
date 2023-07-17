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
import nav_msgs.msg as nav_msgs

class DoUpdate:
    def __init__(self, period):
        self.period = period
        self.last_exec = rospy.Time(0)
    def doUpdate(self):
        if (rospy.Time.now() - self.last_exec).to_sec() >= self.period:
            self.last_exec = rospy.Time.now()
            return True
        return False
    
class myRandom:
    def __init__(self):
        raise NotImplementedError
    
    def update(self):
        raise NotImplementedError

class RandomBool(myRandom):
    def __init__(self, init = True, flip_percentage = 0.5):
        self.state = True
        self.flip_percentage = flip_percentage
    
    def update(self):
        if random.random() < self.flip_percentage:
            self.state = not self.state
        return self.state
    
class RandomInt(myRandom):
    def __init__(self, init = 0, min = -10, max = 10, sigma = 2):
        self.value = int(init)
        self.min = int(min)
        self.max = int(max)
        self.sigma = sigma
    
    def update(self):
        self.value = int(float(self.value) + random.gauss(0, self.sigma))
        if (self.value > self.max):
            self.value = self.max
        if (self.value < self.min):
            self.value = self.min
        return self.value

class RandomFloat(myRandom):
    def __init__(self, init = 0, min  = -1, max = 1, sigma = 0.1):
        self.value = init
        self.min = min
        self.max = max
        self.sigma = sigma
    def update(self):
        self.value += random.gauss(0, self.sigma)
        if (self.value > self.max):
            self.value = self.max
        if (self.value < self.min):
            self.value = self.min
        return self.value
    
class RandomPoint(myRandom):
    def __init__(self, x = 0, y = 0, z = 0, min = -1, max=1, sigma=0.1):
        self.x = RandomFloat(x, min, max, sigma)
        self.y = RandomFloat(y, min, max, sigma)
        self.z = RandomFloat(z, min, max, sigma)
    
    def update(self):
        point = geometry_msgs.Point()
        point.x = self.x.update()
        point.y = self.y.update()
        point.z = self.z.update()
        return point
    
class RandomQuaternion(myRandom):
    def __init__(self, roll = 0, pitch = 0, yaw = 0, sigma = 0.15):
        self.roll = RandomFloat(roll, -np.pi, np.pi, sigma)
        self.pitch = RandomFloat(pitch, -np.pi, np.pi, sigma)
        self.yaw = RandomFloat(yaw, -np.pi, np.pi, sigma)
    
    def update(self):
        q = tf.transformations.quaternion_from_euler(self.roll.update(), self.pitch.update(), self.yaw.update())
        quaternion = geometry_msgs.Quaternion()
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
        return quaternion
    
class RandomQuaternion2D(myRandom):
    def __init__(self, yaw = 0, sigma = 0.25):
        self.yaw = RandomFloat(yaw, -np.pi, np.pi, sigma)
    
    def update(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw.update())
        quaternion = geometry_msgs.Quaternion()
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
        return quaternion

class RandomPose(myRandom):
    def __init__(self, x = 0, y = 0, z = 0, min = -1, max = 1, sigma = 0.1, roll=0, pitch=0, yaw=0):
        self.position = RandomPoint(x, y, z, min, max, sigma)
        self.orientation = RandomQuaternion(roll, pitch, yaw)
    
    def update(self):
        pose = geometry_msgs.Pose()
        pose.position = self.position.update()
        pose.orientation = self.orientation.update()
        return pose

class RandomPoseStamped(myRandom):
    def __init__(self, x = 0, y = 0, z = 0, min = -1, max = 1, sigma = 0.1, roll=0, pitch=0, yaw=0):
        self.position = RandomPoint(x, y, z, min, max, sigma)
        self.orientation = RandomQuaternion(roll, pitch, yaw)
    
    def update(self):
        pose = geometry_msgs.PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = self.position.update()
        pose.pose.orientation = self.orientation.update()
        return pose

class RandomPosePseudo2D(myRandom):
    def __init__(self, x = 0, y = 0, z = 0, min = -1, max = 1, sigma = 0.1, yaw=0):
        self.position = RandomPoint(x, y, z, min, max, sigma)
        self.orientation = RandomQuaternion2D(yaw)
    
    def update(self):
        pose = geometry_msgs.Pose()
        pose.position = self.position.update()
        pose.orientation = self.orientation.update()
        return pose
    
class RandomPose2D(myRandom):
    def __init__(self, x = 0, y = 0, yaw = 0, min = -1, max = 1, sigma = 0.1):
        self.x = RandomFloat(x, min, max, sigma)
        self.y = RandomFloat(y, min, max, sigma)
        self.yaw = RandomFloat(yaw, -np.pi, np.pi, 0.25)
    
    def update(self):
        pose2d = geometry_msgs.Pose2D()
        pose2d.x = self.x.update()
        pose2d.y = self.y.update()
        pose2d.theta = self.yaw.update()
        return pose2d
    
class RandomVector3(myRandom):
    def __init__(self, x=0, y=0, z=0, min=-0.1, max=0.1, sigma=0.01):
        self.x = RandomFloat(x, min, max, sigma)
        self.y = RandomFloat(y, min, max, sigma)
        self.z = RandomFloat(z, min, max, sigma)

    def update(self):
        vector_3 = geometry_msgs.Vector3()
        vector_3.x = self.x.update()
        vector_3.y = self.y.update()
        vector_3.z = self.z.update()
        return vector_3
        

class RandomTwist(myRandom):
    def __init__(self, lin = [0, 0, 0], ang = [0, 0, 0], min=-0.1, max=0.1, sigma=0.01):
        self.linear = RandomVector3(lin[0], lin[1], lin[2], min, max, sigma)
        self.angular = RandomVector3(ang[0], ang[1], ang[2], min, max, sigma)
    
    def update(self):
        twist = geometry_msgs.Twist()
        twist.angular = self.angular.update()
        twist.linear = self.linear.update()
        return twist

# helper, not to be published!
class RandomFloatList(myRandom):
    def __init__(self, range_len=36, min=-0.1, max=0.1, sigma=0.01):
        self.cov = []
        for i in range(range_len):
            scale = 0.1
            if i % 7 == 0:
                scale = 1
            self.cov.append(RandomFloat(0, min * scale, max * scale, sigma * scale))
    
    def update(self):
        cov = []
        for cov_ in self.cov:
            cov.append(cov_.update())
        return cov
    
class RandomPoseWithCov(myRandom):
    def __init__(self):
        self.pose = RandomPose()
        self.cov = RandomFloatList(range_len=36)

    def update(self):
        posecov = geometry_msgs.PoseWithCovariance()
        posecov.pose = self.pose.update()
        posecov.covariance = self.cov.update()
        return posecov
    
class RandomBatteryState(myRandom):
    def __init__(self, min_v = 10, max_v=16):
        self.max_v = max_v
        self.min_v = min_v
        self.voltage = random.uniform(min_v, max_v)
        self.charging_bool = RandomBool(flip_percentage=0.1)
        self.last_charging = True

    def update(self):
        battery_msg = sensor_msgs.BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        increment = abs(random.gauss(0.05, 0.05))
        is_charging = self.last_charging and self.voltage < self.max_v*0.98
        if(self.voltage < self.min_v*1.11):
            is_charging = True
        if not is_charging:
            increment = - increment
        self.voltage += increment
        if not is_charging:
            battery_msg.power_supply_status =  sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            if self.voltage >= self.max_v:
                battery_msg.power_supply_status =  sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                battery_msg.power_supply_status =  sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if self.voltage > self.max_v:
            self.voltage = self.max_v
        if self.voltage < self.min_v:
            self.voltage = self.min_v
        battery_msg.voltage = self.voltage
        battery_msg.percentage = ((self.voltage - self.min_v) / (self.max_v - self.min_v))**2
        self.last_charging = is_charging
        return battery_msg
    

class RandomLaserScan(myRandom):
    def __init__(self, sigma = 0.03):
        self.msg = sensor_msgs.LaserScan()
        self.msg.header.frame_id = "laser_frame"

        self.msg.angle_min = -3.14
        self.msg.angle_max = 3.14
        self.msg.angle_increment = 0.0174532923847
        self.msg.time_increment = 0.0
        self.msg.scan_time = 0.1
        self.msg.range_min = 0.1
        self.msg.range_max = 10.0
        self.msg.ranges = [random.uniform(0, 10) for _ in range(360)]
        self.msg.intensities = [random.uniform(0, 1) for _ in range(360)]

        avg = (self.msg.range_max + self.msg.range_min) / 2.0
        self.np_ranges = np.random.uniform(avg - 1, avg + 1, 360)
        self.sigma = sigma

    def update(self):
        self.msg.header.stamp = rospy.Time.now()
        noise = np.random.normal(scale=self.sigma, size=360)
        self.np_ranges += noise

        # Clip the scan to make sure it stays within the valid range
        self.np_ranges = np.clip(self.np_ranges, self.msg.range_min, self.msg.range_max)
        self.msg.ranges = self.np_ranges
        return self.msg
    
class RandomJointState(myRandom):
    def __init__(self, sigma = 0.03, num_joints = 6):
        self.msg = sensor_msgs.JointState()
        self.msg.header.frame_id = ""
        self.num_joints = num_joints

        self.msg.name = [str("joint %d" %i) for i in range(num_joints)]
        self.msg.position = [random.uniform(0, 1) for _ in range(num_joints)]
        self.msg.velocity = [random.uniform(0, 1) for _ in range(num_joints)]
        self.msg.effort = [random.uniform(0, 1) for _ in range(num_joints)]

        self.np_position = np.random.uniform(-1, 1, num_joints)
        self.np_velocity = np.random.uniform(-0.5, 0.5, num_joints)
        self.np_effort = np.random.uniform(0, 1, num_joints)

        self.sigma = sigma

    def update(self):
        self.msg.header.stamp = rospy.Time.now()

        # Clip the scan to make sure it stays within the valid range
        noise = np.random.normal(scale=self.sigma, size=self.num_joints)
        self.np_position += noise
        self.np_position = np.clip(self.np_position, -1, 1)
        self.msg.position = self.np_position

        noise = np.random.normal(scale=self.sigma, size=self.num_joints)
        self.np_velocity += noise
        self.np_velocity = np.clip(self.np_velocity, -0.5, 0.5)
        self.msg.velocity = self.np_velocity

        noise = np.random.normal(scale=self.sigma, size=self.num_joints)
        self.np_effort += noise
        self.np_effort = np.clip(self.np_effort, 0, 1)
        self.msg.effort = self.np_effort
        return self.msg
    

class RandomOdom(myRandom):
    def __init__(self):
        self.position_cov = RandomPoseWithCov()
    
    def update(self):
        odom = nav_msgs.Odometry()
        odom.header.frame_id = ""
        odom.header.stamp = rospy.Time.now()
        odom.pose = self.position_cov.update()
        return odom


        
class HandlePub:
    def __init__(self, publisher : rospy.Publisher, update_checker : DoUpdate, random : myRandom, publish_frequency = 5, skip_update_percentage = 0.3):
        self.publisher = publisher
        self.update_checker = update_checker
        self.random = random
        self.last_message = self.random.update()
        self.last_publish = rospy.Time.now()
        self.publisher.publish(self.last_message)
        self.publish_frequency = publish_frequency
        self.skip_update_percentage = skip_update_percentage

    def checkUpdateAndPublish(self):
        now = rospy.Time.now()
        if random.random() < self.skip_update_percentage:
            return
        if (now - self.last_publish).to_sec() > 1.0/self.publish_frequency:
            if self.update_checker.doUpdate():
                self.last_message = self.random.update()
            self.publisher.publish(self.last_message)
            self.last_publish = now

if __name__ == '__main__':

    rospy.init_node("pseudo_pub")

    handlers = []


    

    handlers.append(HandlePub(rospy.Publisher("/test/bool", std_msgs.Bool, queue_size=1),
                         DoUpdate(1), RandomBool(flip_percentage=0.1), publish_frequency = 20))

    handlers.append(HandlePub(rospy.Publisher("/test/bool1", std_msgs.Bool, queue_size=1),
                         DoUpdate(1), RandomBool(flip_percentage=0.7), publish_frequency = 2))

    handlers.append(HandlePub(rospy.Publisher("/test/int0", std_msgs.Int8, queue_size=1),
                         DoUpdate(2), RandomInt(5), publish_frequency = 3))

    handlers.append(HandlePub(rospy.Publisher("/test/int1", std_msgs.Int16, queue_size=1),
                         DoUpdate(3), RandomInt(5, -100, 100, 5), publish_frequency = 3))

    handlers.append(HandlePub(rospy.Publisher("/test/int2", std_msgs.Int32, queue_size=1),
                         DoUpdate(2.5), RandomInt(5, -500, 500, 15)))

    handlers.append(HandlePub(rospy.Publisher("/test/int3", std_msgs.Int64, queue_size=1),
                         DoUpdate(1.33), RandomInt(5, -5000, 5000, 150), publish_frequency = 8))

    handlers.append(HandlePub(rospy.Publisher("/test/uint0", std_msgs.UInt8, queue_size=1),
                         DoUpdate(0.9), RandomInt(5, 0, 100, 5)))

    handlers.append(HandlePub(rospy.Publisher("/test/uint1", std_msgs.UInt16, queue_size=1),
                         DoUpdate(4.2), RandomInt(5, 0, 1000, 50)))

    handlers.append(HandlePub(rospy.Publisher("/test/uint2", std_msgs.UInt32, queue_size=1),
                         DoUpdate(5.77), RandomInt(5, 0, 5000, 50)))

    handlers.append(HandlePub(rospy.Publisher("/test/uint3", std_msgs.UInt64, queue_size=1),
                         DoUpdate(7), RandomInt(5, 0, 10000, 500)))

    handlers.append(HandlePub(rospy.Publisher("/test/float0", std_msgs.Float32, queue_size=1),
                         DoUpdate(2), RandomFloat(10, 5, 15, 0.1)))

    handlers.append(HandlePub(rospy.Publisher("/test/float1", std_msgs.Float64, queue_size=1),
                    DoUpdate(5),RandomFloat(10, 5, 15, 0.2)))

    handlers.append(HandlePub(rospy.Publisher("/test/point", geometry_msgs.Point, queue_size=1),
                    DoUpdate(1.5),RandomPoint()))

    handlers.append(HandlePub(rospy.Publisher("/test/quaternion", geometry_msgs.Quaternion, queue_size=1),
                    DoUpdate(3.5),RandomQuaternion(), publish_frequency = 10))

    handlers.append(HandlePub(rospy.Publisher("/test/quaternion2d", geometry_msgs.Quaternion, queue_size=1),
                    DoUpdate(2.5),RandomQuaternion2D()))

    handlers.append(HandlePub(rospy.Publisher("/test/pose", geometry_msgs.Pose, queue_size=1),
                    DoUpdate(2),RandomPose(), publish_frequency = 110))

    handlers.append(HandlePub(rospy.Publisher("/test/posepseudo2d", geometry_msgs.Pose, queue_size=1),
                    DoUpdate(4),RandomPosePseudo2D()))

    handlers.append(HandlePub(rospy.Publisher("/test/vec3", geometry_msgs.Vector3, queue_size=1),
                    DoUpdate(2),RandomVector3()))

    handlers.append(HandlePub(rospy.Publisher("/test/twist", geometry_msgs.Twist, queue_size=1),
                    DoUpdate(5),RandomTwist(), publish_frequency = 110))

    handlers.append(HandlePub(rospy.Publisher("/test/poseWithCovariance", geometry_msgs.PoseWithCovariance, queue_size=1),
                    DoUpdate(4),RandomPoseWithCov()))

    handlers.append(HandlePub(rospy.Publisher("/test/pose2d", geometry_msgs.Pose2D, queue_size=1),
                    DoUpdate(2),RandomPose2D()))

    handlers.append(HandlePub(rospy.Publisher("/test/pose2d_2", geometry_msgs.Pose2D, queue_size=1),
                    DoUpdate(1),RandomPose2D(sigma=0.02)))

    handlers.append(HandlePub(rospy.Publisher("/test/posestamped", geometry_msgs.PoseStamped, queue_size=1),
                    DoUpdate(4.5),RandomPoseStamped()))

    handlers.append(HandlePub(rospy.Publisher("/test/battery", sensor_msgs.BatteryState, queue_size=1),
                    DoUpdate(4),RandomBatteryState()))
    
    handlers.append(HandlePub(rospy.Publisher("/test/laserscan", sensor_msgs.LaserScan, queue_size=1),
                    DoUpdate(0.5), RandomLaserScan(0.1), publish_frequency=10, skip_update_percentage=0.1))
    
    handlers.append(HandlePub(rospy.Publisher("/test/jointstates", sensor_msgs.JointState, queue_size=1),
                    DoUpdate(0.5), RandomJointState(), publish_frequency=10, skip_update_percentage=0.1))
    
    handlers.append(HandlePub(rospy.Publisher("/test/odom", nav_msgs.Odometry, queue_size=1),
                    DoUpdate(0.5), RandomOdom(), publish_frequency=10, skip_update_percentage=0.1))



    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        for handler in handlers:
            handler.checkUpdateAndPublish()

        rate.sleep()
