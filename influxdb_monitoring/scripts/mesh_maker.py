#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

rospy.init_node('mesh_marker')

marker_pub = rospy.Publisher("/avocado", Marker, queue_size = 2)

marker = Marker()

marker.header.frame_id = "base_link"
marker.header.stamp = rospy.Time.now()
marker.ns = ""

# Shape (mesh resource type - 10)
marker.type = 10
marker.id = 0
marker.action = 0

# Note: Must set mesh_resource to a valid URL for a model to appear
marker.mesh_resource = "package://meshes/bases/waffle_pi_base.stl"
marker.mesh_use_embedded_materials = True

# Scale
marker.scale.x = 10.0
marker.scale.y = 10.0
marker.scale.z = 10.0

# Color
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 1.0

# Pose
marker.pose.position.x = 3
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

while not rospy.is_shutdown():
  marker_pub.publish(marker)
  rospy.rostime.wallsleep(1.0)

