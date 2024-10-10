#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray, Marker

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", MarkerArray, queue_size = 2)

marker_array = MarkerArray()
marker_array.markers = []

#################################################################################################################
marker = Marker()

marker.header.frame_id = "map"
marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker.type = 2
marker.action = marker.ADD
marker.id = 0

# Set the scale of the marker
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0

# Set the color
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Set the pose of the marker
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

marker_array.markers.append(marker)

#################################################################################################################
marker2 = Marker()

marker2.header.frame_id = "map"
marker2.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker2.type = 2
marker2.action = marker2.ADD
marker2.id = 1

# Set the scale of the marker2
marker2.scale.x = 1.0
marker2.scale.y = 1.0
marker2.scale.z = 1.0

# Set the color
marker2.color.r = 0.0
marker2.color.g = 1.0
marker2.color.b = 0.0
marker2.color.a = 1.0

# Set the pose of the marker2
marker2.pose.position.x = 10
marker2.pose.position.y = 0
marker2.pose.position.z = 0
marker2.pose.orientation.x = 0.0
marker2.pose.orientation.y = 0.0
marker2.pose.orientation.z = 0.0
marker2.pose.orientation.w = 1.0

marker_array.markers.append(marker2)

rospy.loginfo(f"Number: {len(marker_array.markers)}")

# Renumber the marker IDs
id = 0
for m in marker_array.markers:
    m.id = id
    id += 1

while not rospy.is_shutdown():
  marker_pub.publish(marker_array)
rospy.spin()  # 保持节点运行
  # rospy.rostime.wallsleep(1.0)