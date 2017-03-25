#!/usr/bin/env python
# Copyright 2017 Lucas Walter
#
# Generate triangle list Marker

import math
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def edge_pts(x0, y0, length, diameter, angle, num_segments):
    points = []
    s = 0
    for j in range(num_segments):
        p1 = Point()
        p1.x = x0 + s * math.cos(angle)
        p1.y = y0 + s * math.sin(angle)
        points.append(p1)

        s += length

        p2 = Point()
        p2.x = x0 + s * math.cos(angle)
        p2.y = y0 + s * math.sin(angle)
        points.append(p2)

        s += diameter
    return points

rospy.init_node('hex_marker')

pub = rospy.Publisher("marker", Marker, queue_size=2)

# edge length
length = rospy.get_param("~length", 1.0)
# corner to corner distance
diameter = 2.0 * length
min_diameter = 2.0 * length * math.cos(math.radians(30))

marker = Marker()
marker.header.frame_id = rospy.get_param("~frame_id", "map")
marker.ns = marker.header.frame_id
marker.id = 0
marker.type = Marker.LINE_LIST
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = length / 10.0
marker.scale.y = 1.0
marker.scale.z = 1.0

marker.color.a = 1.0
marker.color.r = rospy.get_param("~r", 0.5)
marker.color.g = rospy.get_param("~g", 0.5)
marker.color.b = rospy.get_param("~b", 0.5)

num_segments = rospy.get_param("~segments", 10)

i = 2
for i in range(3):
    angle = float(i) * 2.0 / 3.0 * math.pi
    fr = 1.0
    if i == 1:
        fr = -1.0
    frl = 1.0
    if i == 2:
        frl = -1.0
    print length, diameter, math.degrees(angle), num_segments
    if True:
        x0 = 0
        y0 = 0
        if i == 2:
            x0 += length
        for j in range(num_segments):
            marker.points.extend(edge_pts(x0, y0, frl * length, frl * diameter, angle, num_segments))
            x0 += fr * min_diameter * math.cos(angle + math.pi/2.0)
            y0 += fr * min_diameter * math.sin(angle + math.pi/2.0)

    if True:
        xoff = 1.5 * length
        yoff = min_diameter / 2.0
        x0 = xoff * math.cos(angle) + yoff * math.cos(angle + math.pi/2.0)
        y0 = xoff * math.sin(angle) + yoff * math.sin(angle + math.pi/2.0)

        if i == 2:
            x0 += length
        for j in range(num_segments):
            marker.points.extend(edge_pts(x0, y0, frl * length, frl * diameter, angle, num_segments))
            x0 += fr * min_diameter * math.cos(angle + math.pi/2.0)
            y0 += fr * min_diameter * math.sin(angle + math.pi/2.0)

# while not rospy.is_shutdown():
for i in range(3):
    marker.header.stamp = rospy.Time.now()
    # rospy.loginfo(len(marker.points))
    pub.publish(marker)
    rospy.sleep(0.5)
