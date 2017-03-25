#!/usr/bin/env python
# Copyright 2017 Lucas Walter
# BSD 3 license
# Generate hex grid Marker

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
        p1.x = x0 + s
        p1.y = y0
        points.append(p1)

        p2 = Point()
        p2.x = p1.x + length * math.cos(angle)
        p2.y = p1.y + length * math.sin(angle)
        points.append(p2)

        s += diameter + length
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

x_segments = rospy.get_param("~x_segments", 10)
y_segments = rospy.get_param("~y_segments", 16)

# the grid will be segments * min_diameter tall (with segments-1 columns interleaved)
# and segments * diameter * 1.5 - length * 1.5 width without aspect ratio correction
ratio = (x_segments * diameter * 1.5 - length * 1.5) / (x_segments * min_diameter)
rospy.loginfo(ratio)

for i in range(3):
    angle = float(i) * 2.0 / 3.0 * math.pi
    fr = 1.0
    if i == 1:
        fr = -1.0
    frl = 1.0
    # if i == 2:
    #     frl = -1.0
    print length, diameter, math.degrees(angle), x_segments
    xoff = 0
    yoff = 0
    if i == 2:
        xoff = 1.5 * length
        yoff = min_diameter / 2.0
    if True:
        x0 = 0 + xoff
        y0 = 0 + yoff
        soff = 0
        if i == 0:
            soff = 1
        for j in range(y_segments + soff):
            marker.points.extend(edge_pts(x0, y0, frl * length, frl * diameter, angle, x_segments))
            y0 += min_diameter

    if True:
        soff = 0
        if i == 2:
            xoff = -1.5 * length
        if i == 0:
            soff = -1
        x0 = 1.5 * length + xoff
        y0 = min_diameter / 2.0 + yoff
        for j in range(y_segments):
            marker.points.extend(edge_pts(x0, y0, frl * length,
                                 frl * diameter, angle, x_segments + soff))
            y0 += min_diameter

# while not rospy.is_shutdown():
for i in range(3):
    marker.header.stamp = rospy.Time.now()
    # rospy.loginfo(len(marker.points))
    pub.publish(marker)
    rospy.sleep(0.5)
