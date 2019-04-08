#!/usr/bin/env python2.7

import math
import rospy
from std_msgs.msg import ByteMultiArray, Float32
import threading

def test_heading():
    pub = rospy.Publisher('/car/heading', Float32, queue_size=1)
    rate = rospy.Rate(10)
    heading = 0
    while not rospy.is_shutdown():
        msg = Float32()
        heading += 10
        heading = heading % 360
        msg.data = heading
        pub.publish(msg)
        rate.sleep()

def test_objects_loop():
    pub = rospy.Publisher('/object_detection/detected_objects_vector', ByteMultiArray, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = ByteMultiArray()
        msg.data.append(0)
        msg.data.append(0)
        msg.data.append(0)
        pub.publish(msg)
        rate.sleep()

def test_curvature():
    pub = rospy.Publisher('/lane_detection/lane_heading', Float32, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Float32()
        msg.data = math.pi / 8
        pub.publish(msg)
        rate.sleep()

def test_center_offset():
    pub = rospy.Publisher('/lane_detection/center_offset', Float32, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Float32()
        msg.data(0)
        pub.publish(msg)
        rate.sleep()

def main():
    rospy.init_node('test_publisher', anonymous=True)
    #heading_thread = threading.Thread(target=test_heading)
    #heading_thread.start()
    curvature_thread = threading.Thread(target=test_curvature)
    curvature_thread.start()

if __name__ == "__main__":
    main()