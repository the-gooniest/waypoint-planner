#!/usr/bin/env python2.7

from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import numpy as np

class DistancePublisher:

    def __init__(self, initial_distance):
        rospy.init_node('distance_publisher', anonymous=True)
        self.x_car = 0
        self.y_car = 0
        self.heading = 0
        self.odom_msg_received = False
        self.initial_dist = initial_distance

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.distance_pub = rospy.Publisher("/distance_publisher/distance", Float32, queue_size = 1)

    def odom_callback(self, msg):
        self.odom_msg_received = True
        self.x_car = msg.pose.pose.position.x
        self.y_car = msg.pose.pose.position.y

        orientation = odom_data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w] 
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.heading = yaw

    def run(self):
        rate = rospy.Rate(10)

        # Wait for this node to reveive one odom_msg
        print "Waiting for odom msgs..."
        while not self.odom_msg_received:
            if rospy.is_shutdown():
                return
            rate.sleep()

        print "Odom msgs received!"
        print "Publishing distance..."

        # Create stopping point directly in front of the vehicle
        point = (self.x_car + np.cos(heading) * self.initial_dist,
                self.y_car + np.sin(heading) * self.initial_dist)

        # Publish distance from point
        while True:
            if rospy.is_shutdown():
                return

            # Calculate distance from point
            distance_from_point = np.sqrt(np.square(self.x_car - point[0]) + np.square(self.y_car - point[1]))

            msg = Float32()
            # msg.data = distance_from_point0
            msg.data = -1
            self.distance_pub.publish(msg)

if __name__ == "__main__":
    pub = DistancePublisher(10)
    pub.run()