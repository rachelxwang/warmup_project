#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# This node directs the robot to drive alongside a wall in a room.
class WallFollower(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node("wall_follower")

        # setup subscriber to scan topic and set process_scan for callback
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # setup publisher to cmd_vel topic
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # initialize empty twist
        self.twist = Twist()

    # process the data from the LiDAR and direct robot to find and
    # begin following a wall within the room
    def process_scan(self, data):

        front = data.ranges[0]
        left = data.ranges[90]

        # initialize angular velocity to 0
        v = 0

        # if far from front wall
        if front > 0.75:
            # if close to left wall, correct to path 0.5m from left wall
            # by proportional control method
            if left < 1:
                v = 0.15 * (left - 0.5)
        # if approaching front wall
        else:
            # determine angular velocity to turn corner
            v = 1.0 * (front - 3.5)

        # set and publish linear and angular velocities
        self.twist.linear.x = 0.2
        self.twist.angular.z = v
        self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()
