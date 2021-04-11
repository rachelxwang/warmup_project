#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# This node directs the robot to follow a person or object.
class PersonFollower(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node("person_follower")

        # setup subscriber to scan topic and set process_scan for callback
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # setup publisher to cmd_vel topic
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # initialize empty twist
        self.twist = Twist()

    # process the data from the LiDAR and direct robot to follow the
    # person/object through proportional control of angular velocity
    def process_scan(self, data):
        # no object or object out of range
        if all(x == data.ranges[0] for x in data.ranges):
            self.twist.angular.z = 0
            self.twist.linear.x = 0
            self.pub.publish(self.twist)
        # object detected
        else:
            cur_dist = min(data.ranges)
            cur_ang = data.ranges.index(cur_dist)

            # stop when less than 0.25m away from object
            if cur_dist < 0.4:
                self.twist.linear.x = 0
            else:
                self.twist.linear.x = 0.3

            # determine error signal
            if cur_ang < 180:
                err = cur_ang - 0
            else:
                err = cur_ang - 360

            # determine angular velocity
            k_p = 0.01
            self.twist.angular.z = k_p * err

            # publish twist
            self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()
