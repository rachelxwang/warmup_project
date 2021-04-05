#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist

# This node publishes messages to direct the robot to drive in a square.
class DriveSquare(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.drive_square_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # directs robot to turn 90 degrees
    def turn(self, twist):
        twist.angular.z = 0.5
        self.drive_square_pub.publish(twist)
        rospy.sleep(3.14159265358)
        twist.angular.z = 0
        self.drive_square_pub.publish(twist)
        rospy.sleep(0.25)

    # direct robot to move forward for 5 seconds
    def go(self, twist):
        twist.linear.x = 0.20
        self.drive_square_pub.publish(twist)
        rospy.sleep(5)
        twist.linear.x = 0
        self.drive_square_pub.publish(twist)
        rospy.sleep(0.25)

    def run(self):
        # setup the Twist message
        my_twist = Twist()

        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        # repeat go and turn movements four times to complete square
        for i in range(0,4):
            self.go(my_twist)
            self.turn(my_twist)


if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
