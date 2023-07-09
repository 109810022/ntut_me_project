#!/usr/bin/env python

import rospy
from roslib import message
from geometry_msgs.msg import Twist, TwistStamped
from math import copysign



class Follower():
    def __init__(self):
    	rospy.init_node("twiststamped_to_twist")


        rospy.on_shutdown(self.shutdown)


        self.move_cmd = Twist()


        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.position_subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.set_cmd_vel, queue_size=5)

        rospy.loginfo("Subscribing to twiststamped......")

        # Wait for the  topic to become available
        rospy.wait_for_message('/twist_cmd', TwistStamped)

        rospy.loginfo("Ready to transform!")

    def set_cmd_vel(self, position):
        rospy.loginfo("linear x: [%5.2f], angular z: [%5.2f]", position.twist.linear.x, position.twist.angular.z)
 
          
        self.move_cmd.linear.x =  position.twist.linear.x
        self.move_cmd.angular.z =  position.twist.angular.z

        
        self.cmd_vel_pub.publish(self.move_cmd)
 

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        self.move_cmd.linear.x =  0
        self.move_cmd.angular.z =  0
     
        rospy.sleep(1)

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("twiststamped_to_twist node terminated.")