#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, String

def callback(raw_msg):
  publisher.publish("i heard: " + str(raw_msg.data))

rospy.init_node("assignment1_subscriber")

rospy.Subscriber("/yaw", Float32, callback)
publisher = rospy.Publisher("/assignment1_pub_sub", String)

rospy.spin()
