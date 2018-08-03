#!/usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node('cmdline')

cmdPub = rospy.Publisher('/feth/race/cmd', String, queue_size=1)
# rospy.spin()

rospy.loginfo("go")
while not rospy.is_shutdown():
    cmd = raw_input('Befehl: ')
    cmdPub.publish((cmd))
    print('Befehl "' + str(cmd) + '" gesendet!')

rospy.spin()