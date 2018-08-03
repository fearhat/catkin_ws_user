#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16, UInt8, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from functools import partial, update_wrapper
import math
import tf
import time
import signal
import sys
import pidcon
import rospkg
import numpy as np
from datetime import datetime

class Controller:
    def __init__(self):
        self.initSpeed = 1300
        self.maxSpeed = self.initSpeed
        self.currentSpeed = 0
        self.currentSteer = 90
        self.orientation = [0, 0, 0, 0]
        self.yaw = 0
        self.position = [0, 0, 0]
        self.error = 0
        self.lastError = 0
        self.lastTimeStamp = 0
        self.euler = (0, 0, 0)
        self.kP = 81.0
        self.kI = 0.0003
        self.kD = 40.0

        self.mapSizeX = 600
        self.mapSizeY = 400
        self.mapRes = 10

        self.carInFront = False
        self.race = False
        self.laneId = 0
        self.goTo = False
        self.goToPoint = [0, 0]

        rospack = rospkg.RosPack()
        self.filePath = rospack.get_path("contr") + "/src/"
        self.laneField = [
            np.load(self.filePath + "matrix100cm_lane1(1).npy"),
            np.load(self.filePath + "matrix100cm_lane2(1).npy"),
            np.load(self.filePath + "parkmitte.npy"),
        ]
        
        rospy.on_shutdown(self.shutdown)
        self.shut = False

        self.__speedPub =  rospy.Publisher("/feth/manual_control/speed", Int16, queue_size=1)
        self.__dataPub = rospy.Publisher("/controllah/data", String, queue_size=1)
        self.__steerPub = rospy.Publisher("/feth/steering", UInt8, queue_size=1)
        self.__odomSub = rospy.Subscriber("/localization/odom/2", Odometry, self.odoMsg)

        self.__cmd = rospy.Publisher("/feth/race/cmd", String, queue_size=1)
        self.__cmdListener = rospy.Subscriber("/feth/race/cmd", String, self.command)

        # bound_f = update_wrapper(partial(f, 1000), f)
        self.__carList = [
            # rospy.Subscriber("/localization/odom/0", Odometry, update_wrapper(partial(self.otherCarMsg, 0), self.otherCarMsg)),
            # rospy.Subscriber("/localization/odom/1", Odometry, update_wrapper(partial(self.otherCarMsg, 1), self.otherCarMsg)),
            # rospy.Subscriber("/localization/odom/3", Odometry, update_wrapper(partial(self.otherCarMsg, 3), self.otherCarMsg)),
            # rospy.Subscriber("/localization/odom/4", Odometry, update_wrapper(partial(self.otherCarMsg, 4), self.otherCarMsg)),
            # rospy.Subscriber("/localization/odom/5", Odometry, update_wrapper(partial(self.otherCarMsg, 5), self.otherCarMsg))
        ]

        self.boot()
        self.printConf()

    def printConf(self):
        rospy.loginfo(
            "\n   kp= " + str(self.kP) + "\n" +
            "   ki= " + str(self.kI) + "\n" +
            "   kd= " + str(self.kD) + "\n" +
            "speed= " + str(self.currentSpeed) + "\n" +
            " lane= " + str(self.laneId) + "\n" +
            "  ori= " + str(self.orientation) + "\n" +
            "  pos= " + str(self.position) + ""
        )

    def command(self, msg):
        cmd = msg.data
        rospy.loginfo(cmd)

        if cmd.startswith("lane"):
            self.laneId = (int(cmd.split("=")[1]) - 1)
        if (cmd == "stop"):
            self.race = False
            self.maxSpeed = 0
            self.currentSpeed = 0
        if (cmd == "start"):
            self.race = True
            self.maxSpeed = self.initSpeed
            self.printConf()
        if (cmd.startswith("speed")):
            cmdA = cmd.split("=")
            # rospy.loginfo(cmdA)
            self.maxSpeed = int(cmdA[1])
            self.printConf()
        if (cmd.startswith("ki")):
            cmdA = cmd.split("=")
            self.kI = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("kp")):
            cmdA = cmd.split("=")
            self.kP = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("kd")):
            cmdA = cmd.split("=")
            self.kD = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("log")):
            self.printConf()

    def goToYaw(self, desiredYaw=np.pi):
        # [x, y, z, w] = self.orientation

        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.orientation)
        diff = desiredYaw - self.yaw - np.pi
        nextSteer = int(diff * 180.0 + 90)

        self.currentSteer = nextSteer

    def driveLane(self):
        # rospy.loginfo("driving lane: " + str(self.position))
        timestamp = datetime.now().microsecond
        xIndex = np.int(self.position[0] * self.mapRes)
        yIndex = np.int(self.position[1] * self.mapRes)

        if (xIndex < 0):
            xIndex = 0
        if (yIndex < 0):
            yIndex = 0
        if (xIndex >= self.mapSizeX):
            xIndex = self.mapSizeX - 1
        if (yIndex >= self.mapSizeY):
            yIndex = self.mapSizeY - 1
        
        x3, y3 = self.laneField[self.laneId][xIndex, yIndex, :]
        yaw = self.yaw
        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3

        # Kp = 120
        # Kp = 2.5
        self.lastError = self.error
        self.error = np.arctan(f_y/(f_x))
        timeDiff = datetime.now().microsecond - self.lastTimeStamp
         #  PID = Kp * error + Ki * self.err_sum * dif_time + Kd *(error - self.last_error) / dif_time

        steering = (self.kP * self.error)
        steering = steering + (self.kI * (self.error + self.lastError) * timeDiff)
        steering = steering + (self.kD * (self.error - self.lastError) / timeDiff)
        yaw = np.arctan(f_y/(f_x))

        if (f_x > 0):
            self.currentSpeed = -self.maxSpeed
        else:
            self.currentSpeed = self.maxSpeed
            # if (f_y > 0):
            #     steering = -np.pi / 2
            # if (f_y < 0):
            #     steering = np.pi / 2
        
        if (f_x > 0):
            self.currentSpeed = max(self.maxSpeed, self.currentSpeed * ((np.pi/3) / (abs(steering) + 1)))

        if (f_x < 0):
            self.currentSpeed = -self.currentSpeed
        
        # rospy.loginfo(err)
        # if (abs(err) > 0.3):
        #     self.currentSpeed = 1500
        # rospy.loginfo(steering)
        if (steering > 90):
            steering = 90
        elif (steering < -90):
            steering = -90

        steering = 90 + steering

        # if not (70 <= steering <= 110):
        #     # s_ = steering + 1
        #     # strDiff = (abs(90 - s_) / s_) * 100
        #     # if strDiff > 40:
        #     #     strDiff = 40
        #     strDiff = self.maxSpeed / 10
        #     strDiff = int(strDiff)
        #     self.currentSpeed -= strDiff

        self.currentSteer = steering
        self.lastTimeStamp = timestamp
        
    def loop(self):
        if not self.shut:
            
            if self.race:
                self.driveLane()

            self.steer()
            self.drive()

    def odoMsg(self, rawMsg):
        ori = rawMsg.pose.pose.orientation
        x = rawMsg.pose.pose.position.x
        y = rawMsg.pose.pose.position.y

        self.position = [x, y, 0]
        self.orientation = [ori.x, ori.y, ori.z, ori.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.orientation)
        #  PID = Kp * error + Ki * self.err_sum * dif_time + Kd *(error - self.last_error) / dif_time
        self.yaw = yaw
        
        self.loop()

        # if (abs(err) > 0.5):
        #     self.currentSpeed = self.currentSpeed - (abs(err) * 300)

    def otherCarMsg(self, carId, gpsData):
        # rospy.loginfo(gpsData)
        return 1

    def steer(self):
        if (self.currentSteer > 180): self.currentSteer = 180
        if (self.currentSteer < 0): self.currentSteer = 0
        # rospy.loginfo("steering: " + str(self.currentSteer))

        self.__steerPub.publish(UInt8(self.currentSteer))

    def drive(self):
        self.__speedPub.publish(Int16(self.currentSpeed))
    
    def boot(self):
        self.currentSpeed = 0
        self.currentSteer = 90
        rospy.loginfo("Car started.")

    def shutdown(self):
        rospy.loginfo("Shutting down!")
        self.shut = True
        self.race = False
        self.maxSpeed = 0
        self.currentSpeed = 0
        self.drive()
        rospy.sleep(1)

rospy.init_node("controllah")

controller = Controller()

rospy.spin()

# def signal_handler(sig, frame):
#     print('Cleanup')
#     controller.currentSpeed = 0
#     controller.drive()
#     time.sleep(1)
#     # signal.pause()
#     sys.exit(0)

# signal.signal(signal.SIGINT, signal_handler)