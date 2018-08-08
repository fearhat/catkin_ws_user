#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16, UInt8, Float32
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from functools import partial, update_wrapper
import math
import tf
import time
import signal
import sys
import cv2
import pidcon
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime


class Controller:
    def __init__(self):
        self.initSpeed = 1200
        self.wiggleSpeed = self.initSpeed
        self.maxSpeed = self.initSpeed
        self.desiredSpeed = 0
        self.actualSpeed = 0
        self.currentSteer = 90
        self.orientation = [0, 0, 0, 0]
        self.yaw = 0
        self.position = [0, 0, 0]
        self.error = 0
        self.lastError = 0
        self.lastTimeStamp = 0
        self.euler = (0, 0, 0)
        self.kP = 110.0
        # self.kI = 0.0003
        self.kI = 0.05
        self.kD = 10
        self.accKp = 0.5

        self.mapSizeX = 600
        self.mapSizeY = 400
        self.mapRes = 10

        self.race = False
        self.wiggling = False
        self.stop = False
        self.laneId = 0
        self.laneSwitch = False
        self.obstacleDetection = False
        self.stopOnObstacle = False
        self.goTo = False
        self.goToPoint = [0, 0]

        self.stopAtDensity = 1000
        self.stopAtDistance = 1600

        rospack = rospkg.RosPack()
        self.filePath = rospack.get_path("contr") + "/src/"
        self.laneField = [
            np.load(self.filePath + "matrix100cm_lane1(1).npy"),
            np.load(self.filePath + "matrix100cm_lane2(1).npy"),
            np.load(self.filePath + "parkmitte.npy"),
        ]

        rospy.on_shutdown(self.shutdown)
        self.shut = False

        self.cvBridge = CvBridge()
        self.__speedPub = rospy.Publisher("/feth/speed", Int16, queue_size=1)
        self.__dataPub = rospy.Publisher(
            "/controllah/data", String, queue_size=1)
        self.__steerPub = rospy.Publisher(
            "/feth/steering", UInt8, queue_size=1)
        self.__odomSub = rospy.Subscriber(
            "/localization/odom/2", Odometry, self.odoMsg)
        self.__depthSub = rospy.Subscriber(
            "/feth/app/camera/depth/points", PointCloud2, self.depthMsg)
        # self.spSub = rospy.Subscriber("/feth/speed", Int16, self.loop)
        # self.timer = rospy.Timer(rospy.Duration(0, 50), self.loop)

        self.__cmd = rospy.Publisher("/feth/race/cmd", String, queue_size=1)
        self.__cmdListener = rospy.Subscriber(
            "/feth/race/cmd", String, self.command)

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

    def goToYaw(self, desiredYaw=np.pi):
        # [x, y, z, w] = self.orientation

        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.orientation)
        diff = desiredYaw - self.yaw - np.pi
        nextSteer = int(diff * 180.0 + 90)

        self.currentSteer = nextSteer

    def driveLane(self):
        # rospy.loginfo("driving lane: " + str(self.position))
        # rospy.loginfo("0")
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
        # timeDiff = timestamp - self.lastTimeStamp
        timeDiff = 50
        #  PID = Kp * error + Ki * self.err_sum * dif_time + Kd *(error - self.last_error) / dif_time

        steering = (self.kP * self.error)
        steering = steering + \
            (self.kI * (self.error + self.lastError) * timeDiff)
        steering = steering + \
            (self.kD * (self.error - self.lastError) / timeDiff)
        # rospy.loginfo(timeDiff)

        yaw = np.arctan(f_y/(f_x))

        if (f_x > 0):
            self.desiredSpeed = -self.maxSpeed
        else:
            self.desiredSpeed = self.maxSpeed
            if (f_y > 0):
                steering = -np.pi / 2
            if (f_y < 0):
                steering = np.pi / 2

        if (f_x > 0):
            self.desiredSpeed = max(
                self.maxSpeed, self.desiredSpeed * ((np.pi/3) / (abs(steering) + 1)))

        if (f_x < 0):
            self.desiredSpeed = -self.desiredSpeed

        if (steering > 90):
            steering = 90
        elif (steering < -90):
            steering = -90

        steering = 90 + steering

        errSpeed = ((self.desiredSpeed/2) * (self.error * 2))

        if (self.desiredSpeed > 0):
            self.desiredSpeed -= errSpeed
        # else:
        #     self.desiredSpeed += errSpeed

        self.currentSteer = steering
        self.lastTimeStamp = timestamp

    def loop(self, evt=0):
        if not self.shut:

            if self.race and not self.stop:
                # rospy.loginfo("p")
                self.driveLane()

            if self.wiggling:
                self.wiggle()

            if self.laneSwitch:
                self.checkLaneSwitch()

            self.steer()
            self.drive()

    def checkLaneSwitch(self):
        timestamp = datetime.now().microsecond
        if 85 <= self.currentSteer <= 95 and (timestamp - self.laneSwitchTime) > 4500:
            self.laneSwitch = False
            # self.maxSpeed += 400

    def wiggle(self):
        if self.wiggling:
            self.desiredSpeed = -self.desiredSpeed

    def depthMsg(self, rawImg):
        # rospy.loginfo(rawImg)
        # rawImg
        # img = cv2.imdecode(rawImg, 0)
        # img = self.cvBridge.imgmsg_to_cv2(rawImg)
        img_ = rawImg
        # PointCloud2.read_points
        # rospy.loginfo(img_.data)
        # rospy.loginfo(img)
        # height, width = img_
        # img = img_.data
        # # height = height
        # res = height * width
        # avg = 0
        # ct = 1

        # minDist = 100000

        # for i in range(width//3, (width//3 * 2)):
        #     itm = img[height//2, i]
        #     if itm > 0:
        #         minDist = min(minDist, itm)

        # if (minDist < self.stopAtDistance and self.obstacleDetection):
        #     if (self.stopOnObstacle):
        #         self.stop = True
        #     if (not self.stopOnObstacle):
        #         self.switchLane()
        # else:
        #     self.stop = False

    def switchLane(self):
        if not self.laneSwitch:
            self.laneSwitch = True
            self.laneSwitchTime = datetime.now().microsecond
            # self.maxSpeed -= 400
            if self.laneId == 1:
                self.laneId = 0
            else:
                self.laneId = 1

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
        # self.printConf()

        # if (abs(err) > 0.5):
        #     self.desiredSpeed = self.desiredSpeed - (abs(err) * 300)

    def otherCarMsg(self, carId, gpsData):
        # rospy.loginfo(gpsData)
        return 1

    def steer(self):
        if (self.currentSteer > 180):
            self.currentSteer = 180
        if (self.currentSteer < 0):
            self.currentSteer = 0
        # rospy.loginfo("steering: " + str(self.currentSteer))

        self.__steerPub.publish(UInt8(self.currentSteer))

    def drive(self):
        spd = self.desiredSpeed

        # curSpd = self.actualSpeed
        # spdDiff = spd - curSpd

        # self.accKp = 0.3
        # akp = spdDiff * self.accKp
        # # rospy.loginfo(akp)
        # spd = spd + akp

        if self.stop:
            spd = 0

        self.__speedPub.publish(Int16(spd))

    # def drive(self):
    #     spd = self.desiredSpeed
    #     if self.stop:
    #         spd = 0

    #     self.__speedPub.publish(Int16(spd))

    def printConf(self):
        rospy.loginfo(
            "\n   kp= " + str(self.kP) + "\n" +
            "   ki= " + str(self.kI) + "\n" +
            "   kd= " + str(self.kD) + "\n" +
            "  dns= " + str(self.stopAtDensity) + "\n" +
            "  dst= " + str(self.stopAtDistance) + "\n" +
            "  akp= " + str(self.accKp) + "\n" +
            "speed= " + str(self.desiredSpeed) + "\n" +
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
            self.desiredSpeed = 0
        if (cmd == "start"):
            self.race = True
            self.maxSpeed = self.initSpeed
            self.printConf()
        if (cmd.startswith("speed")):
            cmdA = cmd.split("=")
            # rospy.loginfo(cmdA)
            self.maxSpeed = int(cmdA[1])
            self.initSpeed = self.maxSpeed
            self.printConf()
        if (cmd.startswith("ki")):
            cmdA = cmd.split("=")
            self.kI = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("dns")):
            cmdA = cmd.split("=")
            self.stopAtDensity = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("akp")):
            cmdA = cmd.split("=")
            self.accKp = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("dst")):
            cmdA = cmd.split("=")
            self.stopAtDistance = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("kp")):
            cmdA = cmd.split("=")
            self.kP = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("kd")):
            cmdA = cmd.split("=")
            self.kD = float(cmdA[1])
            self.printConf()
        if (cmd.startswith("ostp")):
            self.stopOnObstacle = not self.stopOnObstacle
        if (cmd.startswith("obs")):
            self.obstacleDetection = not self.obstacleDetection
        if (cmd.startswith("log")):
            self.printConf()
        if (cmd.startswith("wiggle")):
            if self.wiggling:
                self.wiggling = False
                # self.maxSpeed = self.initSpeed
                self.desiredSpeed = 0
            if not self.wiggling:
                self.wiggling = True
                self.desiredSpeed = self.wiggleSpeed
            # self.wiggling = not self.wiggling

    def boot(self):
        self.desiredSpeed = 0
        self.currentSteer = 90
        rospy.loginfo("Car started.")

    def shutdown(self):
        rospy.loginfo("Shutting down!")
        self.shut = True
        self.race = False
        self.maxSpeed = 0
        self.desiredSpeed = 0
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
