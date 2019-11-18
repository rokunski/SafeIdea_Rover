#! /usr/bin/env python3

import rospy
import time
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import random
import threading
from enumerate import State
from copy import deepcopy


BeaconPoints = [Vector3(-20,20,0), Vector3(20,7,0), Vector3(15,-22,0), Vector3(-14,-16,0)]
BeaconPoints = [Vector3(-5,5,0), Vector3(5,5,0), Vector3(5,-5,0), Vector3(-5,-5,0)]

class beacon:

    def __init__(self):

        rospy.init_node("beacon", disable_signals=True)
        rospy.loginfo("Setting up BEACON Scenario node .....")

        self.robot_pose = Vector3(0.,0.,0.)
        self.robot_rot = Vector3(0.,0.,0.)
        self.turn_l = 0
        self.turn = 0
        self.forward_l = 0
        self.forward = 0

        self.aim = BeaconPoints[np.random.randint(0,4)]
        print(self.aim.x, " ", self.aim.y)

        self.stan = State.DIRECTION_SEARCH

        self.attenuation = 3
        self.f = 2400000000
        self.c = 300000000
        self.one_meter = 10 * self.attenuation * np.log10(4 * np.pi * self.f / self.c )
        print(self.one_meter)

        self.sig = 0
        self.first = True
        self.avoid = False
        self.stan_bc = State.DIRECTION_SEARCH
        self.ang_bc = 0
        self.time = 0
        self.stop_time = 0
        self.l = []
        self.ang = 0
        self.best = 1000
        self.left = False
        self.which = -1
        self.next_to = False

        self.sub_lidar = rospy.Subscriber('/rover/laser/scan', LaserScan, self.lidar_receive)
        self.sub_ground_truth = rospy.Subscriber('/ground_truth/state', Odometry, self.ground_receive)
        self.sub_imu = rospy.Subscriber("/rover/imu", Imu, self.imu_receive)
        self.pub_cmd = rospy.Publisher("/rover/cmd_vel", Twist, queue_size =10)

        rospy.sleep(1)

    def lidar_receive(self, message):
        data = message.ranges
        angle_min = message.angle_min
        angle_max = message.angle_max
        angle_increment = message.angle_increment

        min_left = 1000
        min_leftc = 1000
        min_center = 1000
        min_rightc = 1000
        min_right = 1000
        for i in range(0, int(len(data) / 4)):
            if min_right > data[i]:
                min_right = data[i]

        for i in range(int(len(data) / 4), int(len(data) / 2) - 20):
            if min_rightc > data[i]:
                min_rightc = data[i]

        for i in range(int(len(data) / 2) - 20, int(len(data) / 2) + 20):
            if min_center > data[i]:
                min_center = data[i]

        for i in range(int(len(data) / 2) + 20, int(len(data) / 4 * 3)):
            if min_leftc > data[i]:
                min_leftc = data[i]

        for i in range(int(len(data) / 4 * 3), len(data)):
            if min_left > data[i]:
                min_left = data[i]

        mini = min([min_left, min_leftc, min_center, min_rightc, min_right])
        idx = np.argmin(np.array([min_left, min_leftc, min_center, min_rightc, min_right]))

        do = False

        if mini < 1 and 0 < idx < 4:
            do = True
            if not self.avoid:
                self.stan_bc = deepcopy(self.stan)
                self.ang_bc = deepcopy(self.robot_rot.z)
                self.stop_time = time.time()
            self.avoid = True


            # self.turn = 0
            if idx == 1:
                self.forward_l = -0.3
                self.turn_l = -0.15
                self.left = False
                #self.time = time.time()
                #self.steer_time = 3
            elif idx == 3:
                self.forward_l = -0.3
                self.turn_l = 0.15
                self.left = True
                #self.time = time.time()
                #self.steer_time = 3
            else:
                self.forward_l = -0.3
                self.turn_l = -0.15
                self.left = False
                #self.time = time.time()
                #self.steer_time = 6
        if 2 > mini > 1 and 0 < idx < 4:
            do = True

            if not self.avoid:
                self.stan_bc = deepcopy(self.stan)
                self.ang_bc = deepcopy(self.robot_rot.z)
                self.stop_time = time.time()

            self.avoid = True

            if idx == 1:
                self.forward_l = -0.3
                self.turn_l = -0.08
                self.left = False
                #self.time = time.time()
                #self.steer_time = 2
            elif idx == 3:
                self.forward_l = -0.3
                self.turn_l = 0.08
                self.left = True
                #self.time = time.time()
                #self.steer_time = 2
            else:
                self.forward_l = -0.3
                self.turn_l = -0.08
                self.left = False
                #self.time = time.time()
                #self.steer_time = 4
        if mini < 1 and (idx == 0 or idx == 4):
            do = True
            self.next_to = True
            if idx == 0:
                self.left = False
                self.turn_l = -0.08
            else:
                self.left = True
                self.turn_l = 0.08
        if not do:
            self.avoid = False
            self.next_to = False
            self.turn_l = 0
            self.forward_l = 0


    def imu_receive(self,message):
        orientation_q = message.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        euler = r.as_rotvec()

        self.robot_rot.x = euler[0]
        self.robot_rot.y = euler[1]
        self.robot_rot.z = euler[2]

    def ground_receive(self,message):
        self.robot_pose.x = message.pose.pose.position.x
        self.robot_pose.y = message.pose.pose.position.y
        self.robot_pose.z = message.pose.pose.position.z

        l = self.length(self.robot_pose.x, self.robot_pose.y, self.aim.x, self.aim.y)

        self.sig = -(self.one_meter + self.attenuation * 10 * np.log10(l))
        rrr = random.gauss(0,0.001)
        self.sig += self.sig * rrr


    @staticmethod
    def length(x1,y1,x2,y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    @staticmethod
    def angle(x1,y1,x2,y2 ):
        return np.arctan2(y2-y1, x2-x1)

    def steering(self):
        msg = Twist()
        msg.linear.x = self.turn
        msg.angular.z = self.forward
        self.pub_cmd.publish(msg)

    def search(self):
        while True:
            #rospy.loginfo(self.stan)
            try:
                if self.avoid and self.stan < 10:
                    if self.stan_bc == State.DIRECTION_SEARCH:
                        self.stan = State.FIRST_AVOID
                    if self.stan_bc == State.RIDE:
                        self.stan = State.ROT_AVOID
                    self.first = True
                    self.forward = 0
                    self.turn = 0


                if self.stan == State.DIRECTION_SEARCH:
                    if self.first:
                        rospy.loginfo("Starting finding direction ...")
                        self.l = []
                        self.time = time.time()
                        self.first = False
                        self.best = 1000
                    if time.time() - self.time < 8:
                        self.forward = -0.3
                        self.turn = 0
                        self.l.append(10 ** ((0 - self.sig - self.one_meter) / 10 / self.attenuation))
                        if self.l[-1] < 0.9:
                            self.turn = 0
                            self.forward = 0
                            self.first = True
                            self.stan = State.FINISH
                            rospy.loginfo("Finishing ...")
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        self.stan = State.CHECKING
                        rospy.loginfo("I'm done first state ...")
                elif self.stan == State.CHECKING:
                    if self.l[0] > self.l[-1]:
                        self.stan = State.RIDE
                        rospy.loginfo("Pushing forward ....")
                    else:
                        rospy.loginfo("I'm turning around")
                        self.stan = State.ROT_180
                elif self.stan == State.RIDE:
                    if self.first:
                        rospy.loginfo("Starting riding")
                        self.l = []
                        self.best = 1000
                        self.first = False
                    self.forward = -0.3
                    self.turn = 0
                    self.l.append(10 ** ((0 - self.sig - self.one_meter) / 10 / self.attenuation))
                    if self.l[-1] < self.best:
                        self.best = self.l[-1]
                    if self.l[-1] < 0.9:
                        self.turn = 0
                        self.forward = 0
                        self.first = True
                        rospy.loginfo("Finishing ...")
                        self.stan = State.FINISH
                    elif len(self.l) > 50:
                        if self.l[-1] - self.best > self.best*0.1:
                            print(self.l[-1], "    ", self.best)
                            self.turn = 0
                            self.forward = 0
                            self.first = True
                            self.stan = State.ROT_90
                            rospy.loginfo("Worse, 90 degree rotation ...")
                elif self.stan == State.ROT_180:
                    if self.first:
                        #self.time = time.time()
                        rospy.loginfo("Starting rotating 180 ...")
                        self.first = False
                        self.ang = deepcopy(self.robot_rot.z)
                        if self.ang < 0:
                            self.ang += np.pi
                        else:
                            self.ang -= np.pi
                    if np.abs(self.ang - self.robot_rot.z) > 0.01:
                        self.forward = -0.3
                        self.turn = 0.13
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        self.stan = State.RIDE
                        rospy.loginfo("Finish rotation 180 ....")
                elif self.stan == State.ROT_90:
                    if self.first:
                        #self.time = time.time()
                        rospy.loginfo("Starting rotating 90 ...")
                        self.first = False
                        self.ang = deepcopy(self.robot_rot.z)
                        self.ang += np.pi/2
                        right = True
                        if self.ang > np.pi:
                            self.ang -= 2 * np.pi
                            right = False
                    if np.abs(self.ang - self.robot_rot.z) > 0.01:
                        self.forward = -0.3
                        self.turn = 0.13
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        rospy.loginfo("Finish rotation 90 ...")
                        self.stan = State.DIRECTION_SEARCH
                elif self.stan == State.FINISH:
                    if self.first:
                        rospy.loginfo("I'm here")
                        self.first = False
                    self.forward = 0
                    self.turn = 0
                elif self.stan == State.FIRST_AVOID:
                    if self.stop_time - self.time < 4:
                        self.first = True
                        self.forward = 0
                        self.turn = 0
                        self.stan = State.ROT_180_AVOID
                        self.which = 0
                    else:
                        self.first = True
                        self.forward = 0
                        self.turn = 0
                        if self.l[0] > self.l[-1]:
                            self.stan = State.ROT_AVOID
                            rospy.loginfo("Needed to avoid obstacle....")
                        else:
                            rospy.loginfo("I'm turning around")
                            self.which = 1
                            self.stan = State.ROT_180_AVOID
                elif self.stan == State.ROT_180_AVOID:
                    if self.first:
                        #self.time = time.time()
                        rospy.loginfo("Starting rotating 180 ...")
                        self.first = False
                        self.ang = deepcopy(self.robot_rot.z)
                        if self.ang < 0:
                            self.ang += np.pi
                        else:
                            self.ang -= np.pi
                    if np.abs(self.ang - self.robot_rot.z) > 0.01:
                        self.forward = -0.3
                        self.turn = 0.13
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        if self.which == 1:
                            self.stan = State.RIDE
                        else:
                            self.stan = State.DIRECTION_SEARCH
                        self.which = -1
                        rospy.loginfo("Finish rotation 180 ....")
                elif self.stan == State.ROT_AVOID:
                    if self.first:
                        self.first = False
                        rospy.loginfo("Starting avoid obstacle ....")
                    if self.avoid:
                        self.forward = self.forward_l
                        self.turn = self.turn_l
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        self.stan = State.RIDE_AVOID
                        rospy.loginfo("Finish avoid obstacle ....")
                elif self.stan == State.RIDE_AVOID:
                    if self.first:
                        rospy.loginfo("Riding until pass obstacle ....")
                        self.first = False

                    if self.next_to:
                        self.forward = -0.3
                        self.turn = self.turn_l
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        rospy.loginfo("Pass obstacle ....")
                        self.stan = State.ROT_TO_PREV

                elif self.stan == State.ROT_TO_PREV:
                    if self.first:
                        self.first = False
                        rospy.loginfo("Rotating to previous angle ...")
                    if np.abs(self.ang_bc - self.robot_rot.z) > 0.01:
                        if self.left:
                            self.forward = -0.3
                            self.turn = -0.13
                        else:
                            self.forward = -0.3
                            self.turn = 0.13
                    else:
                        self.forward = 0
                        self.turn = 0
                        self.first = True
                        rospy.loginfo("Reach previous angle ...")
                        self.stan = State.RIDE


            except KeyboardInterrupt:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub_cmd.publish(msg)
                break
        return

    def run(self):
        rate = rospy.Rate(30)
        self.read = threading.Thread(target=self.search).start()
        while not rospy.is_shutdown():
            try:
                self.steering()
                rate.sleep()
            except Exception as e:
                print(e)
            except KeyboardInterrupt:
                #self.read.stop()
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub_cmd.publish(msg)
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')




if __name__ == "__main__":
    sc = beacon()
    sc.run()

