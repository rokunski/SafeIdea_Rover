#! /usr/bin/env python3

import rospy
import time
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from laser_geometry import LaserProjection


BeaconPoints = [Vector3(-20,20,0), Vector3(20,7,0), Vector3(15,-22,0), Vector3(-14,-16,0)]


class beacon:

    def __init__(self):

        rospy.init_node("beacon", disable_signals=True)
        rospy.loginfo("Setting up BEACON Scenario node .....")
        self.laserProj = LaserProjection()

        self.pcPub = rospy.Publisher("/laserpoint", PointCloud2, queue_size=2)
        self.sub_lidar = rospy.Subscriber('/rover/laser/scan', LaserScan, self.lidar_receive)
        self.sub_ground_truth = rospy.Subscriber('/ground_truth/state', Odometry, self.ground_receive)
        self.pub_cmd = rospy.Publisher("/rover/cmd_vel", Twist, queue_size =10)

        self.map = [[0,0,0]]
        self.robot_pose = Vector3(0.,0.,0.)
        self.robot_rot = Vector3(0.,0.,0.)
        self.turn_l = 0
        self.turn_p = 0
        self.forward = 0

        self.aim = BeaconPoints[np.random.randint(0,4)]
        #self.aim = BeaconPoints[3]
        print(self.aim.x, " ", self.aim.y)
        self.avoid = False
        self.time = 0
        self.steer_time = 0
        # self.lidar_pose = Vector3(-0.35, 0, 0.625)
        # self.lidar_rot = Vector3(0,0.1222,3.14159265)
        #
        # self.lidar_pose_real = Vector3(-0.35, 0, 0.625)
        # self.lidar_rot_real = Vector3(0, 0.1222, 0)

        rospy.sleep(1)

    def lidar_receive(self, message):
        data = message.ranges
        angle_min = message.angle_min
        angle_max = message.angle_max
        angle_increment = message.angle_increment
        #if self.time == 0:

        min_left = 1000
        min_leftc = 1000
        min_center = 1000
        min_rightc = 1000
        min_right = 1000
        for i in range(0, int(len(data) / 4)):
            if min_right > data[i]:
                min_right = data[i]

        for i in range(int(len(data) / 4) , int(len(data) / 2) - 20):
            if min_rightc > data[i]:
                min_rightc = data[i]

        for i in range(int(len(data) / 2) - 20, int(len(data) / 2) + 20):
            if min_center > data[i]:
                min_center = data[i]

        for i in range(int(len(data) / 2) +20 ,int(len(data) / 4*3)):
            if min_leftc > data[i]:
                min_leftc = data[i]

        for i in range(int(len(data) / 4*3), len(data)):
            if min_left > data[i]:
                min_left = data[i]

        mini = min([min_left, min_leftc, min_center, min_rightc, min_right])
        idx = np.argmin(np.array([min_left, min_leftc, min_center, min_rightc, min_right]))

        do = False
        if mini < 1 and idx > 0 and idx < 4:
            #self.avoid = True
            do = True
            self.turn_p = 0
            if idx == 1:
                self.forward = -0.3
                self.turn_l = -0.19
                self.time = time.time()
                self.steer_time = 3
            elif idx == 3:
                self.forward = -0.3
                self.turn_l = 0.19
                self.time = time.time()
                self.steer_time = 3
            else:
                self.forward = -0.3
                self.turn_l = -0.19
                self.time = time.time()
                self.steer_time = 6
        if mini < 2.5 and mini > 1 and idx > 0 and idx < 4:
            do = True
            self.turn_p = 0
            if idx == 1:
                self.forward = -0.3
                self.turn_l = -0.1
                self.time = time.time()
                self.steer_time = 2
            elif idx == 3:
                self.forward = -0.3
                self.turn_l = 0.1
                self.time = time.time()
                self.steer_time = 2
            else:
                self.forward = -0.3
                self.turn_l = -0.1
                self.time = time.time()
                self.steer_time = 4
        if mini < 1 and (idx == 0 or idx == 4):
            do = True
            if idx == 0:
                #self.forward = -0.3
                self.turn_l = -0.08
            else:
                self.turn_l = 0.08
        if not do:
            #self.avoid = False
            self.turn_l = 0

        #print(mini)
        # rospy.loginfo(data)

        # angle = angle_min
        # for ran in data:
        #
        #     if ran != np.inf:
        #         t = self.calculate_point(ran, angle)
        #         self.map.append(list(t))
        #     angle += angle_increment
        # print("max")
        # print(min(self.map))


    # @staticmethod
    # def quaternion_to_euler(x, y, z, w):
    #
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + y * y)
    #     roll = math.atan2(t0, t1)
    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch = math.asin(t2)
    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw = math.atan2(t3, t4)
    #
    #     return [yaw, pitch, roll]

    def ground_receive(self,message):
        self.robot_pose.x = message.pose.pose.position.x
        self.robot_pose.y = message.pose.pose.position.y
        self.robot_pose.z = message.pose.pose.position.z
        orientation_q = message.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        euler = r.as_rotvec()
        R1 = r.as_dcm()
        #euler = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.robot_rot.x = euler[0]
        self.robot_rot.y = euler[1]
        self.robot_rot.z = euler[2]

        # R2 = R.from_rotvec([self.lidar_rot.x,self.lidar_rot.y,self.lidar_rot.z])
        # #print("sprawdz ", R2.as_euler("xyz", degrees=True))
        #
        # R2 = R2.as_dcm()
        # R3 = np.dot(R1,R2)
        # R3 = R.from_dcm(R3)
        # #RRRR = R3.as_euler("xyz")
        # R3 = R3.as_rotvec()
        #
        # self.lidar_rot_real.x = R3[0]
        # self.lidar_rot_real.y = R3[1]
        # self.lidar_rot_real.z = R3[2]
        # t1 = np.asarray([self.lidar_pose.x,self.lidar_pose.y,self.lidar_pose.z])
        # t = np.asarray([self.robot_pose.x,self.robot_pose.y,self.robot_pose.z]) + np.dot(R1, t1)
        # self.lidar_pose_real.x = t[0]
        # self.lidar_pose_real.y = t[1]
        # self.lidar_pose_real.z = t[2]
        #
        # rospy.loginfo("pozy")
        # rospy.loginfo(self.robot_pose)
        # rospy.loginfo(self.lidar_pose_real)
        # rospy.loginfo("rot")
        # rospy.loginfo(self.robot_rot)
        # print("dane" , self.lidar_rot_real)
        # rospy.loginfo(RRRR)


    # def calculate_point(self, ran, angle):
    #     y = np.sin(angle) * ran
    #     x = np.cos(angle) * ran
    #     # print(x,"y ", y, "rang ",range, "angle",angle )
    #     t = np.asarray([self.lidar_pose_real.x,self.lidar_pose_real.y,self.lidar_pose_real.z])
    #     R1 = R.from_rotvec([self.lidar_rot_real.x,self.lidar_rot_real.y,self.lidar_rot_real.z])
    #     #print("otrzymane: ", R1.as_rotvec())
    #     R1 = R1.as_dcm()
    #     #print(np.dot(R1, np.asarray([x, y, 0])))
    #     t1 = t + np.dot(R1,np.asarray([x,y,0]))
    #     #print(t1)
    #     #print(t1[2] , "   ", angle, " ", x, " ", y)
    #     return t1

    @staticmethod
    def length(x1,y1,x2,y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    @staticmethod
    def angle(x1,y1,x2,y2 ):
        return np.arctan2(y2-y1, x2-x1)

    def to_point(self):
        #if self.avoid:
        #    return
        l = self.length(self.robot_pose.x,self.robot_pose.y, self.aim.x, self.aim.y)
        if l > 1:
            self.forward = -0.6
        elif l > 0.2:
            self.forward = -0.2
        elif l < 0.05:
            self.forward = 0.0
            self.turn_p = 0
            return
        ang = self.angle(self.robot_pose.x,self.robot_pose.y, self.aim.x, self.aim.y)
        robot_ang = self.robot_rot.z

        # print(ang, " ", robot_ang)
        if robot_ang > 0:
            robot_ang -= np.pi
        else:
            robot_ang += np.pi

        # print(ang, " ", robot_ang)

        if robot_ang <= ang:
            lewo = np.fabs(ang - robot_ang)
            prawo = 2*np.pi - lewo
        else:
            prawo = np.fabs(robot_ang - ang)
            lewo = 2 * np.pi - prawo

        if prawo < lewo:
            self.turn_p = prawo/np.pi * self.forward/2
            if l < 1:
                self.turn_p = -prawo/np.pi
        else:
            self.turn_p = -lewo/np.pi * self.forward/2
            if l < 1:
                self.turn_p = lewo/np.pi

        #print(self.turn_p)
        print (l)
        print(prawo, " ", lewo )

        #print("robot_ang", robot_ang)

    def run(self):
        rate = rospy.Rate(30)
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.pub_cmd.publish(msg)
        while not rospy.is_shutdown():

            if self.time != 0 and time.time() - self.time > self.steer_time:
                self.time = 0
            else:
                self.to_point()
            try:
                msg = Twist()
                if self.turn_l == 0:
                    msg.linear.x = self.turn_p
                else:
                    msg.linear.x = 0.8 * self.turn_l + 0.2*self.turn_p
                msg.angular.z = self.forward
                #print(msg)
                self.pub_cmd.publish(msg)
                rate.sleep()

            except Exception as e:
                print(e)
            except KeyboardInterrupt:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub_cmd.publish(msg)
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    sc = beacon()
    sc.run()
