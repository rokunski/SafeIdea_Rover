#! /usr/bin/env python3

#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import rospy
import time
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Byte
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

boundary_green = [([86, 31, 4], [220, 88, 50])]


class Scenario:

    def __init__(self):

        rospy.init_node("scenariusz", disable_signals=True)
        rospy.loginfo("Setting up Following scenario .....")
        print(cv2.__version__)
        self.bridge = CvBridge()
        self.turn = 0
        self.forward = 0
        self.time = 0
        self.steer_time = 0
        self.window_center = 320
        self.center_buffer = 10
        self.steer_bound = float(3)
        self.image_bound = float(320)
        self.kp = self.steer_bound/self.image_bound
        self.left_bound = int(self.window_center - self.center_buffer)
        self.right_bound = int(self.window_center + self.center_buffer)
        self.error = 0
        self.ballPixel = 0
        self.radius = 0
        self.pracka = False
        self.sub_image = rospy.Subscriber('/kamera/depth/image_raw2', Image, self.image_receive)
        self.pub_cmd = rospy.Publisher("follower", Twist, queue_size=10)
        self.sub_god = rospy.Subscriber('interrupts', Byte, self.inter)
        rospy.sleep(1)

    def inter(self, message):
        if message.data == 2:
            self.pracka = True
        else:
            self.pracka = False

    def image_receive(self, message):
        if self.pracka:
            image = message.data
            rospy.loginfo(message.encoding)
            if message.encoding == "rgb8":
                try:
                    image = self.bridge.imgmsg_to_cv2(message, "bgr8")
                except CvBridgeError as e:
                    print(e)
            print()
            if type(image) == np.ndarray:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower = np.array([40, 40, 40], dtype="uint8")
                upper = np.array([70, 255, 255], dtype="uint8")
                mask = cv2.inRange(image, lower, upper)
                output = cv2.bitwise_and(image, image, mask=mask)
                h, s, gray = cv2.split(output)
                circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3, 226.25, minRadius=10, maxRadius=200, param1=183.75,
                                           param2=50.9669)
                # cv2.imshow("grey", np.asarray(gray))
                cv2.imwrite("~/Desktop/gray.png", gray)
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, self.radius) in circles:
                        cv2.circle(gray, (x, y), self.radius, (255, 0, 0), 4)
                        cv2.circle(gray, (x, y), 2, (0, 0, 255), 4)
                        if self.radius > 10:
                            self.ballPixel = x
                        else:
                            self.ballPixel = 0
                else:
                    self.ballPixel = 0

                print("Radius")
                print(self.radius)
                print("Pixel x of a ball")
                print(self.ballPixel)
                # cv2.imshow("input", np.asarray(image))
                # cv2.imshow("threshold", np.asarray(mask))
                cv2.imshow("hough", np.asarray(gray))
                cv2.waitKey(1)

                if self.ballPixel < 10:
                    print("no ball")
                    self.error = 0
                    self.turn = 0.
                    self.radius = 0
                    self.forward = 0.
                elif (self.ballPixel < self.left_bound) or (self.ballPixel > self.right_bound):
                    self.error = self.window_center - self.ballPixel
                    value = abs(self.error * self.kp)
                    if 0 < self.ballPixel < self.left_bound:
                        print("left side")
                        if self.radius > 40:
                            self.turn = value
                            self.forward = 2 * value
                        elif self.radius < 30:
                            zmienna = (30 - self.radius) / 5
                            self.forward = - 5.0 * zmienna
                            self.turn = value
                        elif 30 <= self.radius <= 40:
                            self.turn = value
                            self.forward = 0.0

                    elif 640 > self.ballPixel > self.right_bound:
                        print("right side")
                        if self.radius > 40:
                            self.turn = -value
                            self.forward = 2 * value
                        elif self.radius < 30:
                            zmienna = (30 - self.radius) / 5
                            self.forward = - 5.0 * zmienna
                            self.turn = -value
                        elif 30 <= self.radius <= 40:
                            self.turn = -value
                            self.forward = 0.0
                else:
                    print("middle")
                    if self.radius > 40:
                        self.turn = 0.0
                        self.forward = 2.0
                    elif self.radius < 30:
                        zmienna = (30 - self.radius)/5
                        self.turn = 0.0
                        self.forward = - 5.0 * zmienna
                print("Results: t/f")
                print(self.turn)
                print(self.forward)

    def run(self):
        rate = rospy.Rate(30)
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.pub_cmd.publish(msg)
        while not rospy.is_shutdown():
            try:
                if self.pracka:
                    msg = Twist()
                    msg.linear.x = self.turn
                    msg.angular.z = self.forward
                    self.pub_cmd.publish(msg)
                rate.sleep()
                    # print(msg)

            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    sc = Scenario()
    sc.run()
