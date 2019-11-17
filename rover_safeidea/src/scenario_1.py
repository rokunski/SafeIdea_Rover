#! /usr/bin/env python3

#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import rospy
import time
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

boundary_green = [([86, 31, 4], [220, 88, 50])]


class scenario:

    def __init__(self):

        rospy.init_node("scenariusz", disable_signals=True)
        rospy.loginfo("Setting up Following scenario .....")
        print(cv2.__version__)
        self.sub_image = rospy.Subscriber('/kamera/depth/image_raw2', Image, self.image_receive)
        self.bridge = CvBridge()
        self.pub_cmd = rospy.Publisher("/rover/cmd_vel", Twist, queue_size=10)
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
        rospy.sleep(1)

    def image_receive(self, message):
        image = message.data
        rospy.loginfo(message.encoding)
        if message.encoding == "rgb8":
            try:
                image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            except CvBridgeError as e:
                print(e)
        print()
        if type(image) == np.ndarray:
            lower = np.array([0, 100, 0], dtype="uint8")
            upper = np.array([50, 255, 50], dtype="uint8")
            mask = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask=mask)
            gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3, 500, minRadius=10, maxRadius=200, param1=100,
                                       param2=60)

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, self.radius) in circles:
                    cv2.circle(output, (x, y), self.radius, (0, 255, 0), 4)
                    if self.radius > 10:
                        self.ballPixel = x
                    else:
                        self.ballPixel = 0

                # cv2.imshow("output", output)
            print("Radius")
            print(self.radius)
            print("Pixel x of a ball")
            print(self.ballPixel)
            cv2.imshow("output", np.asarray(gray))
            cv2.waitKey(1)

            if self.ballPixel < 10:
                print("no ball")
                self.error = 0
                self.turn = 0
                self.radius = 0
            elif (self.ballPixel < self.left_bound) or (self.ballPixel > self.right_bound):
                self.error = self.window_center - self.ballPixel
                value = abs(self.error * self.kp)
                if self.ballPixel < self.left_bound:
                    print("left side")
                    if self.radius > 30 and self.ballPixel < 110:
                        self.turn = value
                    else:
                        self.turn = value - 1
                elif self.ballPixel > self.right_bound:
                    print("right side")
                    if self.radius > 30 and self.ballPixel > 540:
                        print(self.ballPixel)
                        self.turn = -value
                    else:
                        self.turn = -value + 1
            else:
                print("middle")
                if self.radius < 40:
                    self.turn = 0
                    self.forward = -0.3
                else:
                    self.turn = 0
                    self.forward = 0
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
                msg = Twist()
                msg.linear.x = self.turn
                msg.angular.z = self.forward
                self.pub_cmd.publish(msg)
                rate.sleep()
                print(msg)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    sc = scenario()
    sc.run()
