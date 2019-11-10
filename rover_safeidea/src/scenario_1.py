#! /usr/bin/env python3

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class scenario:

    def __init__(self):

        rospy.init_node("scenariusz", disable_signals=True)
        rospy.loginfo("Setting up First scenario node .....")

        self.sub_image = rospy.Subscriber('/kamera/depth/image_raw/compressed', Image, self.image_receive)
        self.brigde = CvBridge()
        rospy.sleep(1)

    def image_receive(self,message):
        image = message.data
        rospy.loginfo(message.encoding)
        # try:
        #     image = self.brigde.imgmsg_to_cv2(message, "bgr8")
        # except CvBridgeError as e:
        #     print(e)

        #cv2.imshow("image", image)
        #cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')

if __name__ == "__main__":
    sc = scenario()
    sc.run()