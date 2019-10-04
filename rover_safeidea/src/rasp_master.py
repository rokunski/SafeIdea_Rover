#! /usr/bin/env python3

import rospy
import os
import subprocess
from rover_msg.msg import Controller, Keyboard
from enumerate import Joy


class rasp():

    def __init__(self):
        rospy.init_node('rasp_master')
        rospy.loginfo("Setting up the Rasp Master node ...")

        self.sub_pc = rospy.Subscriber('pc_to_rasp', Keyboard, self.pc_receive)
        self.pub_motor_list = []
        for i in range(6):
            topic_name = "rasp_motor_topic/{0}".format(i)
            pub_motor = rospy.Publisher(topic_name, Keyboard, queue_size=10)
            self.pub_motor_list.append(pub_motor)
        rospy.sleep(1)

        self.msg_to_publish = Keyboard()

    def send_to_motor(self):
        self.pub_rasp.publish(self.msg_to_publish)

    def pc_receive(self, message):
        print("Msg form pc received")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    rsp = rasp()
    rsp.run()
