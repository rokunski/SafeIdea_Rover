#! /usr/bin/env python3

import rospy
import os
import time
from rover_msg.msg import Controller, Keyboard, Master_Motor


class MotorClass:
    def __init__(self, num):
        rospy.init_node('motor_node{0}'.format(num))
        rospy.loginfo("Setting up the Motor node number {0}...".format(num))
        topic_name = "rasp_motor_topic/{0}".format(num)
        self.sub_rasp = rospy.Subscriber(topic_name, Master_Motor, self.rasp_receive)
        self.msg_to_PWM = Master_Motor()

    def rasp_receive(self, message):
        self.msg_to_PWM.value = message.value
        self.msg_to_PWM.fr = message.fr

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            print(self.msg_to_PWM)
            rate.sleep()


if __name__ == "__main__":
    print("Which one motor you want to test?")
    num = input()
    motos = MotorClass(num)
    motos.run()
