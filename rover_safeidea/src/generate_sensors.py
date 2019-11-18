#! /usr/bin/env python3

import rospy
import time
from rover_msg.msg import Sensor
import random


class Generator:
    def __init__(self):
        rospy.init_node("sensors", disable_signals=True)
        rospy.loginfo("Setting up Sensors .....")
        self.pub_sen = rospy.Publisher("sensors_topic", Sensor, queue_size=10)
        self.msg_to_publish = Sensor()
        self.msg_to_publish.temp = 0
        self.msg_to_publish.dust = 0
        self.msg_to_publish.light = 0
        self.msg_to_publish.radiation = 100
        self.temp = 0
        self.dust = 0
        self.light = 0
        self.radiation = 0

    def generate(self):
        temp = random.randrange(-133, 27)
        if temp > self.temp:
            self.temp = self.temp + abs(temp/160)
        elif temp == self.temp:
            pass
        else:
            self.temp = self.temp - abs(temp/160)

        dust = random.randrange(0, 1000, 1)/1000
        if dust > self.dust:
            self.dust = self.dust + 0.001 * (1 + dust)
        elif dust == self.dust:
            pass
        else:
            self.dust = self.dust - 0.001 * (1 + dust)

        light = random.randrange(0, 1000, 1)/1000
        if light > self.light:
            self.light = self.light + 0.001 * (1 + light)
        elif light == self.light:
            pass
        else:
            self.light = self.light - 0.001 * (1 + light)

        radiation = random.randrange(0, 1000, 1)/1000
        if radiation > self.radiation:
            self.radiation = self.radiation + 0.001 * (1 + radiation)
        elif radiation == self.radiation:
            pass
        else:
            self.radiation = self.radiation - 0.001 * (1 + radiation)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                self.generate()
                self.msg_to_publish.temp = self.temp
                self.msg_to_publish.dust = self.dust
                self.msg_to_publish.light = self.light
                self.msg_to_publish.radiation = self.radiation
                self.pub_sen.publish(self.msg_to_publish)
                print(self.msg_to_publish)
            except KeyboardInterrupt:
                rospy.loginfo("Ending...")
                rospy.signal_shutdown("Quit")


if __name__ == "__main__":
    sen = Generator()
    sen.run()
