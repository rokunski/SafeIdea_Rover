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
        self.msg_to_publish.light = 1
        self.msg_to_publish.radiation = 0
        self.temp = 0
        self.dust = 0
        self.light = 1
        self.radiation = 0
        self.start = time.time()
        self.zmiennik_t = 1
        self.zmiennik_d = 1
        self.zmiennik_l = 1
        self.zmiennik_r = 1

    def generate(self):
        temp = random.randrange(-133, 27)
        if temp > self.temp:
            self.temp = self.temp * self.zmiennik_t + abs(temp/160)
        elif temp == self.temp:
            pass
        else:
            self.temp = self.temp * self.zmiennik_t - abs(temp/160)

        dust = random.randrange(0, 1000, 1)/1000
        if dust > self.dust:
            self.dust = self.dust * self.zmiennik_d + 0.001 * (1 + dust)
        elif dust == self.dust:
            pass
        else:
            self.dust = self.dust * self.zmiennik_d - 0.001 * (1 + dust)

        light = random.randrange(0, 1000, 1)/1000
        if light > self.light:
            self.light = self.light * self.zmiennik_l + 0.001 * (1 + light)
        elif light == self.light:
            pass
        else:
            self.light = self.light * self.zmiennik_l - 0.001 * (1 + light)

        radiation = random.randrange(0, 1000, 1)/1000
        if radiation > self.radiation:
            self.radiation = self.radiation * self.zmiennik_r + 0.001 * (1 + radiation)
        elif radiation == self.radiation:
            pass
        else:
            self.radiation = self.radiation * self.zmiennik_r - 0.001 * (1 + radiation)

        if self.start + 50 > time.time() > self.start + 20:
            self.zmiennik_t = abs(self.zmiennik_t - 0.01)
        elif time.time() > self.start + 50:
            self.zmiennik_t = self.zmiennik_t + 0.1

        if self.start + 40 > time.time() > self.start + 30:
            self.zmiennik_d = abs(self.zmiennik_d - 0.01)
        elif time.time() > self.start + 40:
            self.zmiennik_d = self.zmiennik_d + 0.1

        if self.start + 80 > time.time() > self.start + 60:
            self.zmiennik_l = abs(self.zmiennik_l - 0.01)
        elif time.time() > self.start + 80:
            self.zmiennik_l = self.zmiennik_l + 0.1

        if self.start + 20 > time.time() > self.start + 10:
            self.zmiennik_r = abs(self.zmiennik_r - 0.01)
        elif time.time() > self.start + 20:
            self.zmiennik_r = self.zmiennik_r + 0.1

        if time.time() > self.start + 150:
            self.zmiennik_d = 1
            self.zmiennik_l = 1
            self.zmiennik_r = 1
            self.zmiennik_t = 1

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
                time.sleep(0.1)
            except KeyboardInterrupt:
                rospy.loginfo("Ending...")
                rospy.signal_shutdown("Quit")


if __name__ == "__main__":
    sen = Generator()
    sen.run()
