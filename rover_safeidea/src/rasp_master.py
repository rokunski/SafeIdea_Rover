#! /usr/bin/env python3

import rospy
import os
from rover_msg.msg import Controller, Keyboard, Master_Motor
from enumerate import Joy


class Rasp:

    def __init__(self):
        rospy.init_node('rasp_master')
        rospy.loginfo("Setting up the Rasp Master node ...")
        self.msg_to_publish_list = []
        self.msg_to_publish = Master_Motor()
        for i in range(6):
            self.msg_to_publish_list.append(self.msg_to_publish)
        self.received_direction = 0
        self.sub_pc = rospy.Subscriber('pc_to_rasp', Keyboard, self.pc_receive)
        self.pub_motor_list = []
        for i in range(6):
            topic_name = "rasp_motor_topic/{0}".format(i)
            pub_motor = rospy.Publisher(topic_name, Master_Motor, queue_size=10)
            self.pub_motor_list.append(pub_motor)
        rospy.sleep(1)

    def send_to_motor(self):
        # TUTAJ PRZELICZAC W ZALEZNOSCI OD RECEIVED_DIRECTION NA JAKIE SILNIKI JAKIE IDZIE VALUE I ZWROT (FR)
        for i in range(6):
            self.pub_motor_list[i].publish(self.msg_to_publish_list[i])

    def pc_receive(self, message):
        if (message.forward - message.backward) > 0:
            self.msg_to_publish.value = message.forward - message.backward
            self.msg_to_publish.fr = True
        elif (message.forward - message.backward) < 0:
            self.msg_to_publish.value = message.backward - message.forward
            self.msg_to_publish.fr = False
        else:
            self.msg_to_publish.value = 0
            self.msg_to_publish.fr = True

        self.received_direction = message.direction

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            #print(self.msg_to_publish)
            self.send_to_motor()
            rate.sleep()


if __name__ == "__main__":
    rsp = Rasp()
    rsp.run()
