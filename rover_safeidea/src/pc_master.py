#! /usr/bin/env python3

import rospy
import os
import subprocess
from rover_msg.msg import Controller, Keyboard
from enumerate import Joy

class master():

    def __init__(self):
        rospy.init_node('pc_master')
        rospy.loginfo("Setting up the Master node ...")

        #topic_name = rospy.get_param('controller_topic')

        self.sub_controller = rospy.Subscriber("controller_topic", Controller, self.controller_receive)
        self.sub_keyboard = rospy.Subscriber('keyboard_topic', Keyboard, self.keyboard_receive)
        self.pub_rasp = rospy.Publisher('pc_to_rasp', Controller, self.send_to_rasp, queue_size=10)
        rospy.sleep(1)

        self.msg_to_publish = Controller()

        self.controller = Joy.NOT_WORKING

    def send_to_rasp(self):
        self.pub_rasp.publish(self.msg_to_publish)

    def controller_receive(self, message):
        self.controller = Joy(message.status)
        rospy.loginfo(self.controller.name)

    def keyboard_receive(self, message):
        if message.key == ord('x'):
            if self.controller == Joy.NOT_WORKING:
                try:
                    os.system("gnome-terminal -e 'bash -c \"rosrun rover_safeidea xbox_controller.py; exec bash\"'")
                except:
                    try:
                        os.system("mate-terminal -e 'bash -c \"rosrun rover_safeidea xbox_controller.py; exec bash\"'")
                    except:
                        os.system("xterm -e 'bash -c \"rosrun rover_safeidea xbox_controller.py; exec bash\"'")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    mst = master()
    mst.run()
