#! /usr/bin/env python3

import rospy
import os
import subprocess
from rover_msg.msg import Controller, Keyboard, PC2Rasp
from enumerate import Joy


class Master:

    def __init__(self):
        rospy.init_node('pc_master', disable_signals=True)
        rospy.loginfo("Setting up the Master node ...")

        #topic_name = rospy.get_param('controller_topic')

        self.sub_controller = rospy.Subscriber("controller_topic", Controller, self.controller_receive)
        self.sub_keyboard = rospy.Subscriber('keyboard_topic', Keyboard, self.keyboard_receive)
        self.pub_rasp = rospy.Publisher('pc_to_rasp', PC2Rasp, queue_size=10)
        rospy.sleep(1)

        self.msg_to_publish = PC2Rasp()

        self.msg_to_publish.linear = 0
        self.msg_to_publish.angular = 0
        self.msg_to_publish.motor_start = False

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
                    #subprocess.run(['gnome-terminal','rosrun rover_safeidea xbox_controller.py' ])
                    good = os.system("gnome-terminal -e 'bash -c \"rosrun rover_safeidea xbox_controller.py; exec bash\"'")
                    if good != 0:
                        raise Exception('something wrong')
                except:
                    try:
                        good = os.system("mate-terminal -e 'bash -c \"rosrun rover_safeidea xbox_controller.py; exec bash\"'")
                        if good != 0:
                            raise Exception('something wrong')
                    except:
                        good = os.system("xterm -e 'bash -c \"rosrun rover_safeidea xbox_controller.py; exec bash\"'")
                        if good != 0:
                            rospy.loginfo("Cannot run Controller node")
        elif message.key == ord('w') or message.key == ord('s') or message.key == ord('d') or message.key == ord('a') or message.key == ord('r'):
            if self.msg_to_publish.motor_start == True:
                self.msg_to_publish.linear = message.forward - message.backward
                self.msg_to_publish.angular = message.direction

        elif message.key == ord('m'):
            self.msg_to_publish.linear = message.forward - message.backward
            self.msg_to_publish.angular = message.direction
            self.msg_to_publish.motor_start = not self.msg_to_publish.motor_start

        self.send_to_rasp()

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    mst = Master()
    mst.run()
