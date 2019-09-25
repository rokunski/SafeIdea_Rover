#! /usr/bin/env python3

import rospy
from rover_msg.msg import Controller, Keyboard

class master():

    def __init__(self):
        rospy.init_node('pc_master')
        rospy.loginfo("Setting up the Master node ...")

        #topic_name = rospy.get_param('controller_topic')

        self.sub_controller = rospy.Subscriber("controller_topic", Controller, self.controller_receive)
        self.sub_keyboard = rospy.Subscriber('keyboard_topic', Keyboard, self.keyboard_receive)

    def controller_receive(self, message):
        rospy.loginfo("otrzymane dane: {0}".format(message.forward))

    def keyboard_receive(self, message):
        rospy.loginfo('Klawiatura')


    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    mst = master()
    mst.run()
