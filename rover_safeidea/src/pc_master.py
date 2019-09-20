#! /usr/bin/env python3

import rospy
from rover_msg.msg import Controller

class master():

    def __init__(self):
        rospy.loginfo("Setting up the Master node ...")

        rospy.init_node('pc_master')

        topic_name = rospy.get_param('controller_topic')

        self.sub_controller = rospy.Subscriber(topic_name, Controller, self.controller_receive)

    def controller_receive(self, message):
        rospy.loginfo("otrzymane dane: {0}".format(message.forward))


    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    mst = master()
    mst.run()
