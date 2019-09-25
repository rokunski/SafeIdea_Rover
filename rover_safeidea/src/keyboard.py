#! /usr/bin/env python3

import rospy
from rover_msg.msg import Keyboard
from pynput.keyboard import Key, Listener



class keyboard():

    def __init__(self):

        rospy.init_node("keyboard", disable_signals=True)
        rospy.loginfo("Setting up Keyboard node .....")

        self.pub = rospy.Publisher('keyboard_topic', Keyboard, queue_size=10)
        self.msg_to_publish = Keyboard()
        self.forward = 0
        self.backward = 0
        self.direction = 0

    def send_messege(self):
        self.msg_to_publish.forward = self.forward
        self.msg_to_publish.backward = self.backward
        self.msg_to_publish.direction = self.direction
        self.pub.publish(self.msg_to_publish)

    def on_pressed(self, key):
        print('{0} pressed'.format(key))

    def on_released(self,key):
        print('{0} release'.format(key))

    def run(self):
        rate = rospy.Rate(30)
        listener = Listener(on_press=self.on_pressed, on_release=self.on_released)
        listener.start()
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')
if __name__ == "__main__":
    k = keyboard()
    k.run()
