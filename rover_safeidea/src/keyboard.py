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
        self.pressed = []
        rospy.sleep(1)
        rospy.loginfo(rospy.get_name())

    def send_messege(self):
        self.msg_to_publish.forward = self.forward
        self.msg_to_publish.backward = self.backward
        self.msg_to_publish.direction = self.direction
        self.pub.publish(self.msg_to_publish)

    def on_pressed(self, key):
        try:
            k = key.char
            #print('{0} pressed'.format(key.char))
        except:
            k = key.name
            #print('{0} pressed'.format(key.name))

        new = False
        rospy.loginfo(k)

        if k not in self.pressed:
            if k == 'x':
                new = True

            elif k == 'w':
                new = True
                self.forward = (self.forward + 10) % 250

            elif k == 's':
                new = True
                self.backward = (self.backward + 10) % 250

            elif k == 'd':
                new = True
                self.direction = (self.direction + 10) % 1000

            elif k == 'a':
                new = True
                self.direction = (self.direction - 10) % 1000

            elif k == 'r':
                new = True
                self.direction = 0
                self.backward = 0
                self.forward = 0

            elif k == '1' or k == '2' or k == '3' or k == '4' or k == '5' or k == '6' or  k == '7' or k == '8' or k == '9' or k == '0':
                new = True
                self.forward = int(k)*25


        if new:
            self.pressed.append(k)
            self.msg_to_publish.key = ord(k)
            self.send_messege()


    def on_released(self,key):
        try:
            k = key.char
            #print('{0} release'.format(key))
        except:
            k = key.name
            #print('{0} pressed'.format(key.name))

        if k in self.pressed:
            self.pressed.remove(k)


    def run(self):
        rate = rospy.Rate(30)
        listener = Listener(on_press=self.on_pressed, on_release=self.on_released)
        listener.start()
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                listener.stop()
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    k = keyboard()
    k.run()
