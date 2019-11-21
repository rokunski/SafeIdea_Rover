#! /usr/bin/env python3

import rospy
import time
from rover_msg.msg import Keyboard, Sensor, PC2Rasp
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Byte


class dm_system:
    def __init__(self):

        rospy.init_node("main", disable_signals=True)
        rospy.loginfo("Setting up Raspberry Main node .....")

        self.sensor_sub = rospy.Subscriber('sensor_topic', Sensor, self.sensor_receive)

        self.sub_pc = rospy.Subscriber('pc_to_rasp', PC2Rasp, self.pc_receive)
        self.sub_wire = rospy.Subscriber('wireless_scenario', Twist, self.scenario_receive)
        self.sub_fol = rospy.Subscriber('follower', Twist, self.scenario_receive)
        self.pub_interrupt = rospy.Publisher('interrupts', Byte, queue_size=10)

        self.pub_slave = rospy.Publisher('main2slave', PC2Rasp, queue_size=10)
        self.twist_msg = Twist()

        #self.pub = rospy.Publisher('keyboard_topic', Keyboard, queue_size=10)
        #self.msg_to_publish = Keyboard()
        self.start = time.time()
        rospy.sleep(1)
        rospy.loginfo(rospy.get_name())

    def sensor_receive(self, message):

        print(message)

    def inter(self):
        msg = Byte() # 1-przerywa followera, 2-przerywa wireless, 3-ingerencja operatora
        msg.data = 2
        if 50 > time.time() - self.start > 20:
            msg.data = 3
        elif 50 < time.time() - self.start:
            msg.data = 1
        self.pub_interrupt.publish(msg)

    def scenario_receive(self, message):
        msg = PC2Rasp()
        msg.motor_start = 1
        msg.linear = -int(message.angular.z * 255)
        msg.angular = -int(message.linear.x*333.333)
        self.pub_slave.publish(msg)

    def pc_receive(self, message):
        self.pub_slave.publish(message)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                self.inter()
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    k = dm_system()
    k.run()
