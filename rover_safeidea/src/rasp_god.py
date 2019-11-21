#! /usr/bin/env python3

import rospy
from rover_msg.msg import Keyboard, Sensor, PC2Rasp
from geometry_msgs.msg import Vector3, Twist


class dm_system:
    def __init__(self):

        rospy.init_node("main", disable_signals=True)
        rospy.loginfo("Setting up Raspberry Main node .....")

        self.sensor_sub = rospy.Subscriber('sensor_topic', Sensor, self.sensor_receive)

        self.sub_pc = rospy.Subscriber('pc_to_rasp', PC2Rasp, self.pc_receive)
        self.sub_pc = rospy.Subscriber('wireless_scenario', Twist, self.scenario_receive)

        self.pub_slave = rospy.Publisher('main2slave', PC2Rasp, queue_size=10)
        self.twist_msg = Twist()

        #self.pub = rospy.Publisher('keyboard_topic', Keyboard, queue_size=10)
        #self.msg_to_publish = Keyboard()

        rospy.sleep(1)
        rospy.loginfo(rospy.get_name())

    def sensor_receive(self, message):

        print(message)

    def scenario_receive(self, message):
        msg = PC2Rasp()
        msg.motor_start = 1
        msg.linear = -int(message.angular.z * 255)
        msg.angular = -int(message.linear.x*333.333)
        self.pub_slave.publish(msg)

    def pc_receive(self,message):
        self.pub_slave.publish(message)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    k = dm_system()
    k.run()
