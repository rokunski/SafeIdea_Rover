#! /usr/bin/env python3

import rospy
import os
from rover_msg.msg import PC2Rasp, Master_Motor
from enumerate import Motor
from std_msgs.msg import Byte, Float64MultiArray
import numpy as np
import math
import pigpio


"""
    pierwsze 3 silniki są od prawych kół, a pozostałe od lewych
"""


list_GPIO_FR_SR = [11,12,13,14,15,16]

class Rasp:

    def __init__(self):
        rospy.init_node('rasp_master', disable_signals=True)
        rospy.loginfo("Setting up the Rasp Master node ...")
        self.msg_to_publish_list = []
        self.msg_to_publish = Master_Motor()
        self.msg_to_publish.fr = False
        self.msg_to_publish.rs  = False
        self.msg_to_publish.permission = False
        self.msg_to_publish.value = 0
        for i in range(6):
            self.msg_to_publish_list.append(self.msg_to_publish)

        self.received_direction = 0
        self.sub_pc = rospy.Subscriber('pc_to_rasp', PC2Rasp, self.pc_receive)
        self.pub_motor_list = []
        for i in range(6):
            topic_name = "rasp_motor_topic/{0}".format(i)
            pub_motor = rospy.Publisher(topic_name, Master_Motor, queue_size=10)
            self.pub_motor_list.append(pub_motor)

        self.motor_start = False
        self.demux_operation = False
        self.do_operation = False
        self.l = 0
        self.done = False

        self.sub_motor_list = []
        self.motor_status = []
        for i in range(6):
            topic_name = "motor_status/{0}".format(i)
            sub_motor = rospy.Subscriber(topic_name, Byte, self.motor_receive)
            self.sub_motor_list.append(sub_motor)
            self.motor_status.append(0)

        self.sub_freq_list = []
        self.freq = []
        for i in range(6):
            topic_name = "rasp_freq_topic/{0}".format(i)
            sub_motor = rospy.Subscriber(topic_name, Float64MultiArray, self.freq_receive)
            self.sub_freq_list.append(sub_motor)
            self.freq.append(0.0)

        self.data = []
        #self.data_PWM = []

        for i in range(6):
            self.data.append(PC2Rasp())
            self.data[i].linear = 0.0
            self.data[i].angular = 0.0
            #self.data_PWM.append(0)

        #self.pi = pigpio.pi()
        #for i in range(len(list_GPIO_FR_SR)):
            #self.pi.set_mode(list_GPIO_FR_SR[i], pigpio.OUTPUT)
        # self.pi.write(list_GPIO_FR_SR[5], 1)
        rospy.sleep(0.1)
        # self.pi.write(list_GPIO_FR_SR[5], 0)
        rospy.sleep(1)



    def motor_receive(self, message):
        nr = int(message.data/10)
        data = Motor(message.data%10)
        if self.motor_status[nr] != data:
            self.motor_status[nr] = data
            if self.demux_operation or self.do_operation:
                self.msg_to_publish_list[nr].permission = False
                self.l += 1
        else:
            rospy.loginfo("coś sie psuje")
        rospy.loginfo(message)
        if self.demux_operation or self.do_operation:
            self.done = False

    def freq_receive(self, message):
        self.freq[message.data[0]] = message.data[1]

    def send_to_motor_launch(self):
        rospy.loginfo("zaczynam")
        self.l = 0
        self.done = False
        while self.l < 6:
            if not self.done:
                if self.l == 6:  #czasami tu wchodzi
                    continue
                rospy.loginfo("wysyłam ")
                self.msg_to_publish_list[self.l].permission = True
                self.pub_motor_list[self.l].publish(self.msg_to_publish_list[self.l])
                self.done = not self.done
        self.demux_operation = False
        self.do_operation = False

    def send_to_motor_shut_down(self):
        self.do_operation = False
        for i in range(6):
            self.pub_motor_list[i].publish(self.msg_to_publish_list[i])
        if self.motor_status.count(Motor.WAITING) == 6:
            self.do_operation = True
        else:
            while self.motor_status.count(Motor.BREAKING) == 6:
                continue
            self.do_operation = True

    def send_to_motor(self):
        for i in range(6):
            self.pub_motor_list[i].publish(self.msg_to_publish_list[i])

    def motor_velocity(self, number, message=None):
        if message is not None:
            self.data[number] = message

        if self.data[number].angular == 0.0:
            #self.data_PWM[number] = self.data[number].linear
            if self.freq[number] == 0.0:
                return self.data[number].linear
            else:
                if self.data[number].linear > 0:
                    sig = 1
                else:
                    sig = -1
                return self.data[number].linear + sig*(self.freq[number]/1000 - self.data[number].linear/255)*0.5 *255
        else:
            ratio = self.data[number].angular / 1000 * 0.5
            if number < 3: # kola prawe
                #if message.angular > 0.0: #odwrotnie do ruchu wskazówek zegara, dodatnie skręt w lewo

                if self.freq[number] == 0.0:
                    #self.data_PWM[number] = self.data[number].linear + self.data[number].linear * ratio
                    return self.data[number].linear + self.data[number].linear * ratio
                else:
                    if self.data[number].linear > 0:
                        sig = 1
                    else:
                        sig = -1
                    #self.data_PWM[number] = self.data[number].linear + self.data[number].linear * ratio
                    pwm = self.data[number].linear + self.data[number].linear * ratio
                    return pwm + sig * (self.freq[number]/1000 - pwm/255)*0.5 *255
            else:
                if self.freq[number] == 0.0:
                    #self.data_PWM[number] = self.data[number].linear + self.data[number].linear * ratio
                    return self.data[number].linear - self.data[number].linear * ratio
                else:
                    #self.data_PWM[number] = self.data[number].linear + self.data[number].linear * ratio
                    if self.data[number].linear > 0:
                        sig = 1
                    else:
                        sig = -1
                    pwm = self.data[number].linear - self.data[number].linear * ratio
                    return pwm - sig *(self.freq[number]/1000 - pwm/255)*0.5 *255





    def pc_receive(self, message):
        if not self.motor_start and not message.motor_start:
            return
        else:
            if not self.demux_operation and not self.do_operation:
                if not self.motor_start and message.motor_start:
                    self.motor_start = message.motor_start
                    for i in range(6):
                        self.msg_to_publish_list[i].rs = True
                    self.demux_operation = True
                    self.send_to_motor_launch()
                elif self.motor_start and not message.motor_start:
                    self.motor_start = message.motor_start
                    for i in range(6):
                        self.msg_to_publish_list[i].rs = False
                    self.send_to_motor_shut_down()
                else:
                    for i in range(6):
                        vel = int(self.motor_velocity(i, message))
                        vel = min([255, vel])
                        vel = max([-255, vel])
                        if vel != 0:
                            if math.copysign(1, message.linear) != math.copysign(1,vel):
                                vel = 0
                        print(i, " ", vel)
                        self.msg_to_publish_list[i].value = np.abs(vel)
                        if vel >= 0:
                            self.msg_to_publish_list[i].fr = False
                        else:
                            self.msg_to_publish_list[i].fr = True
                    #print("\n")
                    self.send_to_motor()

    def operation(self):
        if not self.do_operation:
            return
        else:
            self.l = 0
            self.done = False
            self.demux_operation = False
            while self.l < 6:
                if not self.done:
                    if self.l == 6:  # czasami tu wchodzi
                        continue
                    rospy.loginfo("wysyłam ")
                    self.msg_to_publish_list[self.l].permission = True
                    self.pub_motor_list[self.l].publish(self.msg_to_publish_list[self.l])
                    self.done = not self.done
            self.demux_operation = False
            self.do_operation = False



    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                self.operation()
                if self.motor_status.count(Motor.BREAKING) == 6 and self.freq.count(0.0) == 6: #and prędkość == 0
                    self.do_operation = True
                if self.motor_status.count(Motor.NOT_WORKING) > 0 or self.motor_status.count(Motor.WAITING) > 0 or self.motor_status.count(Motor.BREAKING) > 0:
                    continue
                else:
                    if not self.do_operation and not self.demux_operation:
                        for i in range(6):
                            vel = int(self.motor_velocity(i))
                            vel = min([255, vel])
                            vel = max([-255, vel])
                            if vel != 0:
                                if math.copysign(1, self.data[i].linear) != math.copysign(1, vel):
                                    vel = 0
                            print(i, " ", vel)
                            self.msg_to_publish_list[i].value = np.abs(vel)
                            if vel >= 0:
                                self.msg_to_publish_list[i].fr = False
                            else:
                                self.msg_to_publish_list[i].fr = True
                        self.send_to_motor()
                        #print("\n")
                rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    rsp = Rasp()
    rsp.run()
