#! /usr/bin/env python3

import rospy
import os
import time
from rover_msg.msg import Master_Motor
import pigpio
from std_msgs.msg import Byte
from enumerate import Motor

# TODO: uaktualnic liste uzywanymi pinami
list_GPIO_PWM = [5, 6, 7, 8, 9, 10]

list_GPIO_FR_SR = [11,12,13,14,15,16] #0-3 nr wyjścia demuxa binarnie, 4 - wyłączenie demuxa, 5 - reset




class MotorClass:
    def __init__(self):
        rospy.init_node('motor_node', disable_signals=True)
        self.num = rospy.get_param("~number")

        rospy.loginfo("Setting up the Motor node number {0}...".format(self.num))
        topic_name = "rasp_motor_topic/{0}".format(self.num)

        self.GPIO_PWM = list_GPIO_PWM[self.num]
        self.RS_num = 2 * self.num
        self.FR_num = 2 * self.num + 1

        self.sub_rasp = rospy.Subscriber(topic_name, Master_Motor, self.rasp_receive)
        self.msg_to_PWM = Master_Motor()

        topic_name = "motor_status/{0}".format(self.num)
        self.pub_info = rospy.Publisher(topic_name, Byte, queue_size=10)
        self.status = Motor.NOT_WORKING
        self.freq_prev = 0
        self.fr = False
        self.velocity = 0


        rospy.sleep(1)
        #self.pi = pigpio.pi()



    def rasp_receive(self, message):
        self.msg_to_PWM.value = message.value
        self.msg_to_PWM.fr = message.fr
        self.msg_to_PWM.rs = message.rs
        self.msg_to_PWM.permission = message.permission

    def send2rasp(self):
        self.pub_info.publish(self.status + 10 * self.num)

    def init_GPIOs(self):
        '''
        self.pi.set_PWM_frequency(self.GPIO_PWM, 500)
        self.pi.set_PWM_dutycycle(self.GPIO_PWM,   0)
        '''

    def set_GPIOs(self, num):
        '''
        self.pi.write(list_GPIO_FR_SR[4], 1)
        binary = [int(x) for x in list('{0:0b}'.format(8))]
        for i in range(4-len(binary)):
            binary.append(0)
        binary = [1-x for x in binary]
        for i in range(4):
            self.pi.write(list_GPIO_FR_SR[i], binary[i])
        self.pi.write(list_GPIO_FR_SR[4], 0)
        rospy.sleep(0.001)
        self.pi.write(list_GPIO_FR_SR[4], 1)
        for i in range(4):
            self.pi.write(list_GPIO_FR_SR[i], 0)
        '''

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                if self.status == Motor.NOT_WORKING and self.msg_to_PWM.rs and self.msg_to_PWM.permission:
                    # ustawianie odpowiedniego pinu/ włączenie sr
                    rospy.sleep(0.001)
                    self.status = Motor.WAITING
                    self.send2rasp()
                    self.msg_to_PWM.permission = False
                    rospy.loginfo(str(self.num) + " włączam")
                elif self.status == Motor.WAITING and not self.msg_to_PWM.rs and self.msg_to_PWM.permission:
                    # ustawianie odpowiedniego pinu/ wyłączenie sr
                    rospy.sleep(0.001)
                    self.status = Motor.NOT_WORKING
                    self.send2rasp()
                    self.msg_to_PWM.permission = False
                    rospy.loginfo(str(self.num) + " wyłączam")
                elif self.msg_to_PWM.rs:
                    if self.status == Motor.WAITING and self.msg_to_PWM.fr == self.fr:
                        if self.msg_to_PWM.value > 0:
                            # self.pi.set_PWM_dutycycle(self.GPIO_PWM, self.msg_to_PWM.value)
                            self.freq_prev = self.msg_to_PWM.value
                            if self.fr:
                                self.status = Motor.BACKWARD
                            else:
                                self.status = Motor.FORWARD
                            self.send2rasp()
                            rospy.loginfo(str(self.num) + " dzialam")

                    elif self.status == Motor.WAITING and self.msg_to_PWM.fr != self.fr and self.msg_to_PWM.permission:
                        # zmień fr
                        if self.msg_to_PWM.fr:
                            self.status = Motor.BACKWARD
                        else:
                            self.status = Motor.FORWARD
                        self.fr = not self.fr
                        self.send2rasp()
                        self.msg_to_PWM.permission = False
                        rospy.loginfo(str(self.num) + " zmieniam")

                    elif self.status == Motor.FORWARD and not self.msg_to_PWM.fr:
                        #ustawienie PWM na pin
                        if self.freq_prev != self.msg_to_PWM.value:
                            #self.pi.set_PWM_dutycycle(self.GPIO_PWM, self.msg_to_PWM.value)
                            self.freq_prev = self.msg_to_PWM.value
                            self.send2rasp()
                    elif self.status == Motor.BACKWARD and self.msg_to_PWM.fr:
                        # ustawienie PWM na pin
                        if self.freq_prev != self.msg_to_PWM.value:
                            #self.pi.set_PWM_dutycycle(self.GPIO_PWM, self.msg_to_PWM.value)
                            self.freq_prev = self.msg_to_PWM.value
                            self.send2rasp()
                    elif self.status == Motor.FORWARD and self.msg_to_PWM.fr:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)
                        self.freq_prev = 0
                        self.status = Motor.BREAKING
                        self.send2rasp()
                    elif self.status == Motor.BACKWARD and not self.msg_to_PWM.fr:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)
                        self.freq_prev = 0
                        self.status = Motor.BREAKING
                        self.send2rasp()
                    elif self.status == Motor.BREAKING and self.msg_to_PWM.permission:# and prędkość silnika == 0
                        #zmień fr
                        if self.msg_to_PWM.fr:
                            self.status = Motor.BACKWARD
                        else:
                            self.status = Motor.FORWARD
                        self.fr = self.msg_to_PWM.fr
                        self.send2rasp()
                        self.msg_to_PWM.permission = False
                        rospy.loginfo(str(self.num) + " zmieniam")
                else:
                    if self.status == Motor.FORWARD:
                       # self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)
                        self.freq_prev = 0
                        self.status = Motor.BREAKING
                        self.send2rasp()
                    elif self.status == Motor.BACKWARD:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)
                        self.freq_prev = 0
                        self.status = Motor.BREAKING
                        self.send2rasp()
                    elif self.status == Motor.BREAKING and self.msg_to_PWM.permission:# and prędkość silnika == 0
                        #ustaw sr
                        self.status = Motor.NOT_WORKING
                        self.send2rasp()
                        self.msg_to_PWM.permission = False
                        rospy.loginfo(str(self.num) + " wyłączam")

                rate.sleep()
            except KeyboardInterrupt:
                #self.pi.stop()
                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    motos = MotorClass()
    motos.run()
