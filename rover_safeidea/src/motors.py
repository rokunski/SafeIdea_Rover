#! /usr/bin/env python3

import rospy
import os
import time
from rover_msg.msg import Master_Motor
import pigpio
from std_msgs.msg import Int32
from enumerate import Motor

# TODO: uaktualnic liste uzywanymi pinami
list_GPIO_PWM = [5, 7, 8, 6, 9, 10]

list_GPIO_FR_SR = [11,12,13,14,15,16] #0-3 nr wyjścia demuxa binarnie, 4 - wyłączenie demuxa, 5 - reset


class PWM:

    """
    This class may be used to generate PWM on multiple GPIO
    at the same time.

    The following diagram illustrates PWM for one GPIO.

    1      +------------+             +------------+
           |    GPIO    |             |    GPIO    |
           |<--- on --->|             |<--- on --->|
           |    time    |             |    time    |
    0 -----+            +-------------+            +---------
         on^         off^           on^         off^
      +--------------------------+--------------------------+
      ^                          ^                          ^
      |<------ cycle time ------>|<------ cycle time ------>|
    cycle                      cycle                      cycle
    start                      start                      start

    The underlying PWM frequency is the same for all GPIO and
    is the number of cycles per second (known as Hertz).

    The frequency may be specified in Hertz or by specifying
    the cycle time in microseconds (in which case a frequency
    of 1000000 / cycle time is set).

    set_frequency(frequency)
    set_cycle_time(micros)

    The PWM duty cycle (the proportion of on time to cycle time)
    and the start of the on time within each cycle may be set on
    a GPIO by GPIO basis.

    The GPIO PWM duty cycle may be set as a fraction of the
    cycle time (0-1.0) or as the on time in microseconds.

    set_pulse_length_in_micros(gpio, length)
    set_pulse_length_in_fraction(gpio, length)

    The GPIO PWM start time within each cycle may be set as
    a fraction of the cycle time from the cycle start (0-1.0)
    or as the number of microseconds from the start of the cycle.

    set_pulse_start_in_micros(gpio, start)
    set_pulse_start_in_fraction(gpio, start)

    There are two convenience functions to set both settings.

    set_pulse_start_and_length_in_micros(gpio, start, length)
    set_pulse_start_and_length_in_fraction(gpio, start, length)

    It doesn't matter whether you use the micros or fraction
    functions.  That is a matter of personal choice.

    pigpio waves are used to generate the PWM.  Note that only
    one wave can be transmitted at a time.  So if waves are being
    used to generate PWM they can't also be used at the same time
    for another purpose.

    A wave is generated of length 1000000/frequency microseconds.
    The GPIO are switched on and off within the wave to set the
    duty cycle for each GPIO.    The wave is repeatedly transmitted.

    Waves have a resolution of one microsecond.

    You will only get the requested frequency if it divides
    exactly into 1000000.

    For example, suppose you want a frequency of 7896 cycles per
    second.  The wave length will be 1000000/7896 or 126 microseconds
    (for an actual frequency of 7936.5) and there will be 126 steps
    between off and fully on.
    """


    def __init__(self, pi, frequency=1000):
        """
        Instantiate with the Pi.

        Optionally the frequency may be specified in Hertz (cycles
        per second).  The frequency defaults to 1000 Hertz.
        """
        self.pi = pi

        self.frequency = frequency
        self.micros = 1000000.0 / frequency

        self.used = False
        self.pS = 0.0
        self.pL = 0.0
        self.gpio = 0
        self.old_wid = None

        self.stop = False

    def set_frequency(self, frequency):
        """
        Sets the PWM frequency in Hertz.

        The change takes affect when the update function is called.
        """
        self.frequency = float(frequency)
        self.micros = 1000000.0 / self.frequency

    def set_cycle_time(self, micros):
        """
        Sets the PWM frequency by specifying the cycle time
        in microseconds.

        The frequency set will be one million divided by micros.

        The change takes affect when the update function is called.
        """
        self.micros = float(micros)
        self.frequency = 1000000.0 / self.micros

    def get_frequency(self):
        """
        Returns the frequency in Hertz.
        """
        return self.frequency

    def get_cycle_length(self):
        """
        Returns the cycle length in microseconds.
        """
        return self.micros

    def set_pulse_length_in_micros(self, gpio, length):
        """
        Sets the GPIO on for length microseconds per cycle.

        The PWM duty cycle for the GPIO will be:

        (length / cycle length in micros) per cycle

        The change takes affect when the update function is called.
        """
        length %= self.micros
        self.gpio = gpio

        self.pL = length / self.micros

        if not self.used:
           self.pi.set_mode(gpio, pigpio.OUTPUT)
           self.used = True

    def set_pulse_length_in_fraction(self, gpio, length):
        """
        Sets the GPIO on for length fraction of each cycle.

        The GPIO will be on for:

        (cycle length in micros * length) microseconds per cycle

        The change takes affect when the update function is called.
        """
        self.set_pulse_length_in_micros(gpio, self.micros * length)

    def set_pulse_start_in_micros(self, gpio, start):
       """
       Sets the GPIO high at start micros into each cycle.

       The change takes affect when the update function is called.
       """

       start %= self.micros
       self.gpio = gpio
       self.pS = start / self.micros

       if not self.used:
          self.pi.set_mode(gpio, pigpio.OUTPUT)
          self.used = True

    def set_pulse_start_in_fraction(self, gpio, start):
        """
        Sets the GPIO high at start fraction into each cycle.

        The change takes affect when the update function is called.
        """
        self.gpio = gpio
        self.set_pulse_start_in_micros(gpio, self.micros * start)

    def set_pulse_start_and_length_in_micros(self, gpio, start, length):
        """
        Sets the pulse start and length of each pulse in units
        of microseconds.

        The change takes affect when the update function is called.
        """
        self.gpio = gpio
        self.set_pulse_start_in_micros(gpio, start)
        self.set_pulse_length_in_micros(gpio, length)

    def set_pulse_start_and_length_in_fraction(self, gpio, start, length):
        """
        Sets the pulse start and length of each pulse as a
        fraction of the cycle length.

        The change takes affect when the update function is called.
        """
        self.gpio = gpio
        self.set_pulse_start_in_fraction(gpio, start)
        self.set_pulse_length_in_fraction(gpio, length)

    def get_GPIO_settings(self, gpio):
        """
        Returns the pulse start and pulse length as a fraction
        of the cycle time.
        """
        if self.used:
            return (True, self.pS, self.pL)
        else:
            return (False, 0.0, 0.0)

    def update(self):
        """
        Updates the PWM for each GPIO to reflect the current settings.
        """

        null_wave = True


        if self.used:

            null_wave = False

            on = int(self.pS * self.micros)
            length = int(self.pL * self.micros)

            micros = int(self.micros)

            if length <= 0:
                self.pi.wave_add_generic([pigpio.pulse(0, 1<<self.gpio, micros)])
            elif length >= micros:
                self.pi.wave_add_generic([pigpio.pulse(1<<self.gpio, 0, micros)])
            else:
                off = (on + length) % micros
                if on < off:
                    self.pi.wave_add_generic([
                        pigpio.pulse(   0, 1<<self.gpio,           on),
                        pigpio.pulse(1<<self.gpio,    0,     off - on),
                        pigpio.pulse(   0, 1<<self.gpio, micros - off),
                        ])
                else:
                    self.pi.wave_add_generic([
                        pigpio.pulse(1<<self.gpio,    0,         off),
                        pigpio.pulse(   0, 1<<self.gpio,    on - off),
                        pigpio.pulse(1<<self.gpio,    0, micros - on),
                        ])

        if not null_wave:

            if not self.stop:

                new_wid = self.pi.wave_create()

                if self.old_wid is not None:

                    self.pi.wave_send_using_mode(
                        new_wid, pigpio.WAVE_MODE_REPEAT_SYNC)

                 # Spin until the new wave has started.

                    while self.pi.wave_tx_at() != new_wid:

                        pass

                 # It is then safe to delete the old wave.

                    self.pi.wave_delete(self.old_wid)

                else:

                    self.pi.wave_send_repeat(new_wid)

                self.old_wid = new_wid

    def cancel(self):
        """
        Cancels PWM on the GPIO.
        """
        self.stop = True

        self.pi.wave_tx_stop()

        if self.old_wid is not None:
            self.pi.wave_delete(self.old_wid)


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
        self.pub_info = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.status = Motor.NOT_WORKING
        self.freq_prev = 0
        self.fr = False
        self.velocity = 0
        self.flag = False



        rospy.sleep(1)
        #self.pi = pigpio.pi()
        #self.pwm = PWM(self.pi)
        self.init_GPIOs()
        self.send2rasp()



    def rasp_receive(self, message):
        self.msg_to_PWM.value = message.value
        self.msg_to_PWM.fr = message.fr
        self.msg_to_PWM.rs = message.rs
        self.msg_to_PWM.permission = message.permission

    def send2rasp(self):
        self.flag = False
        self.pub_info.publish(self.status + 10 * self.num + 100 * self.msg_to_PWM.permission)
        self.flag = True

    def init_GPIOs(self):
        '''
        self.pi.set_PWM_frequency(self.GPIO_PWM, 500)
        self.pi.set_PWM_dutycycle(self.GPIO_PWM,   0)
        '''
        #self.pi.hardware_PWM(self.GPIO_PWM, 500, 0)
        #self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.01, 0.01)
        #self.pwm.update()

        #self.pwm.set_frequency(500)
        #self.pi.set_mode(self.GPIO_PWM, pigpio.OUTPUT)


    def set_GPIOs(self, num):

        #self.pi.write(list_GPIO_FR_SR[5], 0)
        #self.pi.write(list_GPIO_FR_SR[4], 0)
        binary = [int(x) for x in list('{0:0b}'.format(num))]
        print(binary)
        for i in range(4-len(binary)):
            binary.insert(0, 0)
        print(binary)
        binary = [1-x for x in binary]
        print(binary)
        #for i in range(4):
        #    self.pi.write(list_GPIO_FR_SR[3-i], binary[i])
        #self.pi.write(list_GPIO_FR_SR[4], 1)
        #rospy.sleep(0.001)
        #self.pi.write(list_GPIO_FR_SR[4], 0)
        #for i in range(4):
        #    self.pi.write(list_GPIO_FR_SR[i], 0)


    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                if self.status == Motor.NOT_WORKING and self.msg_to_PWM.rs and self.msg_to_PWM.permission:
                    # ustawianie odpowiedniego pinu/ włączenie sr
                    self.set_GPIOs(self.RS_num)
                    rospy.sleep(0.001)
                    self.status = Motor.WAITING
                    self.send2rasp()
                    while not self.flag:
                        continue
                    self.msg_to_PWM.permission = False
                    self.flag = False
                    rospy.loginfo(str(self.num) + " włączam")
                elif self.status == Motor.WAITING and not self.msg_to_PWM.rs and self.msg_to_PWM.permission:
                    # ustawianie odpowiedniego pinu/ wyłączenie sr
                    self.set_GPIOs(self.RS_num)
                    rospy.sleep(0.001)
                    self.status = Motor.NOT_WORKING
                    self.send2rasp()
                    while not self.flag:
                        continue
                    self.msg_to_PWM.permission = False
                    self.flag = False
                    rospy.loginfo(str(self.num) + " wyłączam")
                elif self.msg_to_PWM.rs:
                    if self.status == Motor.WAITING and self.msg_to_PWM.fr == self.fr:
                        if self.msg_to_PWM.value > 0:
                            #self.pi.set_PWM_dutycycle(self.GPIO_PWM, self.msg_to_PWM.value)

                            # if self.msg_to_PWM == 0:
                            #     self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.00, 0.0001)
                            # else:
                            #     self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.01, self.msg_to_PWM.value/255)
                            # self.pwm.update()
                            #
                            #self.pi.hardware_PWM(self.GPIO_PWM, 500, self.msg_to_PWM/255*1000000)
                            self.freq_prev = self.msg_to_PWM.value
                            if self.fr:
                                self.status = Motor.BACKWARD
                            else:
                                self.status = Motor.FORWARD
                            rospy.sleep(0.01)
                            self.send2rasp()
                            rospy.loginfo(str(self.num) + " dzialam")

                    elif self.status == Motor.WAITING and self.msg_to_PWM.fr != self.fr and self.msg_to_PWM.permission:
                        # zmień fr
                        self.set_GPIOs(self.FR_num)
                        rospy.sleep(0.001)
                        if self.msg_to_PWM.fr:
                            self.status = Motor.BACKWARD
                        else:
                            self.status = Motor.FORWARD
                        self.fr = not self.fr
                        self.send2rasp()
                        while not self.flag:
                            continue
                        self.msg_to_PWM.permission = False
                        self.flag = False
                        rospy.loginfo(str(self.num) + " zmieniam")

                    elif self.status == Motor.FORWARD and not self.msg_to_PWM.fr:
                        #ustawienie PWM na pin
                        if self.freq_prev != self.msg_to_PWM.value:
                            # if self.msg_to_PWM == 0:
                            #     self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.0, 0.0001)
                            # else:
                            #     self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.01, self.msg_to_PWM.value/255)
                            # self.pwm.update()
                            #

                            #self.pi.set_PWM_dutycycle(self.GPIO_PWM, self.msg_to_PWM.value)
                            #self.pi.hardware_PWM(self.GPIO_PWM, 500, self.msg_to_PWM/255*1000000)
                            self.freq_prev = self.msg_to_PWM.value
                            rospy.sleep(0.01)
                            self.send2rasp()
                            rospy.loginfo(str(self.num) + " jade")
                    elif self.status == Motor.BACKWARD and self.msg_to_PWM.fr:
                        # ustawienie PWM na pin
                        if self.freq_prev != self.msg_to_PWM.value:
                            #self.pi.hardware_PWM(self.GPIO_PWM, 500, self.msg_to_PWM/255*1000000)
                            #self.pi.set_PWM_dutycycle(self.GPIO_PWM, self.msg_to_PWM.value)

                            # if self.msg_to_PWM == 0:
                            #     self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.0, 0.0001)
                            # else:
                            #     self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.01, self.msg_to_PWM.value/255)
                            # self.pwm.update()
                            self.freq_prev = self.msg_to_PWM.value
                            self.send2rasp()
                            rospy.sleep(0.01)
                            rospy.loginfo(str(self.num) + " jade")
                    elif self.status == Motor.FORWARD and self.msg_to_PWM.fr:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)

                        # self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.00, 0.0001)
                        # self.pwm.update()
                        #
                        #self.pi.hardware_PWM(self.GPIO_PWM, 500, 0)
                        self.freq_prev = 0
                        self.status = Motor.BREAKING
                        self.send2rasp()
                        rospy.loginfo(str(self.num) + " hamuje")
                    elif self.status == Motor.BACKWARD and not self.msg_to_PWM.fr:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)
                        #self.pi.hardware_PWM(self.GPIO_PWM, 500, 0)

                        # self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.0,0.0001)
                        # self.pwm.update()
                        rospy.sleep(0.01)
                        self.freq_prev = 0
                        self.status = Motor.BREAKING
                        self.send2rasp()
                        rospy.loginfo(str(self.num) + " hamuje")
                    elif self.status == Motor.BREAKING and self.msg_to_PWM.permission:# and prędkość silnika == 0
                        #zmień fr
                        self.set_GPIOs(self.FR_num)
                        rospy.sleep(0.001)
                        if self.msg_to_PWM.fr:
                            self.status = Motor.BACKWARD
                        else:
                            self.status = Motor.FORWARD
                        self.fr = self.msg_to_PWM.fr
                        self.send2rasp()
                        while not self.flag:
                            continue
                        self.msg_to_PWM.permission = False
                        self.flag = False
                        rospy.loginfo(str(self.num) + " zmieniam")
                else:
                    if self.status == Motor.FORWARD:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)

                        # self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.0, 0.0001)
                        # self.pwm.update( )

                        #self.pi.hardware_PWM(self.GPIO_PWM, 500, 0)
                        self.freq_prev = 0
                        rospy.sleep(0.01)
                        self.status = Motor.BREAKING
                        self.send2rasp()
                        rospy.loginfo(str(self.num) + " hamuje")
                    elif self.status == Motor.BACKWARD:
                        #self.pi.set_PWM_dutycycle(self.GPIO_PWM, 0)

                        # self.pwm.set_pulse_start_and_length_in_fraction(self.GPIO_PWM, 0.0, 0.0001)
                        # self.pwm.update()

                        #self.pi.hardware_PWM(self.GPIO_PWM, 500, 0)
                        self.freq_prev = 0
                        rospy.sleep(0.01)
                        self.status = Motor.BREAKING
                        self.send2rasp()
                        rospy.loginfo(str(self.num) + " hamuje")
                    elif self.status == Motor.BREAKING and self.msg_to_PWM.permission:# and prędkość silnika == 0
                        #ustaw sr
                        self.set_GPIOs(self.RS_num)
                        rospy.sleep(0.001)
                        self.status = Motor.NOT_WORKING
                        self.send2rasp()
                        while not self.flag:
                            continue
                        self.msg_to_PWM.permission = False
                        self.flag = False
                        rospy.loginfo(str(self.num) + " wyłączam")

                rate.sleep()
            except KeyboardInterrupt:
                # self.pwm.cancel()
                # self.pi.stop()

                rospy.loginfo("Ending.........")
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    motos = MotorClass()
    motos.run()
