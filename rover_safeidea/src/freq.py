#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64MultiArray
import pigpio
import threading

list_GPIO_FREQ = [17,18,19,20,21,22]


class reader:
    """
    A class to read PWM pulses and calculate their frequency
    and duty cycle.  The frequency is how often the pulse
    happens per second.  The duty cycle is the percentage of
    pulse high time per cycle.
    """
    def __init__(self, pi, gpio, weighting=0.0):
        """
        Instantiate with the Pi and gpio of the PWM signal
        to monitor.

        Optionally a weighting may be specified.  This is a number
        between 0 and 1 and indicates how much the old reading
        affects the new reading.  It defaults to 0 which means
        the old reading has no effect.  This may be used to
        smooth the data.
        """
        self.pi = pi
        self.gpio = gpio

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new = 1.0 - weighting # Weighting for new reading.
        self._old = weighting       # Weighting for old reading.

        self._high_tick = None
        self._period = None
        self._high = None

        pi.set_mode(gpio, pigpio.INPUT)

        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):

        if level == 1:

            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)

                if self._period is not None:
                    self._period = (self._old * self._period) + (self._new * t)
                else:
                    self._period = t

            self._high_tick = tick

        elif level == 0:

            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)

                if self._high is not None:
                    self._high = (self._old * self._high) + (self._new * t)
                else:
                    self._high = t

    def frequency(self):
        """
        Returns the PWM frequency.
        """
        if self._period is not None:
            return 1000000.0 / self._period
        else:
            return 0.0

    # def pulse_width(self):
    #     """
    #     Returns the PWM pulse width in microseconds.
    #     """
    #     if self._high is not None:
    #         return self._high
    #     else:
    #         return 0.0

    def duty_cycle(self):
        """
        Returns the PWM duty cycle percentage.
        """
        if self._high is not None:
            return 100.0 * self._high / self._period
        else:
            return 0.0

    def cancel(self):
        """
        Cancels the reader and releases resources.
        """
        self._cb.cancel()


class Freq:

    def __init__(self):
        rospy.init_node('freq_node', disable_signals=True)
        self.num = rospy.get_param("~number")

        rospy.loginfo("Setting up the FREQ node number {0}...".format(self.num))
        topic_name = "rasp_freq_topic/{0}".format(self.num)

        self.GPIO_FREQ = list_GPIO_FREQ[self.num]

        self.pub_rasp = rospy.Publisher(topic_name, Float64MultiArray, queue_size=10)
        self.msg_to_publish = Float64MultiArray()

        rospy.sleep(1)

        self.pi = pigpio.pi()
        self.frequency = 0
        self.duty_cycle = 0
        self.finish = False


    def inf_loop(self):
        p = reader(self.pi, self.GPIO_FREQ)
        while not self.finish:
            time.sleep(0.1)
            self.frequency = p.frequency()
            self.duty_cycle = p.duty_cycle()
        p.cancel()

    def pub_to_rasp(self):
        if self.duty_cycle < 0.3:
            self.msg_to_publish.data = [self.num, 0.0]
        else:
            self.msg_to_publish.data = [self.num, self.frequency]
        self.pub_rasp.publish(self.msg_to_publish)

    def run(self):
        self.read = threading.Thread(target=self.inf_loop).start()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.pub_to_rasp()
                rate.sleep()
            except KeyboardInterrupt:
                self.finish = True
                rospy.loginfo("Ending.........")
                self.pi.stop()
                rospy.signal_shutdown('Quit')


if __name__ == "__main__":
    fr = Freq()
    fr.run()
