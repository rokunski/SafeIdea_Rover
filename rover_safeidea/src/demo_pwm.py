import pigpio
import time

GPIO = [5, 6, 7, 8, 9, 10, 11, 12]

pi = pigpio.pi()

if not pi.connected:
      print("Can't detect Pi")
      exit(0)

pi.set_PWM_frequency(5, 1000)

for i in range(256):
    pi.set_PWM_dutycycle(5, i)
    time.sleep(0.1)

for i in range(256):
    pi.set_PWM_dutycycle(5, 255 - i)
    time.sleep(0.1)

pi.stop()