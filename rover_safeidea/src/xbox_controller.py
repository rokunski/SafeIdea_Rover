#! /usr/bin/env python3

import rospy
import os, time, re
from evdev import InputDevice, categorize, ecodes, list_devices


class controller():

	def __init__(self, device_dir='/dev/input/'):
		rospy.loginfo("Setting up the Xbox controller node ...")

		rospy.init_node('controller')

		self.connect = None
		self.device_dir = device_dir
		self.detect_controller()

	def detect_controller(self):

		def devicenum(device_path):
			digits = re.findall(r'\d+$', device_path)
			return [int(i) for i in digits]

		devices = sorted(list_devices(self.device_dir), key=devicenum)  # search for connected devices
		devices = [InputDevice(path) for path in devices]
		if not devices:
			if self.connect is None:
				rospy.loginfo("There no controller found. Please connect one!")
				self.connect = False
			#rospy.signal_shutdown('Quit')
		else:
			strings = ['xbox','gamepad','controller']
			for device in devices:
				for string in strings:
					search = re.compile(r"\b{0}\b".format(string), re.I)
					tab = search.findall(device.name)
					if len(tab) != 0:
						self.gamepad = device
						break
			self.connect = True
			rospy.loginfo("Connected controller: " + self.gamepad.name)

	def run(self):
		rospy.Rate(30)
		while not rospy.is_shutdown():
			while not self.connect:
				self.detect_controller()


if __name__ == "__main__":
	#device_dir = '/dev/input/'  # path to input devices
	xbox = controller()
	xbox.run()


#devID = '/dev/input/event21'
#while not os.path.exists(devID):
#	time.sleep(0.01)
#	rospy.loginfo("Please, connect the controller")
#
# try:
# 	gamepad = InputDevice(devID)
# 	rospy.loginfo(gamepad)
# except:
# 	rospy.loginfo("Cannot connect to controller")
#
# print(gamepad.capabilities(verbose=True))
#
# for event in gamepad.read_loop():
# 	#filters by event type
# 	if event.type == ecodes.EV_ABS:
# 		print(event)
# 	elif event.type == ecodes.EV_KEY:
# 		print(event)
# 	if rospy.is_shutdown():
# 		break
