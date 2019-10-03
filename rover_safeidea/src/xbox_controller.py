#! /usr/bin/env python3

import rospy
import os, time, re
from evdev import InputDevice, categorize, ecodes, list_devices
from std_msgs.msg import String
from rover_msg.msg import Controller
from enumerate import Joy


class axis():

	def __init__(self, code, maxx, mini):
		self.code = code
		self.vmax = maxx
		self.vmin = mini


class controller():

	def __init__(self, device_dir='/dev/input/'):

		rospy.init_node('controller', disable_signals=True)
		rospy.loginfo("Setting up the Xbox controller node ...")

		#topic_name = rospy.get_param('controller_topic')

		self.pub = rospy.Publisher('controller_topic', Controller, queue_size=10)
		self.msg_to_publish = Controller()
		self.msg_to_publish.status = Joy.DISCONNECTED.value

		self.forward = 0
		self.backward = 0
		self.direction = 0

		rospy.sleep(1)
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
				rospy.loginfo("There's no controller found. Please connect one!")
				self.connect = False
				self.send_messege()
		else:
			strings = ['xbox', 'gamepad', 'controller', 'x-box', 'joystick', 'joy']
			for device in devices:
				for string in strings:
					search = re.compile(r"\b{0}\b".format(string), re.I)
					tab = search.findall(device.name)
					if len(tab) != 0:
						self.gamepad = device
						break
			self.connect = True
			rospy.loginfo("Connected controller: " + self.gamepad.name)
			self.msg_to_publish.status = Joy.CONNECTED.value
			self.send_messege()

	def send_messege(self):
		self.msg_to_publish.forward = self.forward
		self.msg_to_publish.backward = self.backward
		self.msg_to_publish.direction = self.direction
		self.pub.publish(self.msg_to_publish)

	def find_controller_codes(self):
		all = self.gamepad.capabilities()
		buttons = all[ecodes.EV_KEY]
		abs = dict(all[ecodes.EV_ABS])
		#print (self.gamepad.capabilities(verbose=True))

		if ecodes.BTN_A in buttons:
			self.btnA = ecodes.BTN_A
		if ecodes.BTN_B in buttons:
			self.btnB = ecodes.BTN_B
		if ecodes.BTN_X in buttons:
			self.btnY = ecodes.BTN_Y
		if ecodes.BTN_Y in buttons:
			self.btnX = ecodes.BTN_X
		if ecodes.BTN_TR in buttons:
			self.btnR1 = ecodes.BTN_TR
		if ecodes.BTN_TL in buttons:
			self.btnL1 = ecodes.BTN_TL
		if ecodes.BTN_START in buttons:
			self.btnSTART = ecodes.BTN_START
		if ecodes.BTN_SELECT in buttons:
			self.btnSELECT = ecodes.BTN_SELECT
		if ecodes.BTN_THUMBL in buttons:
			self.btnL3 = ecodes.BTN_THUMBL
		if ecodes.BTN_THUMBR in buttons:
			self.btnR3 = ecodes.BTN_THUMBR

		#lewy analog
		if ecodes.ABS_X in abs:
			info = abs[ecodes.ABS_X]
			self.LX = axis(ecodes.ABS_X,info.max,info.min)
		if ecodes.ABS_Y in abs:
			info = abs[ecodes.ABS_Y]
			self.LY = axis(ecodes.ABS_Y, info.max, info.min)
		#lewy trigger
		if ecodes.ABS_Z in abs:
			info = abs[ecodes.ABS_Z]
			self.L2 = axis(ecodes.ABS_Z, info.max, info.min)
		#prawy analog
		if ecodes.ABS_RX in abs:
			info = abs[ecodes.ABS_RX]
			self.RX = axis(ecodes.ABS_RX, info.max, info.min)
		if ecodes.ABS_RY in abs:
			info = abs[ecodes.ABS_RY]
			self.RY = axis(ecodes.ABS_RY, info.max, info.min)
		#prawy trigger
		if ecodes.ABS_RZ in abs:
			info = abs[ecodes.ABS_RZ]
			self.R2 = axis(ecodes.ABS_RZ, info.max, info.min)

		#DPAD
		if ecodes.ABS_HAT0X in abs:
			info = abs[ecodes.ABS_HAT0X]
			self.DX = axis(ecodes.ABS_HAT0X, info.max, info.min)
		if ecodes.ABS_HAT0Y in abs:
			info = abs[ecodes.ABS_HAT0Y]
			self.DY = axis(ecodes.ABS_HAT0Y, info.max, info.min)

	def button_event(self, event):
		if event.code == self.btnSELECT:
			self.msg_to_publish.status = Joy.NOT_WORKING.value
			self.send_messege()
			rospy.loginfo("Ending.........")
			rospy.signal_shutdown('Quit')
			return True
		return False

	def axis_event(self, event):
		if event.code == self.R2.code:
			self.forward = int((event.value - self.R2.vmin)/(self.R2.vmax-self.R2.vmin)*255)
			#rospy.loginfo("forward velocity: {0}%".format(self.forward))
		elif event.code == self.L2.code:
			self.backward = int((event.value - self.L2.vmin)/(self.L2.vmax-self.L2.vmin)*255)
			#rospy.loginfo("backward velocity: {0}%".format(self.backward))
		elif event.code == self.LX.code:
			if event.value < 0:
				val = event.value/self.LX.vmin
				if val < self.deadzone:
					val = 0
				self.direction = -int(val*1000)
				#rospy.loginfo("Turning right - {0}".format(self.direction))
			else:
				val = event.value / self.LX.vmax
				if val < self.deadzone:
					val = 0
				self.direction = int(val * 1000)
				#rospy.loginfo("Turning left - {0}".format(self.direction))

		self.send_messege()

	def run(self):
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			try:
				while not self.connect:
					self.detect_controller()
					#self.send_messege()
					rate.sleep()
			except KeyboardInterrupt:
				self.msg_to_publish.status = Joy.NOT_WORKING.value
				self.send_messege()
				rospy.loginfo("Ending.........")
				rospy.signal_shutdown('Quit')
				continue

			self.find_controller_codes()
			self.backward = 0
			self.forward = 0
			self.direction = 0
			self.deadzone = 0.1
			try:
				for event in self.gamepad.read_loop():
					#filters by event type
					if event.type == ecodes.EV_ABS:
						self.axis_event(event)
						#rospy.loginfo(event)
						#self.send_messege()
					elif event.type == ecodes.EV_KEY:
						sig = self.button_event(event)
						if sig:
							break
						#rospy.loginfo(categorize(event))
						#self.send_messege()
			except KeyboardInterrupt:
				self.msg_to_publish.status = Joy.NOT_WORKING.value
				self.send_messege()
				rospy.loginfo("Ending.......")
				rospy.signal_shutdown('Quit')
				continue
			except Exception as e:
				self.msg_to_publish.status = Joy.DISCONNECTED.value
				self.send_messege()
				rospy.loginfo("Controller is disconnected")
				self.connect = False
				continue

			rate.sleep()


if __name__ == "__main__":
	#device_dir = '/dev/input/'  # path to input devices
	xbox = controller()
	xbox.run()
