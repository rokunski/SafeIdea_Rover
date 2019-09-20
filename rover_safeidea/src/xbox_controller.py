#! /usr/bin/env python3

import rospy
import os, time, re
from evdev import InputDevice, categorize, ecodes, list_devices
from std_msgs.msg import String
from rover_msg.msg import Controller


class axis():

	def __init__(self, code, maxx, mini):
		self.code = code
		self.vmax = maxx
		self.vmin = mini


class controller():

	def __init__(self, device_dir='/dev/input/'):
		rospy.loginfo("Setting up the Xbox controller node ...")

		rospy.init_node('controller')

		self.pub = rospy.Publisher('controller_data', Controller, queue_size=10)
		self.msg_to_publish = Controller()
		self.msg_to_publish.status = False
		self.msg_to_publish.forward = 0
		self.msg_to_publish.backward = 0
		self.msg_to_publish.direction = 0

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
			#rospy.signal_shutdown('Quit')
		else:
			strings = ['xbox', 'gamepad', 'controller']
			for device in devices:
				for string in strings:
					search = re.compile(r"\b{0}\b".format(string), re.I)
					tab = search.findall(device.name)
					if len(tab) != 0:
						self.gamepad = device
						break
			self.connect = True
			rospy.loginfo("Connected controller: " + self.gamepad.name)
			self.msg_to_publish.status = True
			self.send_messege()

	def send_messege(self):
		try:
			self.msg_to_publish.forward = self.forward
			self.msg_to_publish.backward = self.backwards
			self.msg_to_publish.direction = self.direction
		except:
			a=1
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
			os.system("rosnode list | grep -v rosout | xargs rosnode kill")  #wyslij do glownego programu info raczej

	def axis_event(self,event):
		if event.code == self.R2.code:
			self.forward = int((event.value - self.R2.vmin)/(self.R2.vmax-self.R2.vmin)*100)
			rospy.loginfo("forward velocity: {0}%".format(self.forward))
		elif event.code == self.L2.code:
			self.backwards = int((event.value - self.L2.vmin)/(self.L2.vmax-self.L2.vmin)*100)
			rospy.loginfo("backward velocity: {0}%".format(self.backwards))
		elif event.code == self.LX.code:
			if event.value < 0:
				val = event.value/self.LX.vmin
				if val < self.deadzone:
					val = 0
				self.direction = -int(val*100)
				rospy.loginfo("Turning right - {0}".format(self.direction))
			else:
				val = event.value / self.LX.vmax
				if val < self.deadzone:
					val = 0
				self.direction = int(val * 100)
				rospy.loginfo("Turning left - {0}".format(self.direction))

		self.send_messege()

	def run(self):
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			while not self.connect:
				self.detect_controller()
				self.send_messege()
				rate.sleep()

			self.find_controller_codes()
			self.backwards = 0
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
						self.button_event(event)
						#rospy.loginfo(categorize(event))
						#self.send_messege()
					if rospy.is_shutdown():
						break
			except:
				rospy.loginfo("Controller is disconected")
				os.system("rosnode list | grep -v rosout | xargs rosnode kill")

			rate.sleep()

if __name__ == "__main__":
	#device_dir = '/dev/input/'  # path to input devices
	xbox = controller()
	xbox.run()
