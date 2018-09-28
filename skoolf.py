#!/usr/env python
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from enum import Enum
import sys
import bluepy
import uuid
from bluepy.btle import Scanner, Peripheral, Characteristic
from pathlib import Path
from bluepy.btle import BTLEException
import time
import os.path
import pwd
import json
import random
import platform


CMD_STARTRIGHT    = 0x10
CMD_STARTLEFT     = 0x11
CMD_STOPTURN 	  = 0x15

CMD_RIGHT_30 	  = 0x08
CMD_LEFT_30 	  = 0x09

CMD_RIGHT    = 0x10
CMD_LEFT     = 0x11
CMD_FORWARD  = 0x12
CMD_BACKWARD = 0x13
CMD_STOP     = 0x14
CMD_SLEEP    = 0x15

#    cmd_step_mode = 0x16
#    cmd_buzzer = 0x17
#    cmd_get_ambient = 0x21
CMD_GET_DISTANCE = 0x22
#    cmd_record = 0x30
#    cmd_increase_gain = 0x31
#    cmd_decrease_gain = 0x32
CMD_ROVER_MODE = 0x40

class SkoobotController:
	"""
	Control API for Skoobots
	"""

	def __init__(self):
		self.transport = TransportBluepy()
		self.uuids = {
		"cmd" : "00001525-1212-efde-1523-785feabcd123",
		"data" : "00001524-1212-efde-1523-785feabcd123"
		}
		self.connectedSkoobot = None

	def connect(self, name=None, addr=None):
		"""
		Connect to the given Skoobot.
		If no Skoobot is given, connects to the default.
		Returns the address of the connected Skoobot if successful;
		None otherwise.
		"""
		addrList = []

		if addr != None:
			if isinstance(addr, str):
				addrList.append(addr)
			else:
				raise TypeError("addr should be a string")

		if self.connectedSkoobot != None:
			self.disconnect()

		for botAddr in addrList:
			try:
				self.transport.connect(botAddr)
				self.connectedSkoobot = botAddr
				break
			except BTLEException:
				pass

		return self.connectedSkoobot

	def disconnect(self):
		self.transport.disconnect()
		self.connectedSkoobot = None

	def sendCommand(self, data, waitForResponse=False):
		if self.connectedSkoobot == None:
			raise RuntimeError("BLE not connected")
		data = int(data);
		cmdBytes = data.to_bytes(1, byteorder="little") 
		characteristics = self.transport.getRawCharacteristicsByUUID(self.uuids["cmd"])
		if len(characteristics) == 0:
			raise RuntimeError("cmd characteristic not supported by firmware")
		cmd = characteristics[0]
		cmd.write(cmdBytes, waitForResponse)

	def readData(self):
		if self.connectedSkoobot == None:
			raise RuntimeError("BLE not connected")
			characteristics = self.transport.getRawCharacteristicsByUUID(self.uuids["data"])
		if len(characteristics) == 0:
			raise RuntimeError("data characteristic not supported by firmware")
			charac = characteristics[0]
			dataBytes = charac.read()
			data = int.from_bytes(dataBytes, byteorder="little")
			return data

	def cmdRight(self):
		self.sendCommand(CMD_RIGHT, True)
		
	def cmdLeft(self):
		self.sendCommand(CMD_LEFT, True)

	def cmdForward(self):
		self.sendCommand(CMD_FORWARD, True)
	
	def cmdStartRight(self):
		self.sendCommand(CMD_RIGHT_30, True)

	def cmdStartLeft(self):
		self.sendCommand(CMD_LEFT_30, True)

	def cmdStopTurn(self):
		self.sendCommand(CMD_STOPTURN, True)
		
	def cmdBackward(self):
		self.sendCommand(CMD_BACKWARD, True)
		
	def cmdStop(self):
		self.sendCommand(CMD_STOP, True)

	def cmdSleep(self):
		self.sendCommand(CMD_SLEEP, True)

	def cmdRoverMode(self):
		self.sendCommand(CMD_ROVER_MODE, True)


	def requestDistance(self):
		self.sendCommand(CMD_GET_DISTANCE, True)
		return self.readData()

def control():
	
	global controller

	controller = SkoobotController()

	name = "Skoobot"
	baddr = "F8:F7:81:EB:0F:66"
	changeDefault = False;
	doRegister = False;

	addr = controller.connect(name, baddr)
	if addr == None:
		print("Unable to connect to skoobot")
		exit(1)

    
def control_disconnect():
	
	global controller 
	controller.disconnect()


class TransportBluepy():
    def __init__(self):
        self.devices = []
        self.peripheral = None

    def findRawDevices(self, timeout=1.0):
        rawDevices = []
        scanner = Scanner()
        rawDevices = scanner.scan(timeout)

        return rawDevices

    def rawDeviceInfoStr(self, rawDevice):
        """
        Convert the raw device into an info string.

        The format of the string is transport-specific.
        """
        info = "Address: {0:s}\n".format(rawDevice.addr)
        info += "Address type: {0:s}\n".format("public" if rawDevice.addrType == bluepy.btle.ADDR_TYPE_PUBLIC else "random")
        info += "Connections?: {0:s}\n".format("yes" if rawDevice.connectable else "no")
        info += "Scan Data:\n"
        scanData = rawDevice.getScanData()
        for scanRow in scanData:
            info += "    {0:d}\t| {1:s}\t| {2:s}\n".format(scanRow[0], scanRow[1], scanRow[2])
        if rawDevice.connectable:
            self.connect(rawDevice.addr, rawDevice.addrType)
            info += "Services:\n"
            rawServices = self.getRawServices()
            for rawService in rawServices:
                info += "{0:s}".format(self.rawServiceInfoStr(rawService))
            self.disconnect()
        
        return info

    def rawServiceInfoStr(self, rawService):
        info = "    UUID: {0:s}\n".format(str(rawService.uuid))
        info += "    Characteristics:\n"
        rawCharacteristics = self.getRawCharacteristicsForService(rawService) 
        for rawCharacteristic in rawCharacteristics:
            info += "{0:s}".format(self.rawCharacteristicInfoStr(rawCharacteristic))
        return info

    def rawCharacteristicInfoStr(self, rawCharacteristic):
        info = "    -   UUID: {0:s}\n".format(str(rawCharacteristic.uuid))
        info += "    -   Handle: {0:d} (0x{0:x})\n".format(rawCharacteristic.getHandle())
        info += "    -   Properties: {0:s}\n".format(rawCharacteristic.propertiesToString())
        return info
        
    def connect(self, addr, addrType=bluepy.btle.ADDR_TYPE_RANDOM):
        self.disconnect()
        self.peripheral = Peripheral(addr, addrType)
        
    def disconnect(self):
        if self.peripheral != None:
            self.peripheral.disconnect()
            self.peripheral = None

    def getRawServices(self):
        if self.peripheral == None:
            print("Not connected.\n")
            return {}
        rawServices = self.peripheral.getServices()
        print("Found {0:d} services\n".format(len(rawServices)))
        return rawServices

    def getRawCharacteristicsForService(self, service):
        return service.getCharacteristics()

    def getRawCharacteristicsByUUID(self, uuid):
        results = []
        if self.peripheral != None:
            results = self.peripheral.getCharacteristics(uuid=uuid)
        return results

class dirs(Enum):
	leftAndDown = 0
	leftAndUp = 1
	rightAndDown = 3
	rightAndUp = 4

first_frame = 0
Kernel_size=15
low_threshold=40
high_threshold=120

rho0=10
threshold=15
theta0=np.pi/180
minLineLength=10
maxLineGap=1

maincont = 0
row = 0
col = 0
going = dirs.leftAndDown
imago = 0
slope = 0.0
vx = 0.0

def main():

	global maincont,row,col,going,imago,slope,vx
		
	# initialize the camera and grab a reference to the raw camera capture
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 10
	rawCapture = PiRGBArray(camera, size=(640, 480))

	# Create a blank 300x300 black image
	imago = np.zeros((640, 480, 3), np.uint8)
	# Fill image with white color(set each pixel to white)
	imago[:] = (255, 255, 255)
	 
	# allow the camera to warmup
	time.sleep(0.1)

	# grab an image from the camera
	camera.capture(rawCapture, format="bgr")
	imager = rawCapture.array
	b0 = imager.copy()
	# set green and red channels to 0
	#b[:, :, 1] = 0
	#b[:, :, 2] = 0	
	grayscaled0 = cv2.cvtColor(b0,cv2.COLOR_BGR2GRAY)
	#ret,th = cv2.threshold(grayscaled,127,255,cv2.THRESH_BINARY)
	th0 = cv2.adaptiveThreshold(grayscaled0, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 81, 6)

	#Perform canny edge-detection.
	#If a pixel gradient is higher than high_threshold is considered as an edge.
	#if a pixel gradient is lower than low_threshold is is rejected , it is not an edge.
	#Bigger high_threshold values will provoque to find less edges.
	#Canny recommended ratio upper:lower  between 2:1 or 3:1
	edged0 = cv2.Canny(th0, low_threshold, high_threshold)
			
	im2, contours0, hierarchy = cv2.findContours(edged0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	nav_pts = np.zeros((0,0,3), np.uint8)
	for cont in contours0:
		x,y,w,h = cv2.boundingRect(cont)
		if w > 160 and y < 300:
			maincont = cont
			cv2.drawContours(imago, [cont], -1, (0,0,0), 3)
			pixelpoints = np.transpose(np.nonzero(~imago))
			for pt in pixelpoints:
				nav_pts = np.append(nav_pts,pt)
			break
			
	row = nav_pts.item(0)
	col = nav_pts.item(1)
	cv2.circle(imago,(col,row),20,(0,0,255),1)
	print(imago[row, col])
	cv2.imshow("Frame", imago)
	cv2.waitKey(0)
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	row_robot = 0
	col_robot = 0
	
	control()
	start = 1
		
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array
	 
		# Make new image and fill with white color(set each pixel to white)
		imago = np.zeros((640, 480, 3), np.uint8)
		imago[:] = (255, 255, 255)
		# Add the trace back in, set it to black
		cv2.drawContours(imago, [maincont], -1, (0,0,0), 3)
	
		#do pixel walk before drawing anything besides contour, because I check pixels for black (0,0,0)
		if start == 0:
			do_pixel_walk()

		# get a section of image to place in corner
		rowhigh = 20
		rowlow = 20
		colhigh = 20
		collow = 20
		if row-rowlow < 0:
			rowlow = abs(0 - row)
		if row+rowhigh > imago.shape[0]:
			rowhigh = imago.shape[0] - row
		if col-collow < 0:	
			collow = abs(0-col)
		if col+colhigh > imago.shape[1]:
			colhigh = imago.shape[1] - col
		section = imago[row-rowlow:row+rowhigh, col-collow:col+colhigh]

		#fit a line to section
		slicer = cv2.cvtColor(section,cv2.COLOR_BGR2GRAY)
		points = cv2.findNonZero(~slicer)
		[vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
		#calculate y intercept on left
		lefty = int((-x*vy/vx) + y)
		#calculate y intercept on right
		righty = int(((slicer.shape[1]-x)*vy/vx)+y)
		cv2.line(section,(slicer.shape[1]-1,righty),(0,lefty),255,2)
		slope = vy/vx
		cv2.circle(imago,(col,row),20,(0,0,255),1)
		
		imago[20:20+rowlow+rowhigh,20:20+collow+colhigh] = imago[row-rowlow:row+rowhigh, col-collow:col+colhigh]

		# find green robot and make rectangle
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_green = np.array([50,50,50])
		upper_green = np.array([90,255,255])
		mask = cv2.inRange(hsv, lower_green, upper_green)
		imask = mask>0
		green = np.zeros_like(image, np.uint8)
		green[:] = (255,255,255)
		green[imask] = image[imask]			 
		grayed = cv2.cvtColor(green,cv2.COLOR_BGR2GRAY)
		th = cv2.adaptiveThreshold(grayed, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 81, 6)
		edged = cv2.Canny(th, low_threshold, high_threshold)
		im2, contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		found_robot = 0
		for cnt in contours:
			x,y,w,h = cv2.boundingRect(cnt)
			if w > 15:
				cv2.putText(imago, "Green Object Detected", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1, 1)
				cv2.drawContours(imago, [cnt], -1, (0,255,0), 3)
				M = cv2.moments(cnt)
				if M["m00"] != 0.0:
					col_robot = int(M["m10"] / M["m00"])
					row_robot = int(M["m01"] / M["m00"])
					if row_robot >= 0 and col_robot >= 0:
						cv2.circle(imago,(col_robot,row_robot),20,(0,255,0),1)
						found_robot = 1
				break
		
		print("robot ",row_robot,col_robot)
		print("line ",row,col)
		if col_robot - col != 0:
			slope_to_go = (row_robot - row)/(col_robot - col)
		else:
			print("Have col zeros")
		print("Slope to go = ",slope_to_go, " vx = ",vx)
			
		if found_robot == 1 and start == 1:
			controller.cmdForward()
			start = 0
			print("Robot found start")
		elif found_robot == 1:
			print("Robot found")
			controller.cmdForward()
		else:
			print("Robot missed")
			controller.cmdStop()
		
		if found_robot == 1:
			num_turns = abs(int(slope_to_go))
			while num_turns > 0:
				if slope_to_go > 0.0:
					controller.cmdStartRight()	
					print("Right")		
				else:
					controller.cmdStartLeft()
					print("Left")
				num_turns = num_turns - 1
			
		# show the frame
		cv2.imshow("Frame", imago)
		key = cv2.waitKey(1) & 0xFF
	 
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
	 
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			control_disconnect()
			break

def do_pixel_walk():	
	global going,row,col,maincont,imago,vx
		
	i = 0
	while i < 45:

		#going left or down
		if going == dirs.leftAndDown:	#can go left or down
			print("left and down")
			col = col - 1
			if np.all(imago[row,col]) != 0:	#fail
				#can't go left, go down
				col = col + 1 #rewind left
				row = row + 1
				if np.all(imago[row,col]) != 0:	#fail
					if abs(vx) < 0.04:	#is vertical line
						print("vx = ",vx)
						going = dirs.rightAndDown
					else:
						#can't go down, go up
						row = row - 1
						row = row - 1
						if np.all(imago[row,col]) == 0:	#good
							going = dirs.leftAndUp
						else:
							row = row + 1
							going = dirs.rightAndDown
		
		#going left and up
		if going == dirs.leftAndUp:	#left
			print("left and up")
			col = col - 1
			if np.all(imago[row,col]) != 0:	#fail
				#can't go left, go up
				col = col + 1 #rewind left
				row = row - 1
				if np.all(imago[row,col]) != 0:	#fail
					if abs(vx) < 0.04:	#is vertical line
						print("vx = ",vx)
						going = dirs.rightAndUp
					else:
						#can't go up, go down
						row = row + 1
						row = row + 1
						if np.all(imago[row,col]) == 0:	#good
							going = dirs.leftAndDown
						else:
							row = row - 1
							going = dirs.rightAndDown
	
		#going right or down
		if going == dirs.rightAndDown:	#can go right or down
			print("right and down")
			col = col + 1
			if np.all(imago[row,col]) != 0:	#fail
				#can't go left, go down
				col = col - 1 #rewind left
				row = row + 1
				if np.all(imago[row,col]) != 0:	#fail
					if abs(vx) < 0.04:	#is vertical line
						print("vx = ",vx)
						going = dirs.leftAndDown
					else:
						#can't go down, go up
						row = row - 1
						row = row - 1
						if np.all(imago[row,col]) == 0:	#good
							going = dirs.rightAndUp
						else:
							row = row + 1
							going = dirs.leftAndDown
		
		#going right and up
		if going == dirs.rightAndUp:	
			print("right and up")
			col = col + 1
			if np.all(imago[row,col]) != 0:	#fail
				#can't go right, go up
				col = col - 1 #rewind left
				row = row - 1
				if np.all(imago[row,col]) != 0:	#fail
					if abs(vx) < 0.04:	#is vertical line
						print("vx = ",vx)
						going = dirs.leftAndUp
					else:
						#can't go up, go down
						row = row + 1
						row = row + 1
						if np.all(imago[row,col]) == 0:	#good
							going = dirs.rightAndDown
						else:
							row = row - 1
							going = dirs.leftAndDown
		i = i + 1

def scan():
    transport = TransportBluepy()
    registry = SkoobotRegistry()

    rawDevices = transport.findRawDevices()
    skoobots = []
    for device in rawDevices:
        scanList = device.getScanData()
        for scanItem in scanList:
            if scanItem[0] == 9 and scanItem[2] == "Skoobot":
                skoobots.append(device)

    for skoobot in skoobots:
        # print(transport.rawDeviceInfoStr(skoobot))
        addr = skoobot.addr
        registry.addSkoobot(addr)
        name = registry.getSkoobotsByAddress(addr)[0][1]
        if registry.getDefaultName() == None:
            registry.setDefault(name)
        defaultText = " (default)" if registry.getDefaultName() == name else ""
        msg = "Added Skoobot {0:s} to registry with name {1:s}{2:s}"
        print(msg.format(addr, name, defaultText))
    print("Saving to list of Skoobots to registry {0:s}".format(registry.registryPath))
    registry.save()
    shutil.chown(registry.registryPath, pwd.getpwuid(os.getuid())[0])
    
if __name__ == '__main__':
    main()
