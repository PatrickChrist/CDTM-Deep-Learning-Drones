#########
# ps_drone.py
# (w)+(c) J. Philipp de Graaff, www.playsheep.de, drone@playsheep.de, 2012-2014
# Project homepage: www.playsheep.de/drone and https://sourceforge.net/projects/ps-drone/
# Dependencies: a POSIX OS, openCV2 for video-support.
# Base-program of the PS-Drone API: "An open and enhanced API for universal control of the Parrot AR.Drone 2.0 quadcopter."
##########
# Modified and advanced version, based on a part of the master of computer science degree dissertation "Universelle
#   Kontrolle und Ueberwachung einer Parrot AR.Drone 2.0 auf Basis eines offenen und erweiterten Toolkits"
#   by J. Philipp de Graaff, faculty of computer science, Prof. Dr. Hedrich, at the University of Frankfurt / Germany
#   Linked at http://www.em.cs.uni-frankfurt.de/index.php?id=43&L=1
# For further details, information, documentation or tutorials visit: www.playsheep.de/drone
##########
# LICENCE:
#   Artistic License 2.0 as seen on http://opensource.org/licenses/artistic-license-2.0 (retrieved December 2014)
#   If the terms of this license do not permit the full use that you propose to make of PS-Drone, please contact me for a
#   different licensing arrangement.
#   Visit www.playsheep.de/drone or see the PS-Drone-API-documentation for an abstract from the Artistic License 2.0.
##########
# Dedicated to my beloved wife.
###########

import threading, select, socket, time, tempfile, multiprocessing, struct, os, sys
import thread, signal, subprocess

if os.name == 'posix':	import termios, fcntl	# for getKey(), ToDo: Reprogram for Windows

commitsuicideV, showVid, vCruns, lockV, debugV =	False, False, False, threading.Lock(), False	# Global variables for video-decoding
offsetND, suicideND, commitsuicideND = 0, False, False												# Global variables for NavDava-decoding

class Drone(object):
######################################=-
### Start and stop using the drone ###=-
######################################=-
	###### Bootup and base configuration
	def __init__(self):
		self.__Version = 		"2.0.2"
		self.__lock = 			threading.Lock()	# To prevent semaphores
		self.__startTime = 		time.time()
		self.__speed = 			0.2					# Default drone moving speed in percent.
		self.showCommands = 	False				# Shows all sent commands (but not the keepalives)
		self.debug = 			False				# Shows some additional debug information
		self.valueCorrection = 	False
		self.selfRotation = 	0.0185				# use this value, if not checked by getSelfRotation()
		self.stopOnComLoss = 	False				# when there is a communication-problem, drone will land or not

		# Drone communication variables
		self.DroneIP = 		"192.168.1.1"
		self.NavDataPort = 	5554
		self.VideoPort = 	5555
		self.CmdPort = 		5556
		self.CTLPort = 		5559

		# NavData variables
		self.__NavData = 				""
		self.__State = 					[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.__NavDataCount = 			0
		self.__NavDataTimeStamp = 		0.0
		self.__NavDataDecodingTime = 	0.0
		self.__NoNavData = 				False

		# Video variables
		self.__VideoImage = 			None
		self.__VideoImageCount = 		0
		self.__VideoDecodeTimeStamp = 	0
		self.__VideoDecodeTime = 		0
		self.__VideoReady = 			False
		self.__vKey =					""
		self.__SaveVideo = 				False

		# Config variables
		self.__ConfigData = 			[]
		self.__ConfigDataCount = 		0
		self.__ConfigDataTimeStamp = 	0
		self.__ConfigSending = 			True
		self.__ConfigSessionID = 		"03016321"
		self.__ConfigUserID = 			"0a100407"
		self.__ConfigApplicationID = 	"03016321"
		self.sendConfigSaveMode = 		False

		# Internal variables
		self.__NavDataProcess =			""
		self.__VideoProcess =			""
		self.__vDecodeProcess =			""
		self.__ConfigQueue =			[]
		self.__networksuicide =			False
		self.__receiveDataRunning =		False
		self.__sendConfigRunning =		False
		self.__shutdown = 				False
		self.__pDefaultStr = 			"\033[0m"
		self.__pRedStr = 				"\033[91m"
		self.__pGreenStr = 				"\033[92m"
		self.__pYellowStr = 			"\033[93m"
		self.__pBlueStr = 				"\033[94m"
		self.__pPurpleStr = 			"\033[95m"
		self.__pLineUpStr = 			"\033[1A"

	###### Connect to the drone and start all procedures
	def startup(self):
		# Check for drone in the network and wake it up
		try:
			socket.socket().connect((self.DroneIP, 21))
			socket.socket().close()
		except:
			self.printRed()
			print "Drone is not online"
			self.printDefault()
			sys.exit(9)

		# Internal variables
		self.__CmdCounter = 3											# as there are two raw commands, send next steps
		self.__calltime = 	0											# to get some time-values to debug

		#send the first four initial-commands to the drone
		self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)	# Open network connection
		self.__sock.setblocking(0)										# Network should not block
		self.__sendrawmsg("\r")											# Wakes up command port
		time.sleep(0.01)
		self.__sendrawmsg("AT*PMODE=1,2\rAT*MISC=2,2,20,2000,3000\r")	# Initialising drone as sniffed from datastream demo-tool to AR.Drone

		##### Initialising timed thread(s) for drone communication
		# Opening NavData- and Video- Processes
		self.__VidPipePath = tempfile.gettempdir()+"/dronevid-"+str(threading.enumerate()[0])[-12:-2]+"-"+str(time.time())[-7:].replace(".","")+".h264"
		self.__net_pipes = []
		self.__NavData_pipe, navdataChild_pipe 		  = multiprocessing.Pipe()
		self.__Video_pipe,   videoChild_pipe          = multiprocessing.Pipe()
		self.__vdecode_pipe, self.__vdecodeChild_pipe = multiprocessing.Pipe()

		self.__NavDataProcess = multiprocessing.Process( target=mainloopND, args=(self.DroneIP,self.NavDataPort,navdataChild_pipe,os.getpid()))
		self.__NavDataProcess.start()
		self.__VideoProcess =   multiprocessing.Process( target=mainloopV, args=(self.DroneIP,self.VideoPort,self.__VidPipePath,videoChild_pipe,os.getpid()))
		self.__VideoProcess.start()
		self.__vDecodeProcess = multiprocessing.Process( target=vDecode, args=(self.__VidPipePath,self.__vdecodeChild_pipe,os.getpid()))
		# There is a third process called "self.__vDecodeProcess" for decoding video, initiated and started around line 880

		# Final settings
		self.useDemoMode(True) 		# This entry is necessary for the drone's firmware, otherwise the NavData contains just header and footer
		self.setConfig("custom:session_id","-all")
		self.getNDpackage(["demo"])

		time.sleep(1)
		#setup Network-thread
		while not self.__receiveDataRunning or not self.__sendConfigRunning or len(self.__ConfigQueue):		# sometimes they would not start why ever, so TK has to double-check
			if not self.__receiveDataRunning:
				self.__threadReceiveData=threading.Thread(target=self.__receiveData)
				self.__threadReceiveData.start()
				time.sleep(0.05)
			if not self.__sendConfigRunning:
				self.__threadSendConfig=threading.Thread(target=self.__sendConfig)
				self.__threadSendConfig.start()
				time.sleep(0.05)
			time.sleep(0.01)

	###### Clean Shutdown
	def shutdown(self):
		if self.__shutdown:	sys.exit()
		self.__shutdown = True
		if self.debug: print "Shutdown..."
		self.land()
		self.thrust(0,0,0,0)
		try:	self.__NavData_pipe.send("die!")
		except:	pass
		self.__Video_pipe.send("uninit")
		t=time.time()
		while	self.__VideoReady and (time.time()-t)<5:	time.sleep(0.1)
		try:	self.__Video_pipe.send("die!")
		except:	pass

		time.sleep(0.5)
		try:	self.__VideoProcess.terminate()
		except:	pass
		try:	self.__vDecodeProcess.terminate()
		except:	pass
		try:	self.__NavDataProcess.terminate()
		except:	pass

		self.__stopnetwork()
		try:	self.__threadSendConfig.join()
		except:	pass
		try:	self.__threadReceiveData.join()
		except:	pass
		self.__keepalive.cancel()
		sys.exit()

##############################################################=-
### Make internal variables to external read-only variables ###=-
##############################################################=-
	@property
	def Version(self):				return self.__Version
	@property
	def startTime(self):			return self.__startTime
	@property
	def speed(self):				return self.__speed
	@property
	def NavData(self):				return self.__NavData
	@property
	def State(self):				return self.__State
	@property
	def NavDataCount(self):			return self.__NavDataCount
	@property
	def NavDataTimeStamp(self):		return self.__NavDataTimeStamp
	@property
	def NavDataDecodingTime(self):	return self.__NavDataDecodingTime
	@property
	def NoNavData(self):			return self.__NoNavData
	@property
	def VideoImage(self):			return self.__VideoImage
	@property
	def VideoImageCount(self):		return self.__VideoImageCount
	@property
	def VideoDecodeTimeStamp(self):	return self.__VideoDecodeTimeStamp
	@property
	def VideoDecodeTime(self):		return self.__VideoDecodeTime
	@property
	def VideoReady(self):			return self.__VideoReady
	@property
	def SaveVideo(self):			return self.__SaveVideo
	@property
	def ConfigData(self):			return self.__ConfigData
	@property
	def ConfigDataCount(self):		return self.__ConfigDataCount
	@property
	def ConfigDataTimeStamp(self):	return self.__ConfigDataTimeStamp
	@property
	def ConfigSending(self):		return self.__ConfigSending
	@property
	def ConfigSessionID(self):		return self.__ConfigSessionID
	@property
	def ConfigUserID(self):			return self.__ConfigUserID
	@property
	def ConfigApplicationID(self):	return self.__ConfigApplicationID

######################=-
### Drone commands ###=-
######################=-
	###### Commands for configuration
	# change some value
	def setConfig(self, name, value):								# e.g. drone.setConfig(control:altitude_max","5000")
		self.__ConfigQueue.append([str(name), str(value), False])	# Note: changes are not immediately and could take some time

	# change some value and send the configuration Identifier (sendConfigIDs) ahead
	def setMConfig(self, name, value):								# Usage like setConfig
		self.__ConfigQueue.append([str(name), str(value), True])	# Note: changes are not immediately and could take some time

	# get actual configuration
	def getConfig(self):											# Stored in "ConfigData"
		self.at("CTRL", [5,0])										# Wow, that is new, was not necessary before
		self.at("CTRL", [4,0])										# Note: Actual configuration data will be received after setting...
		if self.showCommands:		self.__calltime = time.time()	# 	... automatically. An update will take up to 0.015 sec)

	# setting IDs to store Konfigurations for later
	def setConfigSessionID(self, *args):
		try:
			value = float(*args[0])
	 		self.__ConfigSessionID = normalLen8(value)
			self.setConfig("custom:session_id", self.__ConfigSessionID)
		except:		return (self.__ConfigSessionID)

	def setConfigUserID(self, *args):
		try:
			value = float(*args[0])
	 		self.__ConfigUserID = normalLen8(value)
			self.setConfig("custom:profile_id", self.__ConfigUserID)
		except:		return (self.__ConfigUserID)

	def setConfigApplicationID(self, *args):
		try:
			value = float(*args[0])
	 		self.__ConfigApplicationID = normalLen8(value)
			self.setConfig("custom:application_id", self.__ConfigApplicationID)
		except:		return (self.__ConfigApplicationID)

	def setConfigAllID(self):
		self.setConfig("custom:session_id", self.__ConfigSessionID)
		self.setConfig("custom:profile_id", self.__ConfigUserID)
		self.setConfig("custom:application_id", self.__ConfigApplicationID)

	# Reminds the drone which IDs it has to use (important for e.g. switch cameras)
	def sendConfigIDs(self):
		self.at("CONFIG_IDS", [self.__ConfigSessionID,self.__ConfigUserID,self.__ConfigApplicationID])

	###### Calibration
	def trim(self):
		self.at("FTRIM", [])

	def mtrim(self):
		self.at("CALIB", [0])

	def mantrim(self, thetaAngle, phiAngle, yawAngle):		# manual Trim
		if self.valueCorrection:
			try:		thetaAngle = 		float(thetaAngle)
			except:		thetaAngle = 		0.0
			try:		phiAngle = 			float(phiAngle)
			except:		phiAngle = 			0.0
			try:		yawAngle = 			float(yawAngle)
			except:		yawAngle = 			0.0
		self.at("MTRIM", [thetaAngle,phiAngle,yawAngle])	# floats

	def getSelfRotation(self, wait):
		if self.valueCorrection:
			try:		wait = float(wait)
			except:		wait = 1.0
		reftime = time.time()
		oangle = self.__NavData["demo"][2][2]				# detects the self-rotation-speed of the yaw-sensor
		time.sleep(wait)
		self.selfRotation = (self.__NavData["demo"][2][2]-oangle)/(time.time()-reftime)
		return self.selfRotation

	###### Movement
	# Default speed of movement
	def setSpeed(self, *speed):
		try:	self.__speed = self.__checkSpeedValue(*speed)
		except: pass
		return self.__speed

	# Absolute movement in x, y and z-direction and rotation
	def move(self, leftright, backwardforward, downup, turnleftright):	# Absolute movement in x, y and z-direction and rotation
		if self.valueCorrection:
			try:		leftright = 		float(leftright)
			except:		leftright = 		0.0
			try:		backwardforward = 	float(backwardforward)
			except:		backwardforward = 	0.0
			try:		downup = 			float(downup)
			except:		downup = 			0.0
			try:		turnleftright = 	float(turnleftright)
			except:		turnleftright = 	0.0
		if leftright >  1.0:			leftright =	 		 1.0
		if leftright < -1.0:			leftright =			-1.0
		if backwardforward >  1.0:		backwardforward =	 1.0
		if backwardforward < -1.0:		backwardforward =	-1.0
		if downup >  1.0:				downup =			 1.0
		if downup < -1.0:				downup =			-1.0
		if turnleftright >  1.0:		turnleftright =		 1.0
		if turnleftright < -1.0:		turnleftright =		-1.0
		self.at("PCMD", [3 ,leftright, -backwardforward, downup, turnleftright])

	# Relative movement to controller in x, y and z-direction and rotation
	def relMove(self, leftright, backwardforward, downup, turnleftright, eastwest, northturnawayaccuracy):
		if self.valueCorrection:
			try:		leftright = 		float(leftright)
			except:		leftright = 		0.0
			try:		backwardforward = 	float(backwardforward)
			except:		backwardforward = 	0.0
			try:		downup = 			float(downup)
			except:		downup = 			0.0
			try:		turnleftright = 	float(turnleftright)
			except:		turnleftright = 	0.0
		if leftright >  1.0:			leftright = 		 1.0
		if leftright < -1.0:			leftright =			-1.0
		if backwardforward >  1.0:		backwardforward =	 1.0
		if backwardforward < -1.0:		backwardforward =	-1.0
		if downup >  1.0:				downup =			 1.0
		if downup < -1.0:				downup =			-1.0
		if turnleftright >  1.0:		turnleftright =		 1.0
		if turnleftright < -1.0:		turnleftright =		-1.0
		self.at("PCMD_MAG", [1 ,leftright, -backwardforward, downup, turnleftright, eastwest, northturnawayaccuracy])

	# Stop moving
	def hover(self):
		self.at("PCMD", [0,0.0,0.0,0.0,0.0])
	def stop(self):	# Hammertime !
		self.hover()

	# Basic movements
	def moveLeft(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(-self.__checkSpeedValue(speed),0.0,0.0,0.0)

	def moveRight(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move( self.__checkSpeedValue(speed),0.0,0.0,0.0)

	def moveForward(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(0.0, self.__checkSpeedValue(speed),0.0,0.0)

	def moveBackward(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(0.0,-self.__checkSpeedValue(speed),0.0,0.0)

	def moveUp(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(0.0,0.0, self.__checkSpeedValue(speed),0.0)

	def moveDown(self,args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(0.0,0.0,-self.__checkSpeedValue(speed),0.0)

	def turnLeft(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(0.0,0.0,0.0,-self.__checkSpeedValue(speed))

	def turnRight(self,*args):
		try:	speed=args[0]
		except:	speed=self.__speed
		self.move(0.0,0.0,0.0, self.__checkSpeedValue(speed))

	# Lets the drone rotate defined angle
	# BUG: does not work with 180deg. turns
	# ToDo: Should be able to stop in case of failures
	def turnAngle(self,ndir,speed,*args):
		opos = 		self.__NavData["demo"][2][2]			# get the source/current (original) angle
		npos = 		opos+ndir								# calculate the destination (new) angle
		minaxis = 	opos									# to make sure, that the jump from -180 to 180 will...
		maxaxis = 	opos									# ...be correctly handled
		speed = 	self.__checkSpeedValue(speed)
		ospeed = 	speed									# stores the given speed-value
		reftime = 	time.time()
		accurateness = 0
		try:	accurateness = args[0]
		except:	pass
		if accurateness<=0:
			accurateness = 0.005							# Destination angle can differ +/- this value (not demo-mode)
			if self.__State[10]:	accurateness = 0.1		# Destination angle can differ +/- this value in demo-mode
		stop = False
		while not stop:
			ndc = self.__NavDataCount						# wait for the next NavData-package
			while ndc == self.__NavDataCount:		time.sleep(0.001)
			kalib = (time.time()-reftime)*self.selfRotation	# trys to recalibrate, causing moving sensor-values around 0.0185 deg/sec
			cpos = self.__NavData["demo"][2][2]				# get the current angle
			if minaxis > cpos:			minaxis = cpos		# set the minimal seen angle
			if maxaxis < cpos:			maxaxis = cpos		# set the maximal seen angle
			if cpos-minaxis >= 180:		cpos = cpos-360		# correct the angle-value if necessary...
			elif maxaxis-cpos >= 180:	cpos = cpos+360		# ...for an easier calculation
			speed = abs(cpos-npos+kalib) / 10.0				# the closer to the destination the slower the drone turns
			if speed > ospeed:			speed = ospeed		# do not turn faster than recommended
			if speed < 0.05:			speed = 0.05		# too slow turns causes complications with calibration
			self.__speed = speed
			if cpos > (npos+kalib):		self.turnLeft()		# turn left, if destination angle is lower
			else:						self.turnRight()	# turn right if destination angle is higher
			if cpos < (npos+kalib+accurateness) and cpos > (npos+kalib-accurateness):# if angle is reached...
				self.stop()									# ...stop turning
				time.sleep(0.01)
				stop = True
		return(True)

	def takeoff(self):
		self.at("REF", [290718208]) #290718208=10001010101000000001000000000

	def land(self):
		self.at("REF", [290717696]) #290717696=10001010101000000000000000000

	###### NavData commands
	# Switches to Demo- or Full-NavData-mode
	def useDemoMode(self,value):
		if value:	self.setConfig("general:navdata_demo", "TRUE")
		else:		self.setConfig("general:navdata_demo", "FALSE")

	def useMDemoMode(self,value):
		if value:	self.setMConfig("general:navdata_demo", "TRUE")
		else:		self.setMConfig("general:navdata_demo", "FALSE")

	def getNDpackage(self,packets):
		self.__NavData_pipe.send(("send",packets))

	def addNDpackage(self,packets):
		self.__NavData_pipe.send(("add",packets))

	def delNDpackage(self,packets):
		self.__NavData_pipe.send(("block",packets))

	def reconnectNavData(self):
		self.__NavData_pipe.send("reconnect")

	###### Video & Marker commands
	# This makes the drone fly around and follow 2D tags which the camera is able to detect.
	def aflight(self, flag):
	    self.at("AFLIGHT", [flag])	#Integer: 1: start flight, 0: stop flight

	def slowVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.__Video_pipe.send("slowVideo")
		else:	self.__Video_pipe.send("fastVideo")

	def midVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.__Video_pipe.send("midVideo")
		else:	self.__Video_pipe.send("fastVideo")

	def fastVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.__Video_pipe.send("fastVideo")
		else:	self.__Video_pipe.send("slowVideo")

	def saveVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.__Video_pipe.send("saveVideo")
		else:	self.__Video_pipe.send("unsaveVideo")

	def startVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.__Video_pipe.send("init")
		else:	self.stopVideo()

	def stopVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.__Video_pipe.send("uninit")
		else:	self.startVideo()

	def showVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:
				self.__Video_pipe.send("init")
				self.__Video_pipe.send("show")
		else:	self.hideVideo()

	def hideVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:
				self.__Video_pipe.send("init")
				self.__Video_pipe.send("hide")
		else:	self.showVideo()

	# Selects which video stream to send on the video UDP port.
	def hdVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.setMConfig("video:video_codec","131")
		else:	self.setMConfig("video:video_codec","129")

	def sdVideo(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.setMConfig("video:video_codec","129")
		else:	self.setMConfig("video:video_codec","131")

	def mp4Video(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.setMConfig("video:video_codec","128")
		else:	self.setMConfig("video:video_codec","129")

	# Selects which video-framerate (in frames per second) to send on the video UDP port.
	def videoFPS(self, fps):
		try:
			int(fps)
			if fps>60:	fps = 60
			elif fps<1:	fps = 1
			self.setMConfig("video:codec_fps",fps)
		except:		pass

	# Selects which video-bitrate (in kilobit per second) to send on the video UDP port.
	def videoBitrate(self, bitrate):
		try:
			int(bitrate)
			if bitrate > 20000:	bitrate = 20000
			if bitrate < 250:	bitrate = 250
			self.setMConfig("video:bitrate",bitrate)
		except:		pass

	# Selects which video stream to send on the video UDP port.
	def frontCam(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.setMConfig("video:video_channel","0")
		else:	self.setMConfig("video:video_channel","1")

	def groundCam(self, *args):
		try:	do = args[0]
		except:	do = True
		if do:	self.setMConfig("video:video_channel","1")
		else:	self.setMConfig("video:video_channel","0")

### Misc commands
	def reset(self):
		if self.NavDataCount>0 and self.State[31]==1:
			self.at("REF", [290717952]) #290717952=10001010101000000000100000000

	def thrust(self, fl, fr, rl, rr):	# Controls engines directly, overriding control loops.
		fl *= 2
		if fl > 64000:	fl = 64000
		elif fl < 0:	fl = 0
		fr *= 2
		if fr > 64000:	fr = 64000
		elif fr < 0:	fr = 0
		rl *= 2
		if rl > 64000:	rl = 64000
		elif rl < 0:	rl = 0
		rr *= 2
		if rr > 64000:	rr = 64000
		elif rr < 0:	rr = 0


		self.at("PWM", [int(fl), int(fr), int(rr), int(rl)])
		# Seems that integer-values could be between 0 (stop) to 511 (full); more than 511 seem to have no effect.
		# Beware: if using too high values (e.g. floats (>64k ?)), there will be side-effects like restarting other motors, etc.
		# Drone will shut down, if its flight-angle is more than set.

#   Control the drone's LED.
	def led(self, animation, frequency, duration):
		if animation < 21 and frequency > 0 and duration >= 0:
			self.at("LED", [animation, float(frequency), duration])

#   Makes the drone execute a predefined movement (animation).
	def anim(self, animation, duration):
		if animation < 20 and duration >= 0:
			self.at("ANIM", [animation, duration])


#########################=-
### Low-level Commands ###=-
#########################=-

	# Upgrading the basic drone commands to low-level drone commands:vid
	# Adding command-number, checking the values, convert 32-bit float to 32-bit integer and put it in quotes
	def at(self, command, params):
		self.__lock.acquire()
		paramLn = ""
		if params:
			for p in params:
				if type(p) 	 == int:	paramLn += ","+str(p)
				elif type(p) == float:	paramLn += ","+str(struct.unpack("i", struct.pack("f", p))[0])
				elif type(p) == str:	paramLn += ",\""+p+"\""
		msg = "AT*"+command+"="+str(self.__CmdCounter)+paramLn+"\r"
		self.__CmdCounter += 1
		self.__sendrawmsg(msg)
		self.__lock.release()


	# Sending the low-level drone-readable commands to the drone...better do not use
	def __sendrawmsg(self, msg):
		try:		self.__keepalive.cancel()
		except:		pass
		if self.showCommands:
			if msg.count("COMWDG") < 1:	print msg
		self.__sock.sendto(msg, (self.DroneIP, self.CmdPort))
		self.__keepalive = threading.Timer(0.1, self.__heartbeat)
		self.__keepalive.start()


#############################=-
###  Convenient Commands  ###=-
#############################=-
#    Just add water
	# Checks the battery-status
	def getBattery(self):
		batStatus =	"OK"
		batValue =	0
		if self.__State[15] == 1:	batStatus = "empty"
		try:	batValue = self.__NavData['demo'][1]
		except:	batValue = -1
		return (batValue,batStatus)	# Percent & status ("OK", "empty")

	# Calculates the minor difference between two angles as the drone gives values from -180 to 180...
	# ...so e.g. 170 and -160 are +30 difference and drone will turn to the correct direction
	def angleDiff(self, base, value):
		adiff = ((base+180)-(value+180)) %360
		if adiff>180: adiff-=360
		return adiff

	# Grabs the pressed key (not yet for Windows)
	# ToDo: Reprogram for Windows
	def getKey(self):
		key =	""
		fd =	sys.stdin.fileno()
		if os.name == 'posix':
			oldterm =		termios.tcgetattr(fd)
			newattr =		termios.tcgetattr(fd)
			newattr[3] =	newattr[3] & ~termios.ICANON & ~termios.ECHO
			termios.tcsetattr(fd, termios.TCSANOW, newattr)
			oldflags = 		fcntl.fcntl(fd, fcntl.F_GETFL)
			fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
			try:
				try:	key = sys.stdin.read(1)
				except IOError: pass
			finally:
				termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
				fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
		if os.name == 'nt':
			if msvcrt.kbhit():	key = msvcrt.getch()
		key += self.__vKey
#		self.__vKey = ""
		return key

	# Drone hops like an excited dog
	def doggyHop(self):
		ospeed = self.__speed
		self.__speed = 1
		for i in range (0,4,1):
			self.moveUp()
			time.sleep(0.20)
			self.moveDown()
			time.sleep(0.20)
		self.hover()
		self.__speed = ospeed

	# Drone wags like a happy dog
	def doggyWag(self):
		ospeed = self.__speed
		self.__speed = 1
		for i in range (0,4,1):
			self.moveLeft()
			time.sleep(0.25)
			self.moveRight()
			time.sleep(0.25)
		self.hover()
		self.__speed = ospeed

	# Drone nods
	def doggyNod(self):
		ospeed = self.__speed
		self.__speed = 1
		for i in range (0,4,1):
			self.moveForward()
			time.sleep(0.25)
			self.moveBackward()
			time.sleep(0.25)
		self.hover()
		self.__speed = ospeed

	def printDefault(self, *args):
		if os.name == 'posix':
			print self.__pDefaultStr,
			try:
				if len(*args) > 0:
					for i in args:	print i,
					print self.__pDefaultStr
			except: pass

	def printRed(self, *args):
		if os.name == 'posix':
			print self.__pRedStr,
			try:
				if len(*args) > 0:
					for i in args:	print i,
					print self.__pDefaultStr
			except: pass

	def printGreen(self, *args):
		if os.name == 'posix':
			print self.__pGreenStr,
			try:
				if len(*args) > 0:
					for i in args:	print i,
					print self.__pDefaultStr
			except: pass

	def printYellow(self, *args):
		if os.name == 'posix':
			print self.__pYellowStr,
			try:
				if len(*args) > 0:
					for i in args:	print i,
					print self.__pDefaultStr
			except: pass

	def printBlue(self, *args):
		if os.name == 'posix':
			print self.__pBlueStr,
			try:
				if len(*args) > 0:
					for i in args:	print i,
					print self.__pDefaultStr
			except: pass

	def printPurple(self, *args):
		if os.name == 'posix':
			print self.__pPurpleStr,
			try:
				if len(*args) > 0:
					for i in args:	print i,
					print self.__pDefaultStr
			except: pass

	def printLineUp(self):
		if os.name == 'posix':	print self.__pLineUpStr,


##################################=-
### Threads & Thread-Sidekicks ###=-
##################################=-
# Idea: the network thread listens to the given network-stream and communication-pipes of other processes, such as for video or navdata-decoding.
# 		In case the connection to the drone is cut off for more than 2 seconds (so no keep-alive-command has been sent) the network
#		needs to reconnect. In order to do so the (private) function "__netrecon" starts after 0.1 seconds of no incoming navdata-datapacket to
#		reconnect all given network-sockets.
	def __heartbeat(self):
	# If the drone does not get a command, it will mutter after 50ms (CTRL watchdog / state[28] will set to 1)
	# and panic after 2 seconds and abort data-communication on port 5554 (then you have to initialize the network again).
	# Heartbeat will reset the watchdog and, by the way, the ACK_BIT (state[6], to accept any other AT*CONFIG command)
	# If mainthread isn't alive anymore (because program crashed or whatever), heartbeat will initiate the shutdown.
		if str(threading.enumerate()).count("MainThread, stopped") or str(threading.enumerate()).count("MainThread")==0:	self.shutdown()
		else:	self.at("COMWDG",[])

	# CheckAndReact is periodically called by the receiveData-Thread to check for mainly for critical status-error(s) and
	# changed debug-modes.
	def __checkAndReact(self, debug, showCommands):
		# Automatic process-commands, used for syncing debugging-bits to child-processes
		if debug != self.debug:
			debug = self.debug
			if debug:
				self.__NavData_pipe.send("debug")
				self.__Video_pipe.send("debug")
			else:
				self.__NavData_pipe.send("undebug")
				self.__Video_pipe.send("undebug")
		if showCommands != self.showCommands:
			showCommands = self.showCommands
			if showCommands:
				self.__NavData_pipe.send("showCommands")
				self.__Video_pipe.send("showCommands")
			else:
				self.__NavData_pipe.send("hideCommands")
				self.__Video_pipe.send("hideCommands")
		# Communication problem, shutting down
		if self.stopOnComLoss and self.__State[30]:
			self.shutdown()
			sys.exit()
		return (debug,showCommands)

	# Thread for sending the configuration. It is asynchronous but save.
	# The configuration-requests are in a queue, the first entry is sent. NavData will contain a "Control command ACK" status-bit,...
	# ...that configuration is ready to be set. This will be confirmed and the procedure waits until this bit is 0 again; then the next entry will be processed.
	# In savemode, there is a check whether the configuration has been changed correctly by requesting the current/latest configuration and double-checking this value.
	def __sendConfig(self):
		sleeptime, getconfigtag, self.__sendConfigRunning = 0.001, False, True
		while not self.__networksuicide:
			if len(self.__ConfigQueue):										# If there is something in the queue...
				if self.__ConfigQueue[0][-1]:	self.sendConfigIDs()		# ...check for multiuserconfig-request (and send it)
				self.__ConfigSending = True									# Set tag, to show sending is in process
				qlen = len(self.__ConfigQueue)
				if qlen > 1:												# Testing for double entries, preventing a ping-pong in save-mode
					i = 1
					while True:
						if i >= qlen:	break
						if self.__ConfigQueue[0][0].lower() == self.__ConfigQueue[i][0].lower():
							self.__ConfigQueue.remove(self.__ConfigQueue[0])# Delete double entries
							qlen = len(self.__ConfigQueue)
						else: 			i+=1
				self.at("CONFIG",self.__ConfigQueue[0][:-1])				# Send the first entry in queue
				getconfigtag, configconfirmed, configreconfirmed = False, False, False
				while not configconfirmed and not self.__networksuicide:	# Wait for confirmation-bit from drone...
					if self.__State[6] and not configreconfirmed and not self.__networksuicide:
						self.at("CTRL",[5,0])								# ...and send reset the confirmation-bit
						configreconfirmed = True
					if not self.__State[6] and configreconfirmed and not self.__networksuicide:
						configconfirmed = True								# Wait for the reset of the confirmation-bit
					time.sleep(sleeptime)
				# It seems that the drone stores configurations not always correctly; therfore, here is a save-mode:
				if self.sendConfigSaveMode and not self.__networksuicide:
					lastConfigDataCount = self.__ConfigDataCount			# Wait for the next configuration-list
					self.getConfig()
					while lastConfigDataCount == self.__ConfigDataCount and not self.__networksuicide:	time.sleep(sleeptime)
				# New & Optimized
					for i in range (0,len(self.__ConfigData),1):
						if self.__ConfigData[i][0].find(self.__ConfigQueue[0][0]) > -1:
							if self.__ConfigData[i][1] != self.__ConfigQueue[0][1]:
								if self.debug:
									print "   Configuration missmatched, resending !"
									print "   "+self.__ConfigData[i][0]+" should be \""+self.__ConfigQueue[0][1]+"\" is \""+self.__ConfigData[i][1]+"\""
									self.__ConfigQueue.append(self.__ConfigQueue[0])		# If value is not correctly set, requeue !
				self.__ConfigQueue.remove(self.__ConfigQueue[0])							# Configuration has been (correctly) set, delete request from queue and go on
				if self.__networksuicide:	self.__ConfigQueue=[]
			if not len(self.__ConfigQueue):
				if not getconfigtag:
					self.getConfig()
					getconfigtag = True
					self.__ConfigSending = False
				else:	time.sleep(sleeptime)
		if self.debug:	print "sendConfig-Tread :   committed suicide"

	def __receiveData(self):
		self.__net_pipes=[]
		self.__net_pipes.append(self.__NavData_pipe)
		self.__net_pipes.append(self.__Video_pipe)
		self.__Config_pipe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	#TCP
		self.__Config_pipe.setblocking(0)
		self.__Config_pipe.connect_ex((self.DroneIP, self.CTLPort))
		self.__net_pipes.append(self.__Config_pipe)
		VideoIsDead, configdata, cfgdata, cmd = False, [], "", ""
		self.__vDecodeRunning, debug, showCommands, self.__receiveDataRunning = False, False, False, True

		while not self.__networksuicide:
			in_pipe, dummy1, dummy2 = select.select(self.__net_pipes, [], [], 0.1)	# When something is in a pipe...
			for ip in in_pipe:														# ...go and get it
				if ip == self.__NavData_pipe:		### Receiving sensor-values from NavData-process
					self.__NavData, self.__State, self.__NavDataCount, self.__NavDataTimeStamp, self.__NavDataDecodingTime, self.__NoNavData = self.__NavData_pipe.recv()
				if ip == self.__vdecode_pipe:		### Receiving imagedata and feedback from videodecode-process
					cmd, VideoImageCount, VideoImage, VideoDecodeTime = self.__vdecode_pipe.recv()	# Imagedata
					if self.showCommands and cmd!="Image" :	print "** vDec -> Com :",cmd
					if cmd == "suicided":				self.__Video_pipe.send("vd died")		# videodecode-process died
					if cmd == "foundCodec":				self.__Video_pipe.send("foundCodec")	# the codec of the videostream has been found, do not flood anymore
					if cmd == "VideoUp":				self.__VideoReady = True				# Imagedata is available
					if cmd == "keypressed":				self.__vKey = VideoImage				# Pressed key on window
					if cmd == "reset":					self.__Video_pipe.send(cmd)				# proxy to videodecode-process
					if cmd == "Image":															# Imagedata !
						self.__VideoImageCount =		VideoImageCount
						self.__VideoImage =				VideoImage
						self.__VideoDecodeTime =		VideoDecodeTime
						self.__VideoDecodeTimeStamp =	time.time()-self.__startTime
				if ip == self.__Video_pipe:		### Receiving feedback from videostream-process
					cmd = self.__Video_pipe.recv()
					if self.showCommands and cmd != "":	print "** Vid -> Com : ",cmd
					if cmd == "vDecProc":											# videodecode-process should start
						if not self.__vDecodeRunning:
							self.__vDecodeProcess = multiprocessing.Process( target=vDecode, args=(self.__VidPipePath,self.__vdecodeChild_pipe,os.getpid()))
							#self.__vDecodeProcess.start()
							self.__net_pipes.append(self.__vdecode_pipe)
							self.__vDecodeRunning = True

						self.__Video_pipe.send("vDecProcON")
#					else:						self.__vdecode_pipe.send(cmd)		# If / elif / else is somehow not working here...whyever
					if cmd == "VideoDown":		self.__VideoReady=False				# videodecode-process stopped
					if cmd == "saveVideo":		self.__SaveVideo=True				# no preprocessing of the video
					if cmd == "unsaveVideo":	self.__SaveVideo=False				# preprocessing activated again
					if cmd == "debug":			self.__vdecode_pipe.send(cmd)		# proxy to videodecode-process
					if cmd == "showCommands":	self.__vdecode_pipe.send(cmd)		# proxy to videodecode-process
					if cmd == "hideCommands":	self.__vdecode_pipe.send(cmd)		# proxy to videodecode-process
					if cmd == "show":			self.__vdecode_pipe.send(cmd)		# proxy to videodecode-process
					if cmd == "hide":			self.__vdecode_pipe.send(cmd)		# proxy to videodecode-process
					if cmd == "vDecProcKill":
						self.__vdecode_pipe.send("die!")	# videodecode-process should switch off
						vDecodeRunning = False
				if ip==self.__Config_pipe and not self.__networksuicide:	### Receiving drone-configuration
					try:
						if self.__networksuicide:	  break								# Does not stop sometimes, so the loop will be forced to stop
						cfgdata = cfgdata+self.__Config_pipe.recv(65535)				# Data comes in two or three packages
						if cfgdata.count("\x00"):										# Last byte of sent config-file, everything was received
							if self.__networksuicide:	break
							configdata = (cfgdata.split("\n"))							# Split the huge package into a configuration-list
							for i in range(0, len(configdata), 1):
								configdata[i] = configdata[i].split(" = ")				# Split the single configuration-lines into configuration and value
							self.__ConfigData = configdata[:-1]   						# Last value is "\x00"
							self.__ConfigDataTimeStamp = time.time()-self.__startTime	# Set a timestamp for a better coordination
							self.__ConfigDataCount+=1									# Alters the count of received Configdata for a better coordination
							configdata, cfgdata = [], ""
							if self.showCommands:	print "Got "+str(len(self.__ConfigData))+" Configdata "+str(time.time()-self.__calltime)
							self.__calltime=0
					except IOError:	pass
				debug, showCommands = self.__checkAndReact(debug, showCommands)		# Check for errors and things to change
		if self.debug:	print "receiveData-Thread : committed suicide"

	def __stopnetwork(self):
		self.__networksuicide = True

#############################=-
### Compatibility Commands ###=-
#############################=-
# While programming this API I changed some command-names
# This section converts the old commands into the new ones
	def pwm(self, fl, fr, rl, rr):		# Controls engines directly, overriding control loops.
		if fl > 64000:	fl = 64000
		if fr > 64000:	fr = 64000
		if rl > 64000:	rl = 64000
		if rr > 64000:	rr = 64000
		self.at("PWM", [int(fl), int(fr), int(rr), int(rl)])

	def groundVideo(self, *args):	self.groundCam(*args)
	def frontVideo(self, *args):	self.frontCam(*args)

###############################################################################
### Internal Subfunctions
###############################################################################
	def	__checkSpeedValue(self,value):
		try:
			speed = float(value)
			if self.valueCorrection:
				speed = max(-1.0,speed)
				speed = min( 1.0,speed)
		except:	speed = self.__speed
		return speed

# Checks the inputs for the right length
def normalLen8(value):
	value, zero =	str(value), "00000000"
	vlen =			min(len(value),8)
	normal = 		zero[0:8-vlen] + value[0:8]
	return normal[0:8].lower()

##################################################################################################
###### Receive and Decode Video																######
##################################################################################################
# If the ps_drone-process has crashed, recognize it and kill yourself
def watchdogV(parentPID, ownPID):
	global commitsuicideV
	while not commitsuicideV:
		time.sleep(1)
		try : 		os.getpgid(parentPID)
		except:
			try:	subprocess.Popen(["kill",str(os.getpid())],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
			except:	pass

# Thread to capture, decode and display the video-stream
def vCapture(VidPipePath, parent_pipe):
	import cv2
	global vCruns, commitsuicideV, showVid, lockV, debugV

#	cv2.startWindowThread()
	show = 		False
	hide =		True
	vCruns =	True
	t = 		time.time()
	parent_pipe.send(("VideoUp",0,0,0))
	capture = 	cv2.VideoCapture(VidPipePath)
	ImgCount =	0
	if debugV:	print "CAPTURE: "+str(time.time()-t)
	time.sleep(0.1)
	parent_pipe.send(("foundCodec",0,0,0))
	declag =	time.time()
	count =		-3
	imageXsize = 	0
	imageYsize = 	0
	windowName = 	"PS-Drone"
	codecOK = 		False
	lastKey =		""
	cc=0

	while not commitsuicideV:
		decTimeRev = 		time.time()
		receiveWatchdog = threading.Timer(2.0, VideoReceiveWatchdog, [parent_pipe,"vCapture", debugV])	# Resets video if something hangs
		receiveWatchdog.start()
		success, image = 	capture.read()
		cc+=1
		receiveWatchdog.cancel()
		decTime =			decTimeRev-time.time()
		tlag =				time.time()-declag

		if not codecOK and success:
			if image.shape[:2]==(360,640) or image.shape[:2]==(368,640) or image.shape[:2]==(720,1280) or image.shape[:2]==(1080,1920):
				codecOK = True
				if debugV:	print "Codec seems OK"
			else:
				if debugV:	print "Codec failure"
				parent_pipe.send(("reset",0,0,0))
				commitsuicideV = True
		if codecOK:
			if not (imageXsize == image.shape[1]) or not (imageYsize == image.shape[0]):
				cv2.destroyAllWindows()
				imageYsize, imageXsize = image.shape[:2]
				windowName = "PS-Drone - "+str(imageXsize)+"x"+str(imageYsize)
			if success:
				if tlag > 0.02:	count+=1
				if count > 0:
					ImgCount+=1
					if not show and not hide:
						cv2.destroyAllWindows()
						hide = True
					if show:
						cv2.imshow(windowName, image)
						key=cv2.waitKey(1)
						if key>-1:	parent_pipe.send(("keypressed",0,chr(key%256),0))
					parent_pipe.send(("Image",ImgCount,image,decTime))
			else:	time.sleep(0.01)
			declag = time.time()

			if showVid:
				if not show:
					show=True
					cv2.destroyAllWindows()
			else:
				if show:
					show=False
					cv2.destroyAllWindows()
	vCruns = False
	cv2.destroyAllWindows()
	capture.release()
	if debugV:	print "vCapture-Thread :    committed suicide"

### Process to decode the videostream in the FIFO-Pipe, stored there from main-loop.
# Storing and decoding must not be processed in the same process, thats why decoding is external.
# vDecode controls the vCapture-thread which captures and decodes finally the videostream.
def vDecode(VidPipePath, parent_pipe, parentPID):
	global vCruns, commitsuicideV, showVid, lockV, debugV
	showCommands = 		False
	Thread_vCapture =	threading.Thread(target=vCapture, args=(VidPipePath,parent_pipe))
	Thread_vCapture.start()
	Thread_watchdogV =	threading.Thread(target=watchdogV, args=[parentPID,os.getpid()])
	Thread_watchdogV.start()

	while not commitsuicideV:
		in_pipe, out_pipe, dummy2 = select.select([parent_pipe], [], [], 0.1)		# When something is in a pipe...
		cmd = parent_pipe.recv()
		if showCommands:	print "** Com -> vDec : ",cmd
		if cmd == "die!":			commitsuicideV = True
		elif cmd == "reset":		commitsuicideV = True
		elif cmd == "show":			showVid = 		True
		elif cmd == "hide":			showVid =		False
		elif cmd == "debug":
			debugV = True
			print "vDecode-Process :    running"
			if vCruns:	print "vCapture-Thread :    running"
		elif cmd == "undebug":		debugV =		False
		elif cmd == "showCommands":	showCommands =	True
		elif cmd == "hideCommands":	showCommands =	False
	Thread_vCapture.join()
	parent_pipe.send(("suicided",0,0,0))
	time.sleep(0.1)
	if debugV:	print "vDecode-Process :    committed suicide"

#####################################################
def VideoReceiveWatchdog(parent_pipe,name, debugV):
	if debugV:	print "WHATCHDOG reset von",name
	parent_pipe.send(("reset",0,0,0))

def mainloopV(DroneIP, VideoPort, VidPipePath, parent_pipe, parentPID):
	inited, preinited, suicide, debugV, showCommands, slowVideo = False, False, 0, False, False, False
	rawVideoFrame, VidStreamSnippet, VidStreamSnippetAvalible, iFrame, FrameCount = "", "", False, False, 0
	saveVideo, unsureMode, searchCodecTime, frameRepeat, burstFrameCount = False, True, 0, 1, 0
	reset, resetCount, commitsuicideV, foundCodec = False, 0, False, False

	vstream_pipe, pipes = None, [parent_pipe]
	vdecode_pipe, vdecode_childpipe = multiprocessing.Pipe()
	pipes.append(vdecode_pipe)
	Thread_watchdogV = threading.Thread(target=watchdogV, args=[parentPID,os.getpid()])
	Thread_watchdogV.start()

	while not commitsuicideV:
		in_pipe, out_pipe, dummy2 = select.select(pipes, [], [], 0.1)		# When something is in a pipe...
		for ip in in_pipe:
			if ip == parent_pipe:
				cmd = parent_pipe.recv()
				if showCommands:			print "** Com -> Vid : ",cmd
				if cmd == "die!":
					if inited:
						suicide = True
						parent_pipe.send("vDecProcKill")
						dummy = 0
					else:	commitsuicideV = True
				elif cmd == "foundCodec":	foundCodec = True
				elif cmd == "reset" and not reset:# and resetCount<3:
					inited, preinited, foundCodec = 	False, False, False
					rawVideoFrame, VidStreamSnippet = 	"", ""
					VidStreamSnippetAvalible = 			False
					iFrame, FrameCount, reset =			False, 0, True
					unsureMode, searchCodecTime = 		True, 0
					burstFrameCount	=					0
					resetCount += 1
					parent_pipe.send("vDecProcKill")
				elif cmd == "slowVideo":
					slowVideo = True
					frameRepeat = 1
				elif cmd == "midVideo":
					slowVideo = True
					frameRepeat = 4
				elif cmd == "fastVideo":
					slowVideo = False
					frameRepeat = 1
				elif cmd == "saveVideo":
					saveVideo = True
					parent_pipe.send("saveVideo")
				elif cmd == "unsaveVideo":
					saveVideo = False
					parent_pipe.send("unsaveVideo")
				elif cmd == "showCommands":
					showCommands = True
					parent_pipe.send("showCommands")
				elif cmd == "hideCommands":
					showCommands = False
					parent_pipe.send("hideCommands")
				elif cmd == "debug":
					debugV = True
					print "Video-Process :      running"
					parent_pipe.send("debug")
				elif cmd == "undebug":
					debugV = False
					parent_pipe.send("undebug")
				elif cmd == "init" and not inited and not preinited:
					preinited = True
					try:	os.mkfifo(VidPipePath)
					except:	pass
					parent_pipe.send("vDecProc")
				elif cmd == "vDecProcON":
					rawVideoFrame = ""
					VidStreamSnippet = ""
					iFrame = False
					FrameCount = 0
					foundCodec = False
					searchCodecTime = 0
					if not vstream_pipe:
						vstream_pipe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
						vstream_pipe.setblocking(0)
						vstream_pipe.connect_ex((DroneIP,VideoPort))
						pipes.append(vstream_pipe)
					write2pipe = open(VidPipePath,"w+")
					suicide = False
					inited = True
					preinited = False
					unsureMode = True
				elif cmd == "uninit" and inited:
					parent_pipe.send("vDecProcKill")
				elif cmd == "vd died":
					if inited and not reset:
						pipes.remove(vstream_pipe)
						vstream_pipe.shutdown(socket.SHUT_RDWR)
						vstream_pipe.close()
						write2pipe.close()
						inited = False
						if suicide:		commitsuicideV = True
						parent_pipe.send("VideoDown")
					try:			os.remove(VidPipePath)
					except:			pass
					if not inited and reset:
						try:	os.mkfifo(VidPipePath)
						except:	pass
						parent_pipe.send("VideoDown")
						parent_pipe.send("vDecProc")
						parent_pipe.send("debug")
						reset = 		  False
						burstFrameCount	= 0
				else:
					parent_pipe.send(cmd)

			### Grabs the Videostream and store it in a fifo-pipe for decoding.
			# The decoder has to guess the videostream-format which takes around 266 video-frames.
			#    So the stream is preprocessed, I-Frames will cut out while initiation and a flood of copies
			#	 will be send to the decoder, till the proper decoder for the videostream is found.
			# In case of a slow or midspeed-video, only a single or a few copied I-frames are sent to the decoder.
			if ip == vstream_pipe:
				receiveWatchdog = threading.Timer(2.0, VideoReceiveWatchdog, [parent_pipe,"Video Mainloop", debugV,])	# Resets video if something hangs
				receiveWatchdog.start()
				videoPackage = vstream_pipe.recv(65535)
				receiveWatchdog.cancel()
				if len(videoPackage) == 0:		commitsuicideV = True
				else:
					if inited and not reset:
						if unsureMode:			### An MPEG4-Stream is not confirmed, fallback to savemode ?
							if not searchCodecTime and not len(VidStreamSnippet):	# Video is freshly initiated
								searchCodecTime = time.time()
							if (time.time()-searchCodecTime) < 0.15:				# Collecting VidStreamSnipped for later use
								VidStreamSnippet+=videoPackage
							if (time.time()-searchCodecTime) > 2.0:					# Waited too long for an MPEG4 stream confirmation...
								saveVideo = True									# ... fall back to savemode
								parent_pipe.send("saveVideo")						# Inform the main process
								unsureMode = False
								foundCodec = True									# switch off codec guess speed-up
						if not saveVideo:
	#						if len(videoPackage) == 0:		commitsuicideV = True
	#						else:
								if videoPackage[31:40].find("\x00\x00\x00")>3:		# Found a new MPEG4-Frame
									FrameCount+=1
									### Processing the last frame
									if iFrame:										# If the last frame was an I-frame
										VidStreamSnippet = rawVideoFrame			# ... save it as VideoStreamSnippet for later use
										if foundCodec:								# OpenCV guessed the used Codec
											if slowVideo:							# Send just the iFrame (openCV stores about 5 in its queue),
												for i in range(0,frameRepeat,1):	#  ... so repeat for less delay in midVideo()-mode
													write2pipe.write(VidStreamSnippet)
										iFrame = False
									else:	pass
									if not slowVideo:								# For all last Frames
										if foundCodec:
											try:	write2pipe.write(rawVideoFrame)
											except: pass
									if not foundCodec:								# Flood the pipe with the last iFrames, so that openCV can guess the codec faster
										for i in range(0,5):
											try:
												write2pipe.write(rawVideoFrame)
												burstFrameCount+=1
											except: pass
									### Processing new Frames
									if ord(videoPackage[30]) == 1:					#### Found an I-Frame
										rawVideoFrame = ""							# Delete the data previous to the first iFrame
										unsureMode,iFrame = False, True
									elif ord(videoPackage[30]) == 3:				#### Found a P-Frame
										unsureMode = False
									else:											#### Found an odd h264-frametype
										if debugV:
											print "*** Odd h264 Frametype: ",FrameCount,
											for i in range(31,43,1):	print ord(videoPackage[i]),
											print " - ",videoPackage[31:40].find("\x00\x00\x00"),ord(videoPackage[30])
									rawVideoFrame = ""
								### Collecting data for the next frame from stream
								rawVideoFrame+=videoPackage
						else: #(saveVideo-Mode)
							if foundCodec:	write2pipe.write(videoPackage)
							else:
								for i in range(0,2):
									write2pipe.write(VidStreamSnippet)
									burstFrameCount+=1
						if not foundCodec and burstFrameCount>350:
							parent_pipe.send(("reset",0,0,0))
							burstFrameCount=0
							if debugV: print "To many pictures send while guessing the codec. Resetting."

	try:
		vstream_pipe.shutdown(socket.SHUT_RDWR)
		vstream_pipe.close()
	except:	pass
	try:	write2pipe.close()
	except:	pass
	try:	vstream_pipe.close()
	except:	pass
	try:
		VidPipe=open(VidPipePath,"r")
		r = "1"
		while len(r):	r=VidPipe.read()
		FIFO.close()
	except:	pass
	try:	os.remove(VidPipePath)
	except:	pass
	if debugV:	print "Video-Process :      committed suicide"




##################################################################################################
###### Receive and Decode NavData															######
##################################################################################################
### Description:
### It follows lousy code for abetter documentation! Later there will be lousy code because of laziness; I will correct it later....maybe.
### You will (normally) find the names of the official AR.drone SDK 2.0, some comments and the official data type of that value.
### A lot of entries are reversed engineered; for some, I have no idea what they are doing or what their meaning is.
### It would be nice if you could give me a hint if you have some further information.

##### Header ##################################################################
def decode_Header(data):
#Bit 00-07: FLY_MASK, VIDEO_MASK, VISION_MASK, CONTROL_MASK, ALTITUDE_MASK, USER_FEEDBACK_START, COMMAND_MASK, CAMERA_MASK
#Bit 08-15: TRAVELLING_MASK, USB_MASK, NAVDATA_DEMO_MASK, NAVDATA_BOOTSTRAP, MOTORS_MASK, COM_LOST_MASK, SOFTWARE_FAULT, VBAT_LOW
#Bit 16-23: USER_EL, TIMER_ELAPSED, MAGNETO_NEEDS_CALIB, ANGLES_OUT_OF_RANGE, WIND_MASK, ULTRASOUND_MASK, CUTOUT_MASK, PIC_VERSION_MASK
#Bit 24-31: ATCODEC_THREAD_ON, NAVDATA_THREAD_ON, VIDEO_THREAD_ON, ACQ_THREAD_ON, CTRL_WATCHDOG_MASK, ADC_WATCHDOG_MASK, COM_WATCHDOG_MASK, EMERGENCY_MASK
	stateBit = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	stateBit[ 0] = data[1]    &1	#  0: FLY MASK :					(0) ardrone is landed, (1) ardrone is flying
	stateBit[ 1] = data[1]>> 1&1	#  1: VIDEO MASK :					(0) video disable, (1) video enable
	stateBit[ 2] = data[1]>> 2&1	#  2: VISION MASK :					(0) vision disable, (1) vision enable
	stateBit[ 3] = data[1]>> 3&1	#  3: CONTROL ALGO :				(0) euler angles control, (1) angular speed control
	stateBit[ 4] = data[1]>> 4&1	#  4: ALTITUDE CONTROL ALGO :	 	(0) altitude control inactive (1) altitude control active
	stateBit[ 5] = data[1]>> 5&1	#  5: USER feedback : 				Start button state
	stateBit[ 6] = data[1]>> 6&1	#  6: Control command ACK : 		(0) None, (1) one received
	stateBit[ 7] = data[1]>> 7&1	#  7: CAMERA MASK : 				(0) camera not ready, (1) Camera ready
	stateBit[ 8] = data[1]>> 8&1	#  8: Travelling mask : 			(0) disable, (1) enable
	stateBit[ 9] = data[1]>> 9&1	#  9: USB key : 					(0) usb key not ready, (1) usb key ready
	stateBit[10] = data[1]>>10&1	# 10: Navdata demo : 				(0) All navdata, (1) only navdata demo
	stateBit[11] = data[1]>>11&1	# 11: Navdata bootstrap : 			(0) options sent in all or demo mode, (1) no navdata options sent
	stateBit[12] = data[1]>>12&1	# 12: Motors status : 				(0) Ok, (1) Motors problem
	stateBit[13] = data[1]>>13&1	# 13: Communication Lost : 			(0) Com is ok, (1) com problem
	stateBit[14] = data[1]>>14&1	# 14: Software fault detected - user should land as quick as possible (1)
	stateBit[15] = data[1]>>15&1	# 15: VBat low : 					(0) Ok, (1) too low
	stateBit[16] = data[1]>>16&1	# 16: User Emergency Landing :		(0) User EL is OFF, (1) User EL is ON
	stateBit[17] = data[1]>>17&1	# 17: Timer elapsed : 				(0) not elapsed, (1) elapsed
	stateBit[18] = data[1]>>18&1	# 18: Magnetometer calib state :	(0) Ok, no calibration needed, (1) not ok, calibration needed
	stateBit[19] = data[1]>>19&1	# 19: Angles :						(0) Ok,	(1) out of range
	stateBit[20] = data[1]>>20&1	# 20: WIND MASK:					(0) Ok, (1) Too much wind
	stateBit[21] = data[1]>>21&1	# 21: Ultrasonic sensor :			(0) Ok, (1) deaf
	stateBit[22] = data[1]>>22&1	# 22: Cutout system detection :		(0) Not detected, (1) detected
	stateBit[23] = data[1]>>23&1	# 23: PIC Version number OK :		(0) a bad version number, (1) version number is OK
	stateBit[24] = data[1]>>24&1	# 24: ATCodec thread ON : 			(0) thread OFF, (1) thread ON
	stateBit[25] = data[1]>>25&1	# 25: Navdata thread ON : 			(0) thread OFF, (1) thread ON
	stateBit[26] = data[1]>>26&1	# 26: Video thread ON : 			(0) thread OFF, (1) thread ON
	stateBit[27] = data[1]>>27&1	# 27: Acquisition thread ON : 		(0) thread OFF, (1) thread ON
	stateBit[28] = data[1]>>28&1	# 28: CTRL watchdog : 				(0) control is well scheduled, (1) delay in control execution (> 5ms)
	stateBit[29] = data[1]>>29&1	# 29: ADC Watchdog :				(0) uart2 is good, (1) delay in uart2 dsr (> 5ms)
	stateBit[30] = data[1]>>30&1	# 30: Communication Watchdog :		(0) Com is ok, (1) com problem
	stateBit[31] = data[1]>>31&1	# 31: Emergency landing : 			(0) no emergency, (1) emergency
	stateBit[32] = data[2]
	stateBit[33] = data[3]
	# Alternative code:
	#	for i in range (0,32,1):	arState[i]=data>>i&1
	return (stateBit)

##### ID = 0 ### "demo" #######################################################
def decode_ID0(packet):		# NAVDATA_DEMO_TAG
	dataset = struct.unpack_from("HHIIfffifffIffffffffffffIIffffffffffff", packet, 0)
	if dataset[1] != 148:		print "*** ERROR : Navdata-Demo-Options-Package (ID=0) has the wrong size !!!"
	demo=[[0,0,0,0,0,0,0,0,0,0,0,0],0,[0,0,0],0,[0,0,0],0,[0,0,0,0,0,0,0,0,0],[0,0,0],0,0,[0,0,0,0,0,0,0,0,0],[0,0,0]]
	demo[0][ 0] = dataset[2]>>15&1	# DEFAULT			(bool)
	demo[0][ 1] = dataset[2]>>16&1	# INIT				(bool)
	demo[0][ 2] = dataset[2]>>17&1	# LANDED			(bool)
	demo[0][ 3] = dataset[2]>>18&1	# FLYING			(bool)
	demo[0][ 4] = dataset[2]>>19&1	# HOVERING			(bool)  (Seems like landing)
	demo[0][ 5] = dataset[2]>>20&1	# TEST				(bool)
	demo[0][ 6] = dataset[2]>>21&1	# TRANS_TAKEOFF		(bool)
	demo[0][ 7] = dataset[2]>>22&1	# TRANS_GOFIX		(bool)
	demo[0][ 8] = dataset[2]>>23&1	# TRANS_LANDING		(bool)
	demo[0][ 9] = dataset[2]>>24&1	# TRANS_LOOPING		(bool)
	demo[0][10] = dataset[2]>>25&1	# TRANS_NO_VISION	(bool)
	demo[0][11] = dataset[2]>>26&1	# NUM_STATE			(bool)
	demo[1]		=dataset[3]			# vbat_flying_percentage	battery voltage (filtered) in percent	(uint32)
	demo[2][0]	=dataset[4]/1000.0	# theta						pitch in degrees						(float)
	demo[2][1]	=dataset[5]/1000.0	# phi						roll  in degrees						(float)
	demo[2][2]	=dataset[6]/1000.0	# psi						yaw   in degrees						(float)
	demo[3]		=dataset[7]/10.0	# altitude					altitude in centimetres					(int32)
	demo[4][0]	=dataset[8]			# vx						estimated speed in X in mm/s			(float)
	demo[4][1]	=dataset[9]			# vy						estimated speed in Y in mm/s			(float)
	demo[4][2]	=dataset[10]		# vz						estimated speed in Z in mm/s			(float)
	demo[5]		=dataset[11]		# num_frames				streamed frame index 					(uint32) (Not used to integrate in video stage)
	for i in range (0,9,1):	demo[6][i]	= dataset[12+i]	# detection_camera_rot		Camera parameters compute by detection	(float matrix33)
	for i in range (0,3,1):	demo[7][i]	= dataset[21+i]	# detection_camera_trans	Deprecated ! Don't use !				(float vector31)
	demo[8]								= dataset[24]	# detection_tag_index		Deprecated ! Don't use !				(uint32)
	demo[9]								= dataset[25]	# detection_camera_type   	Type of tag								(uint32)
	for i in range (0,9,1):	demo[10][i]	= dataset[26+i]	# drone_camera_rot			Camera parameters computed by drone		(float matrix33)
	for i in range (0,3,1):	demo[11][i]	= dataset[35+i]	# drone_camera_trans		Deprecated ! Don't use !				(float vector31)
	return(demo)

##### ID = 1 ### "time" #######################################################
def decode_ID1(packet):			#NAVDATA_TIME_TAG
	dataset = struct.unpack_from("HHI", packet, 0)
	if dataset[1] != 8:		print "*** ERROR : navdata-time-Options-Package (ID=1) has the wrong size !!!"
	time=[0.0]
	# Value: 11 most significant bits represent the seconds, and the 21 least significant bits represent the microseconds.
	for i in range(0,21,1):		time[0] += ((dataset[2]>>i&1)*(2**i))		# Calculating the millisecond-part
	time[0] /= 1000000
	for i in range(21,32,1):	time[0] += (dataset[2]>>i&1)*(2**(i-21))	# Calculating second-part
	return(time)

##### ID = 2 ### "raw_measures" ################################################
def decode_ID2(packet):			#NAVDATA_RAW_MEASURES_TAG
	dataset = struct.unpack_from("HHHHHhhhhhIHHHHHHHHHHHHhh", packet, 0)
	if dataset[1] != 52:		print "*** ERROR : navdata-raw_measures-Options-Package (ID=2) has the wrong size !!!"
	raw_measures = [[0,0,0],[0,0,0],[0,0],0,0,0,0,0,0,0,0,0,0,0,0,0]
	for i in range(0,3,1):	raw_measures[0][i] = dataset[2+i]	# raw_accs[xyz]			filtered accelerometer-datas [LSB]	(uint16)
	for i in range(0,3,1):	raw_measures[1][i] = dataset[5+i]	# raw_gyros[xyz]		filtered gyrometer-datas [LSB]		(int16)
	for i in range(0,2,1):	raw_measures[2][i] = dataset[8+i]	# raw_gyros_110[xy]		gyrometers  x/y 110 deg/s [LSB]		(int16)
	raw_measures[ 3] = dataset[10]		# vbat_raw				battery voltage raw (mV)			(uint)
	raw_measures[ 4] = dataset[11]		# us_debut_echo			[LSB]								(uint16)
	raw_measures[ 5] = dataset[12]		# us_fin_echo			[LSB]								(uint16)
	raw_measures[ 6] = dataset[13]		# us_association_echo	[LSB]								(uint16)
	raw_measures[ 7] = dataset[14]		# us_distance_echo		[LSB]								(uint16)
	raw_measures[ 8] = dataset[15]		# us_courbe_temps		[LSB]								(uint16)
	raw_measures[ 9] = dataset[16]		# us_courbe_valeur		[LSB]								(uint16)
	raw_measures[10] = dataset[17]		# us_courbe_ref			[LSB]								(uint16)
	raw_measures[11] = dataset[18]		# flag_echo_ini			[LSB]								(uint16)
	raw_measures[12] = dataset[19]		# nb_echo				[LSB]								(uint16)
	raw_measures[13] = dataset[21]		# sum_echo				juRef_st lower 16Bit, upper 16Bit=tags?	(uint32)
	raw_measures[14] = dataset[23]		# alt_temp_raw			in Milimeter	(just lower 16Bit)	(int32)
	raw_measures[15] = dataset[24]		# gradient				[LSB]								(int16)
	return(raw_measures)

##### ID = 3 ### "phys_measures" ##############################################
def decode_ID3(packet):  		#NAVDATA_PHYS_MEASURES_TAG
	dataset = struct.unpack_from("HHfHffffffIII", packet, 0)
	if dataset[1] != 46:		print "*** ERROR : navdata-phys_measures-Options-Package (ID=3) has the wrong size !!!"
	phys_measures = [0,0,[0,0,0],[0,0,0],0,0,0]
	phys_measures[0] = dataset[2]	#float32   accs_temp
	phys_measures[1] = dataset[3]	#uint16    gyro_temp
	phys_measures[4] = dataset[10]	#uint32    alim3V3              3.3volt alim [LSB]
	phys_measures[5] = dataset[11]	#uint32    vrefEpson            ref volt Epson gyro [LSB]
	phys_measures[6] = dataset[12]	#uint32    vrefIDG              ref volt IDG gyro [LSB]
	dataset = struct.unpack_from(">HHfHffffffIII", packet, 0) 	#switch from little to big-endian
	for i in range(0,3,1):	phys_measures[2][i] = dataset[4+i]	#float32   phys_accs[xyz]
	for i in range(0,3,1):	phys_measures[3][i] = dataset[7+i]	#float32   phys_gyros[xyz]
	return(phys_measures)

##### ID = 4 ### "gyros_offsets" ##############################################
def decode_ID4(packet):  		#NNAVDATA_GYROS_OFFSETS_TAG
	dataset = struct.unpack_from("HHfff", packet, 0)
	if dataset[1] != 16:		print "*** ERROR : navdata-gyros_offsets-Options-Package (ID=4) has the wrong size !!!"
	gyros_offsets = [0,0,0]
	for i in range (0,3,1):		gyros_offsets[i]=dataset[i+2]	# offset_g[xyz]				in deg/s					(float)
	return(gyros_offsets)

##### ID = 5 ### "euler_angles" ###############################################
def decode_ID5(packet):			#NAVDATA_EULER_ANGLES_TAG
	dataset = struct.unpack_from("HHff", packet, 0)
	if dataset[1] != 12:		print "*** ERROR : navdata-euler_angles-Options-Package (ID=5) has the wrong size !!!"
	euler_angles = [0,0]
	euler_angles[0] = dataset[2]	#float32   theta_a (head/back)
	euler_angles[1] = dataset[3]	#float32   phi_a   (sides)
	return(euler_angles)

##### ID = 6 ### "references" #################################################
def decode_ID6(packet):			#NAVDATA_REFERENCES_TAG
	dataset = struct.unpack_from("HHiiiiiiiiffffffIfffffI", packet, 0)
	if dataset[1] != 88:		print "*** ERROR : navdata-references-Options-Package (ID=6) has the wrong size !!!"
	references = [[0,0,0],[0,0],[0,0,0],[0.0,0.0],[0.0,0.0],[0.0,0.0],0,[0.0,0.0,0.0,0.0,0.0,0]]
	references[0][0] = dataset[2]		#ref_theta  	Theta_ref_embedded [milli-deg]	(int32)
	references[0][1] = dataset[3]		#ref_phi		Phi_ref_embedded [milli-deg]	(int32)
	references[0][2] = dataset[9]		#ref_psi		Psi_ref_embedded [milli-deg]	(int32)
	references[1][0] = dataset[4]		#ref_theta_I	Theta_ref_int [milli-deg]		(int32)
	references[1][1] = dataset[5]		#ref_phi_I		Phi_ref_int [milli-deg]			(int32)
	references[2][0] = dataset[6]		#ref_pitch		Pitch_ref_embedded [milli-deg]	(int32)
	references[2][1] = dataset[7]		#ref_roll		Roll_ref_embedded [milli-deg]	(int32)
	references[2][2] = dataset[8]		#ref_yaw		Yaw_ref_embedded [milli-deg/s]	(int32)
	references[3][0] = dataset[10]		#vx_ref			Vx_Ref_[mm/s]					(float)
	references[3][1] = dataset[11]		#vy_ref			Vy_Ref_[mm/s]					(float)
	references[4][0] = dataset[12]		#theta_mod		Theta_modele [radian]			(float)
	references[4][1] = dataset[13]		#phi_mod		Phi_modele [radian]				(float)
	references[5][0] = dataset[14]		#k_v_x											(float)
	references[5][1] = dataset[15]		#k_v_y											(float)
	references[6]    = dataset[16]		#k_mode											(uint32)
	references[7][0] = dataset[17]		#ui_time										(float)
	references[7][1] = dataset[18]		#ui_theta										(float)
	references[7][2] = dataset[19]		#ui_phi											(float)
	references[7][3] = dataset[20]		#ui_psi											(float)
	references[7][4] = dataset[21]		#ui_psi_accuracy								(float)
	references[7][5] = dataset[22]		#ui_seq											(int32)
	return(references)

##### ID = 7 ### "trims" ######################################################
def decode_ID7(packet):			#NAVDATA_TRIMS_TAG
	dataset = struct.unpack_from("HHfff", packet, 0)
	if dataset[1] != 16:		print "*** ERROR : navdata-trims-Options-Package (ID=7) has the wrong size !!!"
	trims = [0,0,0]
	trims[0] = dataset[2]	#  angular_rates_trim									(float)
	trims[1] = dataset[3]	#  euler_angles_trim_theta	[milli-deg]					(float)
	trims[2] = dataset[4]	#  euler_angles_trim_phi	[milli-deg]					(float)
	return(trims)

##### ID = 8 ### "rc_references" ##############################################
def decode_ID8(packet):			#NAVDATA_RC_REFERENCES_TAG
	dataset = struct.unpack_from("HHiiiii", packet, 0)
	if dataset[1] != 24:		print "*** ERROR : navdata-rc_references-Options-Package (ID=8) has the wrong size !!!"
	rc_references = [0,0,0,0,0]
	rc_references[0] = dataset[2]	#  rc_ref_pitch		Pitch_rc_embedded			(int32)
	rc_references[1] = dataset[3]	#  rc_ref_roll		Roll_rc_embedded			(int32)
	rc_references[2] = dataset[4]	#  rc_ref_yaw		Yaw_rc_embedded				(int32)
	rc_references[3] = dataset[5]	#  rc_ref_gaz		Gaz_rc_embedded				(int32)
	rc_references[4] = dataset[6]	#  rc_ref_ag		Ag_rc_embedded				(int32)
	return(rc_references)

##### ID = 9 ### "pwm" ########################################################
def decode_ID9(packet):			#NAVDATA_PWM_TAG
	dataset = struct.unpack_from("HHBBBBBBBBffffiiifiiifHHHHff", packet, 0)
	if dataset[1] != 76 and dataset[1] != 92:   #92 since firmware 2.4.8 ?
		print "*** ERROR : navdata-navdata_pwm-Options-Package (ID=9) has the wrong size !!!"
		#print "Soll: 76     Ist:",dataset[1]
	pwm = [[0,0,0,0],[0,0,0,0],0.0,0.0,0.0,0.0,[0,0,0],0.0,[0,0,0,0.0],[0,0,0,0],0.0,0.0]
	for i in range(0,4,1):	pwm[0][i] = dataset[2+i]	#  motor1/2/3/4		[Pulse-width mod]	(uint8)
	for i in range(0,4,1):	pwm[1][i] = dataset[6+i]	#  sat_motor1/2/3/4	[Pulse-width mod]	(uint8)
	pwm[2]    = dataset[10]			# gaz_feed_forward		[Pulse-width mod]	(float)
	pwm[3]    = dataset[11]			# gaz_altitud			[Pulse-width mod]	(float)
	pwm[4]    = dataset[12]			# altitude_integral		[mm/s]				(float)
	pwm[5]    = dataset[13]			# vz_ref				[mm/s]				(float)
	pwm[6][0] = dataset[14]			# u_pitch				[Pulse-width mod]	(int32)
	pwm[6][1] = dataset[15]			# u_roll				[Pulse-width mod]	(int32)
	pwm[6][2] = dataset[16]			# u_yaw					[Pulse-width mod]	(int32)
	pwm[7]    = dataset[17]			# yaw_u_I				[Pulse-width mod]	(float)
	pwm[8][0] = dataset[18]			# u_pitch_planif		[Pulse-width mod]	(int32)
	pwm[8][1] = dataset[19]			# u_roll_planif			[Pulse-width mod]	(int32)
	pwm[8][2] = dataset[20]			# u_yaw_planif			[Pulse-width mod]	(int32)
	pwm[8][3] = dataset[21]			# u_gaz_planif			[Pulse-width mod]	(float)
	for i in range(0,4,1):
		pwm[9][i] = dataset[22+i]	# current_motor1/2/3/4	[mA]				(uint16)
	pwm[10]   = dataset[26]			# altitude_prop			[Pulse-width mod]	(float)
	pwm[11]   = dataset[27]			# altitude_der			[Pulse-width mod]	(float)
	return(pwm)

##### ID = 10 ### "altitude" ###################################################
def decode_ID10(packet):		#NAVDATA_ALTITUDE_TAG
	dataset = struct.unpack_from("HHifiiffiiiIffI", packet, 0)
	if dataset[1] != 56:		print "*** ERROR : navdata-navdata_altitude-Options-Package (ID=10) has the wrong size !!!"
	altitude = [0,0.0,0,0,0.0,0.0,[0,0,0],0,[0,0],0]
	altitude[0] = dataset[2]			# altitude_vision	[mm]					(int32)
	altitude[1] = dataset[3]			# altitude_vz		[mm/s]					(float)
	altitude[2] = dataset[4]			# altitude_ref		[mm]					(int32)
	altitude[3] = dataset[5]			# altitude_raw		[mm]					(int32)
	altitude[4] = dataset[6]			# obs_accZ			Observer AccZ [m/s2]	(float)
	altitude[5] = dataset[7]			# obs_alt			Observer altitude US [m](float)
	for i in range (0,3,1):
		altitude[6][i] = dataset[8+i]	# obs_x				3-Vector				(int32)
	altitude[7] = dataset[11]			# obs_state			Observer state [-]		(uint32)
	for i in range (0,2,1):
		altitude[8][i] = dataset[12+i]	# est_vb			2-Vector				(float)
	altitude[9] = dataset[14]			# est_state			Observer flight state 	(uint32)
	return(altitude)

##### ID = 11 ### "vision_raw" #################################################
def decode_ID11(packet):		#NAVDATA_VISION_RAW_TAG
	dataset = struct.unpack_from("HHfff", packet, 0)
	if dataset[1] != 16:	print "*** ERROR : navdata-vision_raw-Options-Package (ID=11) has the wrong size !!!"
	vision_raw = [0,0,0]
	for i in range (0,3,1):		vision_raw[i] = dataset[2+i]	#  vision_tx_raw (xyz)				(float)
	return(vision_raw)

##### ID = 12 ### "vision_of" #################################################
def decode_ID12(packet):		#NAVDATA_VISION_OF_TAG
	dataset = struct.unpack_from("HHffffffffff", packet, 0)
	if dataset[1] != 44:	print "*** ERROR : navdata-vision_of-Options-Package (ID=12) has the wrong size !!!"
	vision_of = [[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0]]
	for i in range (0,5,1):		vision_of[0][i] = dataset[2+i]	#  of_dx[5]							(float)
	for i in range (0,5,1):		vision_of[1][i] = dataset[7+i]	#  of_dy[5]							(float)
	return(vision_of)

##### ID = 13 ### "vision" #####################################################
def decode_ID13(packet):		#NAVDATA_VISION_TAG
	dataset = struct.unpack_from("HHIiffffifffiIffffffIIff", packet, 0)
	if dataset[1] != 92:	print "*** ERROR : navdata-vision-Options-Package (ID=13) has the wrong size !!!"
	vision=[0,0,0.0,0.0,0.0,0.0,0,[0.0,0.0,0.0],0,0.0,[0.0,0.0,0.0],[0.0,0.0,0.0],0,0,[0.0,0.0]]
	vision[0] = dataset[2]				# vision_state FIXME: What are the meanings of the tags ?
	vision[1] = dataset[3]				# vision_misc							(int32)
	vision[2] = dataset[4]				# vision_phi_trim						(float)
	vision[3] = dataset[5]				# vision_phi_ref_prop					(float)
	vision[4] = dataset[6]				# vision_theta_trim						(float)
	vision[5] = dataset[7]				# vision_theta_ref_prop					(float)
	vision[6] = dataset[8]				# new_raw_picture						(int32)
	for i in range (0,3,1):
		vision[7][i] = dataset[9+i]		# theta/phi/psi_capture					(float)
	vision[8] = dataset[12]				# altitude_capture						(int32)
	for i in range (0,21,1):						# Calculating milisecond-part
		vision[9] += ((dataset[13]>>i&1)*(2**i))
	vision[9] /= 1000000
	for i in range (21,32,1):						# Calculating second-part
		vision[9] += (dataset[13]>>i&1)*(2**(i-21))	# time_capture			(float)
	for i in range (0,3,1):
		vision[10][i] = dataset[14+i]	#  velocities[xyz]						(float)
	for i in range (0,3,1):
		vision[11][i] = dataset[17+i]	#  delta_phi/theta/psi					(float)
	vision[12] =    dataset[20]			# gold_defined							(uint32)
	vision[13] =    dataset[21]			# gold_reset							(uint32)
	vision[14][0] = dataset[22]			# gold_x								(float)
	vision[14][1] = dataset[23]			# gold_y								(float)
	return(vision)

##### ID = 14 ### "vision_perf" ###############################################
def decode_ID14(packet):		#NAVDATA_VISION_PERF_TAG
	dataset = struct.unpack_from("HHffffffffffffffffffffffffff", packet, 0)
	if dataset[1] != 108:		print "*** ERROR : navdata-vision_of-Options-Package (ID=14) has the wrong size !!!"
	vision_perf=[0.0,0.0,0.0,0.0,0.0,0.0,[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
	vision_perf[0] = dataset[2]				# time_szo								(float)
	vision_perf[1] = dataset[3]				# time_corners							(float)
	vision_perf[2] = dataset[4]				# time_compute							(float)
	vision_perf[3] = dataset[5]				# time_tracking							(float)
	vision_perf[4] = dataset[6]				# time_trans							(float)
	vision_perf[5] = dataset[7]				# time_update							(float)
	for i in range (0,20,1):
		vision_perf[6][i] = dataset[8+i]	# time_custom[20]						(float)
	return(vision_perf)

##### ID = 15 ### "trackers_send" #############################################
def decode_ID15(packet):  		#NAVDATA_TRACKERS_SEND_TAG
	dataset = struct.unpack_from("HHiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii", packet, 0)
	if dataset[1] != 364:		print "*** ERROR : navdata-trackers_send-Options-Package (ID=15) has the wrong size !!!"
	DEFAULT_NB_TRACKERS_WIDTH  = 6
	DEFAULT_NB_TRACKERS_HEIGHT = 5
	limit = DEFAULT_NB_TRACKERS_WIDTH*DEFAULT_NB_TRACKERS_HEIGHT
	trackers_send = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]]
	for i in range (0, limit, 1):
		trackers_send[0][i] = dataset[2+i]			#  locked[limit]				(int32)
	for i in range (0, limit, 1):
		trackers_send[1][i][0] = dataset[32+(i*2)]	#  point[x[limit],y[limit]]		(int32)
		trackers_send[1][i][1] = dataset[33+(i*2)]
	return(trackers_send)

##### ID = 16 ### "vision_detect" #############################################
def decode_ID16(packet):  		#NAVDATA_VISION_DETECT_TAG
	dataset = struct.unpack_from("HHIIIIIIIIIIIIIIIIIIIIIIIIIffffIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII", packet, offsetND)
	if dataset[1] != 328:	print "*** ERROR : navdata-vision_detect-Package (ID=16) has the wrong size !!!"
	vision_detect = [0,[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0.0,0.0,0.0,0.0],[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]],[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],[0,0,0,0]]
	#Max marker detection in one picture: 4
	vision_detect[0] = dataset[2]									 		# nb_detected						(uint32)
	for i in range (0,4,1):		vision_detect[1][i] = dataset[3+i]			# type[4]							(uint32)
	for i in range (0,4,1):		vision_detect[2][i] = dataset[7+i]			# xc[4]								(uint32)
	for i in range (0,4,1):		vision_detect[3][i] = dataset[11+i]			# yc[4]								(uint32)
	for i in range (0,4,1):		vision_detect[4][i] = dataset[15+i]			# width[4]							(uint32)
	for i in range (0,4,1):		vision_detect[5][i] = dataset[19+i]			# height[4]							(uint32)
	for i in range (0,4,1):		vision_detect[6][i] = dataset[23+i]			# dist[4]							(uint32)
	for i in range (0,4,1):		vision_detect[7][i] = dataset[27+i]			# orientation_angle[4]				(float)
	for i in range (0,4,1):
		for j in range (0,9,1):	vision_detect[8][i][j] = dataset[31+i+j]	# rotation[4]						(float 3x3 matrix (11,12,13,21,...)
	for i in range (0,4,1):
		for j in range (0,3,1):	vision_detect[9][i][j] = dataset[67+i+j]	# rotation[4]						(float 3 vector)
	for i in range (0,4,1):		vision_detect[10][i] = dataset[79+i]		# camera_source[4]					(uint32)
	return(vision_detect)

##### ID = 17 ### "watchdog" ###################################################
def decode_ID17(packet):  		#NAVDATA_WATCHDOG_TAG
	dataset = struct.unpack_from("HHI", packet, offsetND)
	if dataset[1] != 8:	print "*** ERROR : navdata-watchdog-Package (ID=17) has the wrong size !!!"
	watchdog = dataset[2]		# watchdog			Watchdog controll [-]				(uint32)
	return(watchdog)

##### ID = 18 ### "adc_data_frame" #############################################
def decode_ID18(packet):  		#NAVDATA_ADC_DATA_FRAME_TAG
	dataset = struct.unpack_from("HHIBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB", packet, offsetND)
	if dataset[1] != 40:	print "*** ERROR : navdata-adc_data_frame-Package (ID=18) has the wrong size !!!"
	adc_data_frame = [0,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]
	adc_data_frame[0] = dataset[2]									# version								(uint32)
	for i in range (0,32,1):	adc_data_frame[1][i] = dataset[3+i]	# data_frame[32]						(uint8)
	return(adc_data_frame)

##### ID = 19 ### "video_stream" ###############################################
def decode_ID19(packet):		#NAVDATA_VIDEO_STREAM_TAG
	dataset = struct.unpack_from("HHBIIIIfIIIiiiiiII", packet, offsetND)
	if dataset[1] != 65:	print "*** ERROR : navdata-video_stream-Package (ID=19) has the wrong size !!!"
	video_stream = [0,0,0,0,0,0.0,0,0,0,[0,0,0,0,0],0,0]
	video_stream[0] = dataset[2]	# quant   		quantizer reference used to encode [1:31]   				(uint8)
	video_stream[1] = dataset[3]	# frame_size	frame size in bytes   										(uint32)
	video_stream[2] = dataset[4]	# frame_number	frame index   												(uint32)
	video_stream[3] = dataset[5]	# atcmd_ref_seq	atmcd ref sequence number   								(uint32)
	video_stream[4] = dataset[6]	# atcmd_mean_ref_gap	mean time between two consecutive atcmd_ref (ms)	(uint32)
	video_stream[5] = dataset[7]	# atcmd_var_ref_gap															(float)
	video_stream[6] = dataset[8]	# atcmd_ref_quality		estimator of atcmd link quality   					(uint32)
	#Drone 2.0:
	video_stream[7] = dataset[9]	# out_bitrate			measured out throughput from the video tcp socket	(uint32)
	video_stream[8] = dataset[10]	# desired_bitrate		last frame size generated by the video encoder		(uint32)
	for i in range (0,5,1):		video_stream[9][i] = dataset[11+i]	# data		misc temporary data				(int32)
	video_stream[10] = dataset[16]	# tcp_queue_level		queue usage											(uint32)
	video_stream[11] = dataset[17]	# fifo_queue_level		queue usage											(uint32)
	return(video_stream)

##### ID = 20 ### "games" ######################################################
def decode_ID20(packet):  		#NAVDATA_GAMES_TAG
	dataset = struct.unpack_from("HHII", packet, offsetND)
	if dataset[1] != 12:	print "*** ERROR : navdata-games-Package (ID=20) has the wrong size !!!"
	games = [0,0]
	games[0] = dataset[2]	# double_tap_counter 			   							(uint32)
	games[1] = dataset[3]	# finish_line_counter										(uint32)
	return(games)

##### ID = 21 ### "pressure_raw" ###############################################
def decode_ID21(packet):  		#NAVDATA_PRESSURE_RAW_TAG
	dataset = struct.unpack_from("HHihii", packet, offsetND)
	if dataset[1] != 18:	print "*** ERROR : navdata-pressure_raw-Package (ID=21) has the wrong size !!!"
	pressure_raw = [0,0,0,0]
	pressure_raw[0] = dataset[2]	# up 			   									(int32)
	pressure_raw[1] = dataset[3]	# ut												(int16)
	pressure_raw[2] = dataset[4]	# Temperature_meas 									(int32)
	pressure_raw[3] = dataset[5]	# Pression_meas										(int32)
	return(pressure_raw)

##### ID = 22 ### "magneto" ####################################################
def decode_ID22(packet):		#NAVDATA_MAGNETO_TAG
	dataset = struct.unpack_from("HHhhhffffffffffffBifff", packet, offsetND)
	if dataset[1] != 83:	print "*** ERROR : navdata-magneto-Package (ID=22) has the wrong size !!!"
	magneto = [[0,0,0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],0.0,0.0,0.0,0,0,0.0,0.0,0.0]
	for i in range (0,3,1):		magneto[0][i]=dataset[2+i]	# mx/my/mz											(int16)
	for i in range (0,3,1):		magneto[1][i]=dataset[5+i]	# magneto_raw		magneto in the body frame [mG]	(vector float)
	for i in range (0,3,1):		magneto[2][i]=dataset[8+i]	# magneto_rectified									(vector float)
	for i in range (0,3,1):		magneto[3][i]=dataset[11+i]	# magneto_offset									(vector float)
	magneto[ 4] = dataset[14]								# heading_unwrapped 								(float)
	magneto[ 5] = dataset[15]								# heading_gyro_unwrapped							(float)
	magneto[ 6] = dataset[16]								# heading_fusion_unwrapped 							(float)
	magneto[ 7] = dataset[17]								# magneto_calibration_ok							(char)
	magneto[ 8] = dataset[18]								# magneto_state 									(uint32)
	magneto[ 9] = dataset[19]								# magneto_radius									(float)
	magneto[10] = dataset[20]								# error_mean 										(float)
	magneto[11] = dataset[21]								# error_var											(float)
	return(magneto)

##### ID = 23 ### "wind_speed" ################################################
def decode_ID23(packet):		#NAVDATA_WIND_TAG
	dataset = struct.unpack_from("HHfffffffffffff", packet, offsetND)
	if dataset[1] != 56 and dataset[1] != 64:
		print "*** ERROR : navdata-wind_speed-Package (ID=23) has the wrong size !!!"
	wind_speed = [0.0,0.0,[0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0]]
	wind_speed[0]    = dataset[2]							# wind_speed 			   					(float)
	wind_speed[1]    = dataset[3]							# wind_angle								(float)
	wind_speed[2][0] = dataset[4]							# wind_compensation_theta 					(float)
	wind_speed[2][1] = dataset[5]							# wind_compensation_phi						(float)
	for i in range (0,6,1):	wind_speed[3][i]=dataset[6+i]	# state_x[1-6]								(float)
	for i in range (0,3,1):	wind_speed[4][i]=dataset[7+i]	# magneto_debug[1-3]						(float)
	return(wind_speed)

##### ID = 24 ### "kalman_pressure" ###########################################
def decode_ID24(packet):  		#NAVDATA_KALMAN_PRESSURE_TAG
	dataset = struct.unpack_from("HHffffffffff?f?ff??", packet, offsetND)
	if dataset[1] != 72:	print "*** ERROR : navdata-wind_speed-Package (ID=24) has the wrong size !!!"
	kalman_pressure = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0.0,False,0.0,0.0,False,False]
	kalman_pressure[ 0] = dataset[2]	# offset_pressure 			   					(float)
	kalman_pressure[ 1] = dataset[3]	# est_z											(float)
	kalman_pressure[ 2] = dataset[4]	# est_zdot 										(float)
	kalman_pressure[ 3] = dataset[5]	# est_bias_PWM 									(float)
	kalman_pressure[ 4] = dataset[6]	# est_biais_pression							(float)
	kalman_pressure[ 5] = dataset[7]	# offset_US 			   						(float)
	kalman_pressure[ 6] = dataset[8]	# prediction_US									(float)
	kalman_pressure[ 7] = dataset[9]	# cov_alt 										(float)
	kalman_pressure[ 8] = dataset[10]	# cov_PWM										(float)
	kalman_pressure[ 9] = dataset[11]	# cov_vitesse									(float)
	kalman_pressure[10] = dataset[12]	# bool_effet_sol								(bool)
	kalman_pressure[11] = dataset[13]	# somme_inno									(float)
	kalman_pressure[12] = dataset[14]	# flag_rejet_US									(bool)
	kalman_pressure[13] = dataset[15]	# u_multisinus									(float)
	kalman_pressure[14] = dataset[16]	# gaz_altitude									(float)
	kalman_pressure[15] = dataset[17]	# Flag_multisinus								(bool)
	kalman_pressure[16] = dataset[18]	# Flag_multisinus_debut							(bool)
	return(kalman_pressure)

##### ID = 25 ### "hdvideo_stream" ############################################
def decode_ID25(packet):		#NAVDATA_HDVIDEO-TAG
	dataset = struct.unpack_from("HHfffffff", packet, offsetND)
	if dataset[1] != 32:	print "*** ERROR : navdata-hdvideo_stream-Package (ID=25) has the wrong size !!!"
	hdvideo_stream = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	hdvideo_stream[0] = dataset[2]	# hdvideo_state 			   					(float)
	hdvideo_stream[1] = dataset[3]	# storage_fifo_nb_packets						(float)
	hdvideo_stream[2] = dataset[4]	# storage_fifo_size 							(float)
	hdvideo_stream[3] = dataset[5]	# usbkey_size 			USB key in kb (no key=0)(float)
	hdvideo_stream[4] = dataset[6]	# usbkey_freespace		USB key in kb (no key=0)(float)
	hdvideo_stream[5] = dataset[7]	# frame_number 			PaVE field of the frame starting to be encoded for the HD stream (float)
	hdvideo_stream[6] = dataset[8]	# usbkey_remaining_time	[sec]					(float)
	return(hdvideo_stream)

##### ID = 26 ### "wifi" ######################################################
def decode_ID26(packet):		#NAVDATA_WIFI_TAG
	dataset = struct.unpack_from("HHI", packet, offsetND)
	if dataset[1] != 8:	print "*** ERROR : navdata-wifi-Package (ID=26) has the wrong size !!!"
	wifi = dataset[2]	# link_quality 			   								(uint32)
	return(wifi)

##### ID = 27 ### "zimmu_3000" ################################################
def decode_ID27(packet):  #NAVDATA_ZIMU_3000_TAG
	dataset = struct.unpack_from("HHif", packet, offsetND)
	if dataset[1] != 12 and dataset[1] != 216:		# 216 since firmware 2.4.8 ?
		print "*** ERROR : navdata-zimmu_3000-Package (ID=27) has the wrong size !!!"
	zimmu_3000 = [0,0.0]
	zimmu_3000[0] = dataset[2]	# vzimmuLSB 			   							(int32)
	zimmu_3000[1] = dataset[3]	# vzfind 			   								(float)
	return(zimmu_3000)

##### Footer ### "chksum" #####################################################
def decode_Footer(packet,allpacket):   ### Decode Checksum options-package ID=65535
	dataset = struct.unpack_from("HHI", packet, offsetND)
	if dataset[1] != 8:	print "*** ERROR : Checksum-Options-Package (ID=65535) has the wrong size !!!"
	chksum = 	[0,False]
	chksum[0] = dataset[2]
	sum, plen =	0, len(allpacket)-8
	for i in range (0,plen,1):	sum += ord(allpacket[i])		# Slows down this Navdata-subprocess massivly
	if sum == chksum[0]:	chksum[1] = True
	return(chksum)


###############################################################################
### Navdata-Decoding
###############################################################################
def getDroneStatus(packet):
	arState =	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	checksum = 	(0,False)
	length =	len(packet)
	dataset = 	struct.unpack_from("IIII", packet, 0)	# Reading (Header, State, Sequence, Vision)
	offsetND =	struct.calcsize("IIII")

###############################=-
###	Decode Options-Packages ###=-
###############################=-
def getNavdata(packet,choice):
	navdata =	{}
	length =	len(packet)
	dataset = 	struct.unpack_from("IIII", packet, 0)	# Reading (Header, State, Sequence, Vision)
	navdata["state"] = decode_Header(dataset)
	offsetND = 	struct.calcsize("IIII")
	#Demo-mode contains normally Option-Packages with ID=0 (_navdata_demo_t), ID=16 (seems empty) and ID=65535 (checksum)
	# Full Mode contains
	while offsetND < length:
		dataset =  struct.unpack_from("HH", packet, offsetND)	# Reading (Header, Length)
		if dataset[0]== 0 and choice[ 0]: navdata["demo"]				= decode_ID0(packet[offsetND:])
		if dataset[0]== 1 and choice[ 1]: navdata["time"] 				= decode_ID1(packet[offsetND:])
		if dataset[0]== 2 and choice[ 2]: navdata["raw_measures"]		= decode_ID2(packet[offsetND:])
		if dataset[0]== 3 and choice[ 3]: navdata["phys_measures"] 		= decode_ID3(packet[offsetND:])
		if dataset[0]== 4 and choice[ 4]: navdata["gyros_offsets"] 		= decode_ID4(packet[offsetND:])
		if dataset[0]== 5 and choice[ 5]: navdata["euler_angles"]		= decode_ID5(packet[offsetND:])
		if dataset[0]== 6 and choice[ 6]: navdata["references"] 		= decode_ID6(packet[offsetND:])
		if dataset[0]== 7 and choice[ 7]: navdata["trims"]				= decode_ID7(packet[offsetND:])
		if dataset[0]== 8 and choice[ 8]: navdata["rc_references"] 		= decode_ID8(packet[offsetND:])
		if dataset[0]== 9 and choice[ 9]: navdata["pwm"]				= decode_ID9(packet[offsetND:])
		if dataset[0]==10 and choice[10]: navdata["altitude"]			= decode_ID10(packet[offsetND:])
		if dataset[0]==11 and choice[11]: navdata["vision_raw"] 		= decode_ID11(packet[offsetND:])
		if dataset[0]==12 and choice[12]: navdata["vision_of"] 			= decode_ID12(packet[offsetND:])
		if dataset[0]==13 and choice[13]: navdata["vision"]				= decode_ID13(packet[offsetND:])
		if dataset[0]==14 and choice[14]: navdata["vision_perf"]		= decode_ID14(packet[offsetND:])
		if dataset[0]==15 and choice[15]: navdata["trackers_send"] 		= decode_ID15(packet[offsetND:])
		if dataset[0]==16 and choice[16]: navdata["vision_detect"]		= decode_ID16(packet[offsetND:])
		if dataset[0]==17 and choice[17]: navdata["watchdog"] 			= decode_ID17(packet[offsetND:])
		if dataset[0]==18 and choice[18]: navdata["adc_data_frame"] 	= decode_ID18(packet[offsetND:])
		if dataset[0]==19 and choice[19]: navdata["video_stream"] 		= decode_ID19(packet[offsetND:])
		if dataset[0]==20 and choice[20]: navdata["games"]				= decode_ID20(packet[offsetND:])
		if dataset[0]==21 and choice[21]: navdata["pressure_raw"]		= decode_ID21(packet[offsetND:])
		if dataset[0]==22 and choice[22]: navdata["magneto"]			= decode_ID22(packet[offsetND:])
		if dataset[0]==23 and choice[23]: navdata["wind_speed"] 		= decode_ID23(packet[offsetND:])
		if dataset[0]==24 and choice[24]: navdata["kalman_pressure"]	= decode_ID24(packet[offsetND:])
		if dataset[0]==25 and choice[25]: navdata["hdvideo_stream"]		= decode_ID25(packet[offsetND:])
		if dataset[0]==26 and choice[26]: navdata["wifi"]				= decode_ID26(packet[offsetND:])
		if dataset[0]==27 and choice[27]: navdata["zimmu_3000"]			= decode_ID27(packet[offsetND:])
		if dataset[0]==65535 and choice[28]: navdata["chksum"]			= decode_Footer(packet[offsetND:],packet)
		offsetND += dataset[1]
	return(navdata)

###############################=-
###	Threads					###=-
###############################=-
def reconnect(navdata_pipe, commitsuicideND, DroneIP,NavDataPort):
	if not commitsuicideND:		navdata_pipe.sendto("\x01\x00\x00\x00", (DroneIP, NavDataPort))

def watchdogND(parentPID):
	global commitsuicideND
	while not commitsuicideND:
		time.sleep(1)
		try : 		os.getpgid(parentPID)
		except:		commitsuicideND=True
# It seems that you just have to reinitialize the network-connection once and the drone keeps on sending forever then.

def mainloopND(DroneIP,NavDataPort,parent_pipe,parentPID):
	global commitsuicideND
	something2send, MinimalPacketLength, timetag =	False, 30, 0
	packetlist =		["demo","time","raw_measures","phys_measures","gyros_offsets","euler_angles","references","trims","rc_references","pwm","altitude","vision_raw","vision_of","vision","vision_perf","trackers_send","vision_detect","watchdog","adc_data_frame","video_stream","games","pressure_raw","magneto","wind_speed","kalman_pressure","hdvideo_stream","wifi","zimmu_3000","chksum","state"]
	choice =			[False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True]
	overallchoice =		False	# This and oneTimeFailOver is necessary because of a bug (?) of AR.Drone sending NavData in DemoMode...
	oneTimeFailOver =	True 	# ...while setting a configuration the drone sends the next DemoMode-package with just its status.
	debug =				False
	showCommands =		False

	# Checks if the main-process is running and sends ITS own PID back
	ThreadWatchdogND = threading.Thread(target=watchdogND,args=[parentPID])
	ThreadWatchdogND.start()

	# Prepare communication-pipes
	pipes = []
	pipes.append(parent_pipe)

	navdata_pipe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	navdata_pipe.setblocking(0)
	navdata_pipe.bind(('', NavDataPort))
	pipes.append(navdata_pipe)

	# start connection
	reconnect(navdata_pipe, commitsuicideND, DroneIP, NavDataPort)
	netHeartbeat = threading.Timer(2.0, reconnect, [navdata_pipe,commitsuicideND,DroneIP,NavDataPort,])	# Inits the first Network-Heartbeat (2 secs after disconnection the drone stops sending)
	netHeartbeat.start()

	if choice.count(True) > 0: overallchoice = True

	while not commitsuicideND:
		in_pipe, out_pipe, dummy2 = select.select(pipes, [], [], 0.5)		# When something is in a pipe...
		for ip in in_pipe:
			if ip == parent_pipe:
				cmd = parent_pipe.recv()
				if showCommands:		print "** Com -> Nav : ",cmd
				# Signal to stop this process and all its threads
				if cmd == "die!":				commitsuicideND = True
				# Enables/disables Debug-bit
				elif cmd == "debug":
					debug = True
					print "NavData-Process :    running"
				elif cmd == "undebug":
					debug = False
				# Enables/disables Debug-bit
				elif cmd == "showCommands":	showCommands = True
				elif cmd == "hideCommands":	showCommands = False
				elif cmd == "reconnect":		reconnect(navdata_pipe, commitsuicideND, DroneIP, NavDataPort)
				# Sets explicitly the value-packages which shall be decoded
				elif cmd[0] == "send":
					if cmd[1].count("all"):
						for i in range (0,len(choice),1):		choice[i] = True
					else:
						for i in range (0,len(packetlist),1):
							if cmd[1].count(packetlist[i]):	choice[i] = True
							else:								choice[i] = False
					if choice.count(True) > 0:		overallchoice = True
					else: 								overallchoice = False
				# Adds value-packages to the other which shall be decoded
				elif cmd[0] == "add":
					for i in range (0,len(packetlist),1):
						if cmd[1].count(packetlist[i]):		choice[i] = True
					if cmd[1].count("all"):
						for i in range (0,len(choice),1):		choice[i] = True
					if choice.count(True)>0: 			overallchoice = True
					else: 								overallchoice = False
				# Deletes packages from the value-package-list which shall not be decoded anymore
				elif cmd[0] == "block":
					if cmd[1].count("all"):
						for i in range (0,len(packetlist),1):	choice[i] = False
					else:
						for i in range (0,len(packetlist),1):
							if cmd.count(packetlist[i]):		choice[i] = False
					if choice.count(True) > 0:			overallchoice = True
					else: 								overallchoice = False
			if ip == navdata_pipe:
				try:
					netHeartbeat.cancel()									# Connection is alive, Network-Heartbeat not necessary for a moment
					Packet = 		navdata_pipe.recv(65535)				# Receiving raw NavData-Package
					netHeartbeat = 	threading.Timer(2.1,reconnect,[navdata_pipe,commitsuicideND,DroneIP,NavDataPort])
					netHeartbeat.start()									# Network-Heartbeat is set here, because the drone keeps on sending NavData (vid, etc you have to switch on)
					timestamp =		timetag									# Setting up decoding-time calculation
					timetag = 		time.time()
					if overallchoice:
						try:		lastdecodedNavData=decodedNavData
						except:		lastdecodedNavData={}
					decodedNavData = getNavdata(Packet,choice)
					state = decodedNavData["state"]
					# If there is an abnormal small NavPacket, the last NavPacket will be sent out with an error-tag
					NoNavData = False
					if len(Packet)<MinimalPacketLength and overallchoice:	decodedNavData, NoNavData = lastdecodedNavData, True
					dectime = time.time()-timetag
					# Sends all the data to the mainprocess
					parent_pipe.send((decodedNavData, state[0:32], state[32], timestamp, dectime, NoNavData))
				except IOError:	pass
	suicideND = True
	netHeartbeat.cancel()
	if debug:	print "NavData-Process :    committed suicide"




##################################################################################################
###### Playground																			######
##################################################################################################
if __name__ == "__main__":
###
### Here you can write your first test-codes and play around with them
###

	import time
	import ps_drone

	drone = ps_drone.Drone()								# Start using drone
	drone.printBlue("Battery: ")

	drone.startup()											# Connects to drone and starts subprocesses
	drone.reset()											# Always good, at start

	while drone.getBattery()[0] == -1:	time.sleep(0.1)		# Waits until the drone has done its reset
	time.sleep(0.5)											# Give it some time to fully awake

	drone.printBlue("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1]))	# Gives a battery-status

	stop = False
	while not stop:
		key = drone.getKey()
		if key == " ":
			if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:	drone.takeoff()
			else:																drone.land()
		elif key == "0":	drone.hover()
		elif key == "w":	drone.moveForward()
		elif key == "s":	drone.moveBackward()
		elif key == "a":	drone.moveLeft()
		elif key == "d":	drone.moveRight()
		elif key == "q":	drone.turnLeft()
		elif key == "e":	drone.turnRight()
		elif key == "7":	drone.turnAngle(-10,1)
		elif key == "9":	drone.turnAngle( 10,1)
		elif key == "4":	drone.turnAngle(-45,1)
		elif key == "6":	drone.turnAngle( 45,1)
		elif key == "1":	drone.turnAngle(-90,1)
		elif key == "3":	drone.turnAngle( 90,1)
		elif key == "8":	drone.moveUp()
		elif key == "2":	drone.moveDown()
		elif key == "*":	drone.doggyHop()
		elif key == "+":	drone.doggyNod()
		elif key == "-":	drone.doggyWag()
		elif key != "":		stop = True

	print "Batterie: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
