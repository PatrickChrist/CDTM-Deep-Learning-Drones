#!/usr/bin/env python

"""Eagle_Eye app for the AR.Drone.

This application allows to control the AR.Drone via a PC keyboard while seeing the camera stream.
It also features an autonomous mode in which the drone follows an "aruco"-marker without active control by a pilot.
"""


import cv2
import cv2.aruco as aruco
import libardrone.libardrone as libardrone
import numpy as np

import threading
import time

exitFlag = 0

# thread to control the drone while in manual control mode. Basic control stays active in
# autonomous mode to still allow interception if required
class manualControlThread (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.running = True
		
	def run(self):
		
		global drone
		global exiting
		global manual_mode
		global thread_lock
		global thread_lock_manual_mode
		global thread_lock_key
		global key
		
		print "Starting " + self.name
		
		while self.running:
			# query pressed keys
			thread_lock_key.acquire()
			k = key
			thread_lock_key.release()
			# escape to stop program execution
			if k == 27: # 27=escape
				self.running = False
				thread_lock.acquire()
				exiting = True
				thread_lock.release()
				drone.reset()
			 # takeoff
			elif k == 13: # 13=enter
				print("return")
				drone.takeoff()
			# land
			elif k == 32: # 32=space
				print("space")
				drone.land()
			# emergency
			elif k == 8: # 8=backspace
				drone.reset()
			# switch control mode
			elif k == ord('m'):
				drone.hover()
				thread_lock_manual_mode.acquire()
				manual_mode = not manual_mode
				thread_lock_manual_mode.release()
	 		 # switch between manual and autonomous control
			elif manual_mode:
				# listen for additional key events for manual control
				# forward / backward
				if k == ord('w'):
					drone.move_forward()
				elif k == ord('s'):
					drone.move_backward()
				# left / right
				elif k == ord('a'):
					drone.move_left()
				elif k == ord('d'):
					drone.move_right()
				# up / down
				elif k == 2490368:
					drone.move_up()
				elif k == 2621440:
					drone.move_down()
				# turn left / turn right
				elif k == 2424832:
					drone.turn_left()
				elif k == 2555904:
					drone.turn_right()
				# speed
				elif k == ord('1'):
					drone.speed = 0.1
				elif k == ord('2'):
					drone.speed = 0.2
				elif k == ord('3'):
					drone.speed = 0.3
				elif k == ord('4'):
					drone.speed = 0.4
				elif k == ord('5'):
					drone.speed = 0.5
				elif k == ord('6'):
					drone.speed = 0.6
				elif k == ord('7'):
					drone.speed = 0.7
				elif k == ord('8'):
					drone.speed = 0.8
				elif k == ord('9'):
				  drone.speed = 0.9
				elif k == ord('0'):
				  drone.speed = 1.0
				# if no matching input: hover
				else:
				  drone.hover()
			  
		print("Shutting down...")
		drone.halt()
		print("Ok.")		
		print "Exiting " + self.name

# thread that performs the automatic control of the drone while not in manual mode
class automaticControlThread (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.running = True
		self.status = "Start"
		
		
	def run(self):
		
		global drone
		global thread_lock
		global thread_lock_manual_mode
		global thread_lock_camera_frame
		global exiting
		global manual_mode
		global W
		global H
		global new_frame
		global render_frame
		print "Starting " + self.name
		
		# initializations for startup
		miss_counter = 0
		p = (-W	,0)	
		
		# loop while flight active
		while self.running:
			thread_lock.acquire()
			if exiting:
				thread_lock.release()
				self.running = False
			else:
				thread_lock.release()
				
				thread_lock_manual_mode.acquire()
				# check if in autonomous mode
				if not manual_mode:
					thread_lock_manual_mode.release()

					# Tracking 
					if self.status == "Start":
						# To do with image recognition
						self.status = "Searching"
					elif self.status == "Searching":
						# To do turn and search marker
						p,area = self.getAndSearchImage()
						if p[0] >= -W/2:
							self.status = "Tracking"
						else:
							drone.hover()
						print 'Searching'
					elif self.status == "Tracking":
						# While in tracking mode, allow 4 misses(frames without marker detection) before going back to hovering
						if p[0] >= -W/2:
							miss_counter = 0
							# call controller
							self.controlStep(p,area)
							# get next frame from camera stream, track marker and display it
							p,area = self.getAndSearchImage()
						elif miss_counter < 5:
							miss_counter += 1
						else:
							self.status = "Searching"
						print 'tracking'
				# if not in autonomous mode: deliver frames from camarea stream without marker tracking (colored)
				else:
					thread_lock_manual_mode.release()
					try:
						# pull image
						pixelarray = drone.get_image()
						if pixelarray != None:
							frame = pixelarray[:,:,::-1].copy()
						#resize image
							resized=cv2.resize(frame,(W,H))
						
						thread_lock_camera_frame.acquire()
						render_frame = resized
						new_frame = True
						thread_lock_camera_frame.release()
					except:
						pass
		print "Exiting" + self.name
		
	# function used by the automomous control thread retrieve an image from the camera stream and perform the tracking
	def getAndSearchImage(self):
		global drone
		global thread_lock_camera_frame
		global render_frame
		global new_frame
		global W
		global H
		try:
			# print pygame.image
			pixelarray = drone.get_image()
			if pixelarray != None:
				frame = pixelarray[:,:,::-1].copy()
				#resize image
				resized=cv2.resize(frame,(W,H))
				# aruco detection
				gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
				aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
				parameters =  aruco.DetectorParameters_create()
		
				#lists of ids and the corners beloning to each id
				corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
				
				# iterate over all found markers to determine the one to follow (biggest one)
				selected_corners = []
				max_index = (0,0)
				counter = 0
				if len(corners) > 0:
					dim = corners[0].shape
					#print dim
					while counter<dim[0]:
						tmp_corners = ((corners[0])[counter])
						# A=(1/2)|[(x3-x1)(y4-y2) +(x4-x2)(y1-y3)]|
						area = 0.5*((tmp_corners[2][0] - tmp_corners[0][0]) * (tmp_corners[3][1] - tmp_corners[1][1]) + (tmp_corners[3][0] - tmp_corners[1][0]) * (tmp_corners[0][1] - tmp_corners[2][1]))
						if area > max_index[0]:
							max_index = (area,counter)
						counter +=1

					max_corners = ((corners[0])[max_index[1]])
					selected_corners = np.array([np.array([(corners[0])[max_index[1]]],dtype=np.float32)])#[max_index[0]*4:max_index[0]*4+3]
					
				# draw all markers
				display = aruco.drawDetectedMarkers(resized, corners)

				thread_lock_camera_frame.acquire()
				render_frame = display
				new_frame = True
				thread_lock_camera_frame.release()

				# prepare function output
				if len(selected_corners) > 0:
					x,y = max_corners.sum(axis=0)/4
					area = max_index[0]
				else:
					x = -W
					y = -1
					area = -1
				return (x-W/2,y-H/2), area
		except:
			pass
		
	# function to perform a single control step in the autonomous control thread
	# TODO: vectorize calculation to avoid redundancy, determine I&D controller values(currently commented out)
	def controlStep(self,p,area):
		
		global tx_prev
		global uix_prev
		global ex_prev
		global uif_prev
		global ef_prev
		global uiy_prev
		global ey_prev
		up_down = 0
		left_right = 0
		front_back = 0
		x = p[0]	  
		y = p[1]
		
		move_command = False

		MAX_SPEED_ROT = 1.5
		MAX_SPEED_MOVE = 2.0

		# control direction

		K_px=1.0
		K_dx=1.0
		K_ix=1.0	
		ux_threshold = 15		
			
		#control x
		#error for x between the desired and actual output
		ex = 0 - x
		if tx_prev == 0:
			tx_prev = time.time() - 0.008
		tx = time.time() - tx_prev
		
		#Integration input
		uix = uix_prev + 1/K_ix * tx*ex
		#Derivation input
		udx = 1/K_dx * (ex-ex_prev)/tx
		
		#adjust previous values
		ex_prev = ex
		tx_prev += tx
		uix_prev = uix
	
		#calculate input for the system
		ux = K_px * (ex) #+ uix + udx)
		
		if ux < -ux_threshold or ux > ux_threshold:
			left_right = MAX_SPEED_ROT * ux / W * 2
			print 'left_right: '+str(MAX_SPEED_ROT  * ux / W * 2)
			move_command = True
		
		#control height
		K_py=0.5
		K_dy=1.0
		K_iy=1.0	
		uy_threshold = 0.1		
				
		#control y		
		#error for y between the desired and actual output
		ey = 0 - y
		ty = tx
	
		#Integration input
		uiy = uiy_prev + 1/K_iy * ty*ey
		#Derivation input
		udy = 1/K_dy * (ey-ey_prev)/ty
		
		#adjust previous values
		ey_prev = ey
		uiy_prev = uiy
		
		#calculate input for the system
		uy = 2.0/H*K_py * (ey) #+ uiy + udy)
		if uy < -uy_threshold or uy > uy_threshold:
			up_down = MAX_SPEED_MOVE * uy
			move_command = True
			print 'up_down: '+str(MAX_SPEED_MOVE * uy)
		
		# control forward
		K_pf=0.4
		K_df=1.0
		K_if=1.0	
		uf_threshold = 0.005
			
		#control f
		#error for f between the desired and actual output
		ef = 0.2 - (area/(W*H)) **0.5
		tf = tx
		#Integration input
		uif = uif_prev + 1/K_if * tf*ef
		#Derivation input
		udf = 1/K_df * (ef-ef_prev)/tf
		
		#adjust previous values
		ef_prev = ef
		uif_prev = uif
	
		#calculate input for the system
		uf = K_pf * (ef) #+ uif + udf)
		
		if uf < -uf_threshold or uf > uf_threshold:
			front_back = MAX_SPEED_MOVE * uf 
			move_command = True		
			print 'front_back: '+str(MAX_SPEED_MOVE * uf)

		# apply control vectors
		drone.at(libardrone.at_pcmd, move_command, 0, -front_back, up_down, -left_right)


def main():
	global drone
	global thread_lock
	global thread_lock_manual_mode
	global thread_lock_camera_frame
	global thread_lock_key
	global exiting
	global manual_mode
	global W
	global H
	global render_frame
	global new_frame
	global key
	global tx_prev
	global uix_prev
	global ex_prev
	global uif_prev
	global ef_prev
	global uiy_prev
	global ey_prev

	#initialization
	W, H = 640, 360
	key = -1
	
	drone = libardrone.ARDrone(True)
	drone.reset()
	
	exiting = False
	manual_mode = True
	new_frame = False
	threads = []
	thread_lock = threading.Lock()
	thread_lock_manual_mode = threading.Lock()
	thread_lock_camera_frame = threading.Lock()
	thread_lock_key = threading.Lock()

	tx_prev = 0
	uix_prev = 0
	ex_prev = 0
	uif_prev = 0
	ef_prev = 0
	uiy_prev = 0
	ey_prev = 0		
	
	# Create new threads
	manual_control_thread = manualControlThread(1, "Manual Control Thread")
	automatic_control_thread = automaticControlThread(2, "Automatic Control Thread")
	
	# Start new Threads
	manual_control_thread.start()
	automatic_control_thread.start()
	
	# Add threads to thread list
	threads.append(manual_control_thread)
	threads.append(automatic_control_thread)

	thread_lock.acquire()
	while not exiting:		
		thread_lock.release()

		# wait for pressed key
		thread_lock_key.acquire()
		key = cv2.waitKey(33)
		thread_lock_key.release()

		# display new frame from camera
		thread_lock_camera_frame.acquire()
		if new_frame:
			cv2.imshow('Drone',render_frame)	
		thread_lock_camera_frame.release()
		thread_lock.acquire()

	# Wait for all threads to complete
	for t in threads:
		t.join()
	print "Exiting Main Program"

if __name__ == '__main__':
	main()
