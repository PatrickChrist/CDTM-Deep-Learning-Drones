# -*- coding: utf-8 -*-
"""
Created on Wed Aug 10 11:48:48 2016

@author: Christian
"""
from PIDController import PID_Controller
from libardrone import libardrone
import patternRecognition 
import time
import cv2
import numpy as np

class CentralControl(object):
    # Controller parameter
    x_PIDController=PID_Controller(1,0.0,2.5,"xController")
    y_PIDController=PID_Controller(1.5,0.0,2.25,"yController")
    bf_PIDController=PID_Controller(1,0.0,3,"bfController")
    speedRange = [0.45,0.15,0.375,0.1]  #turn x, move y, go forward, move x      ## Stable values [0.25,0.15,0.2,0.05]
    maxPIDValue = [200,200,200,200]
    x_offset=0.003
    
    
    def __init__(self,standardXCoord,standardYCoord,standardSize):
        self.standardXCoord=standardXCoord
        self.standardYCoord=standardYCoord
        self.standardSize=standardSize
        
    
    #Image Recognition returns left upper corner coordinates and right downer corner coordinates
    
    def computeSize(self,xCoordinate_leftUpperCorner,yCoordinate_leftUpperCorner,xCoordinate_rightDownerCorner,yCoordinate_rightDownerCorner):
        return ((xCoordinate_leftUpperCorner-xCoordinate_rightDownerCorner)**2+(yCoordinate_leftUpperCorner-yCoordinate_rightDownerCorner)**2)**0.5
    
    def reciprocalSize(self,desiredValue,actualValue):
        if actualValue<desiredValue:
            return 2*desiredValue-(desiredValue**2)/actualValue
        else:
            return actualValue
    
    def controlLoop(self):
        drone=libardrone.ARDrone(True)        
        drone.reset()

        frame=drone.get_image()
        cv2.imshow('img',frame)
        #Wait for any key to start over
        cv2.waitKey(0)
        
        drone.takeoff()
        print "Takeoff"
        
        logFilePIDPath="logFilePID_11.csv"
        logFilePID=open(logFilePIDPath,"a")
        logFileCmdPath="logFileCmd_11.csv"
        logFileCmd=open(logFileCmdPath,"a")

        logFilePID.write("\n\n=================================================================================\n")
        logFileCmd.write("\n\n=================================================================================\n")
        
        running=True

        while running:
            key=cv2.waitKey(5)
            if key==32:
                print "Land drone"
                running=False
                drone.land()
            
            frame=drone.get_image()
            
            # call imageRec
            xlu,ylu,xrd,yrd=patternRecognition.cornerPointsChess(frame,logFileCmd)
            if not(xlu==-1 and ylu==-1 and xrd==-1 and yrd==-1):    
                # computeSize
                currentsize=self.computeSize(xlu,ylu,xrd,yrd)
                recipSize = self.reciprocalSize(self.standardSize,currentsize)
                xAvg = (xlu+xrd)/2.0
                yAvg = (ylu+yrd)/2.0
                # call PIDController
                x_PIDValue=self.x_PIDController.pidControl(self.standardXCoord,xAvg)
                y_PIDValue=self.y_PIDController.pidControl(self.standardYCoord,yAvg)
                bf_PIDValue=self.bf_PIDController.pidControl(self.standardSize,recipSize)
                #log-file entries
                self.logFileWrite(logFileCmd,"x_PID: "+str(x_PIDValue))
                self.logFileWrite(logFileCmd,"y_PID: "+str(y_PIDValue))
                self.logFileWrite(logFileCmd,"bf_PID: "+str(bf_PIDValue))              
                self.logFileWrite(logFilePID,str(x_PIDValue)+","+str(y_PIDValue)+","+str(bf_PIDValue))
                # Actuate                 
                xSpeed,ySpeed,bfSpeed,x2Speed = self.calcSpeed(x_PIDValue,y_PIDValue,bf_PIDValue)
                self.actuateAll(x2Speed,xSpeed,ySpeed,bfSpeed,drone)
            else:
                drone.hover()
                pass
            
            time.sleep(0.01)
            
        #Close log-files
        logFilePID.close    
        logFileCmd.close  
        print "Drone landed"     
         
        print("Shutting down...")
        drone.halt()
        print("Ok.")
            

    def calcSpeed(self,x_PIDValue,y_PIDValue,bf_PIDValue):
        # x-speed: change sign PIDValue>0 <=> speed<0
        if abs(x_PIDValue)>self.maxPIDValue[0]:        
            xSpeed=-np.sign(x_PIDValue)*self.speedRange[0]
        else:
            xSpeed=-x_PIDValue/self.maxPIDValue[0]*self.speedRange[0]
        # y-speed: keep sign PIDValue>0 <=> speed>0
        if abs(y_PIDValue)>self.maxPIDValue[1]:        
            ySpeed=np.sign(y_PIDValue)*self.speedRange[1]
        else:
            ySpeed=y_PIDValue/self.maxPIDValue[1]*self.speedRange[1]
        # bf-speed: change sign PIDValue>0 <=> speed<0
        if abs(bf_PIDValue)>self.maxPIDValue[2]:        
            bfSpeed=-np.sign(bf_PIDValue)*self.speedRange[2]
        else:
            bfSpeed=-bf_PIDValue/self.maxPIDValue[2]*self.speedRange[2]
        # x-speed: change sign PIDValue>0 <=> speed<0
        if abs(x_PIDValue)>self.maxPIDValue[3]:        
            x2Speed=-np.sign(x_PIDValue)*self.speedRange[3]
        else:
            x2Speed=-x_PIDValue/self.maxPIDValue[3]*self.speedRange[3]
        return xSpeed,ySpeed,bfSpeed,x2Speed
        
        
    def actuateAll(self,x2Speed,xSpeed,ySpeed,bfSpeed,drone):
        drone.at(libardrone.at_pcmd, True, x2Speed+self.x_offset, bfSpeed, ySpeed, xSpeed)
       
    def logFileWrite(self,file,msg):
        pass
#        file.write("%s,%s\n" % (str(time.time()), msg))
        


control=CentralControl(320,180,80)     #Parameter of ideal position (x,y and z direction [px])
control.controlLoop()
        
        
    
