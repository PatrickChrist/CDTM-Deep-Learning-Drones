# -*- coding: utf-8 -*-
"""
Created on Wed Aug 10 11:19:19 2016

@author: Christian
"""
import time

class PID_Controller(object):

    
    k_p=1
    k_i=1
    k_d=1
    
    sum_value_difference=0
    old_value=-1
    old_time=-1
    kind_of_controller='Default'
    def __init__(self,k_p,k_i,k_d,kind_of_controller):
        self.k_p=k_p
        self.k_i=k_i
        self.k_d=k_d
        self.kind_of_controller=kind_of_controller

    def pidControl(self,desiredValue, actualValue):
        valueDifferenz= desiredValue - actualValue
        
        y_p=self.k_p*valueDifferenz
        
        self.sum_value_difference+=valueDifferenz
        y_i=self.k_i*self.sum_value_difference
        
        if self.old_value==-1:
            y_d = 0
            self.old_value = actualValue
            self.old_time = int(round(time.time() * 1000/250))  #time in number of frames 
        else:
            actual_time = int(round(time.time() * 1000/250))
            y_d = self.k_d*(self.old_value-actualValue)
            self.old_value = actualValue
            self.old_time = actual_time
        
        return y_p+y_i+y_d
    
    