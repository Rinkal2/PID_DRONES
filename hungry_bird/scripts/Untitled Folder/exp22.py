#!/usr/bin/env python

#import numpy as np
#import tensorflow as tf
#import gym
import random
import copy
import math
import csv

#Ros packages
from plutodrone.msg import *
#from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
import rospy
import time

#rospy.init_node('pluto_fly', disable_signals = True)
rospy.init_node('drone_control')


#rate = rospy.Rate(120)

pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1) #rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		
cmd = PlutoMsg()

cmd.rcRoll = 1500
cmd.rcPitch = 1500
cmd.rcYaw = 1500
cmd.rcThrottle = 1000
cmd.rcAUX1 = 0
cmd.rcAUX2 = 0
cmd.rcAUX3 = 0
cmd.rcAUX4 = 1000
cmd.plutoIndex = 0

z_hold = 30


class Drone_state(object):
	
	def __init__(self):
		self.z=0.0
		self.drone_state = self.z
		self.done = False
		self.is_reset = False
		self.Q = dict()
		self.epsilon = 1.0
		self.epsilon_max = 1.0
		self.epsilon_min = 0.01
		self.alpha = 0.4
		self.gamma = 0.9


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle, yaw]
		self.Kp = [7.2,10.32,0,0]
		self.Ki = [0.024,0.016,0,0]
		self.Kd = [487.2,509.7,0,0]

        	
	
	def choose_action(self, state):
		if not(state in ls_states):
			ls_states.append(state)
			ls_max_val_act.append('u1500')
		idx = ls_states.index(state)
		action = ls_max_val_act[idx]
		return action	
		
	
	def set_drone_state(self, state):
		self.drone_state = state

	def get_drone_state(self):
		return self.drone_state

	def reset(self):
		#Ros environment reset
		reset_publish()
		return env.get_drone_state()   
	
        	
	def step(self, action):
		current_state = env.get_drone_state()
		print("current : ", current_state)
		if(action[0]=='u'):
			throttle(int(action[1:]))
			time.sleep(0.04)
		elif(action[0]=='d'):
			throttle(int(action[1:]))
			time.sleep(0.04)				
                		
		next_state = int(round(env.get_drone_state()))
		print("Next : ", next_state)
		reward = get_reward(next_state)
		print("reward : ", reward)
		
		if(next_state<22 or next_state>38): #or abs(xp)>7 or abs(yp)>6):
			env.done = True

		return next_state, reward, env.done	
        	

env = Drone_state()	
						
					        	

def get_reward(state):
	reward = 0.0
	if ((abs(state - z_hold) < 1)) :
		# goal reached 
		reward = 100
		
	elif (abs(state - z_hold) < 2) :
		reward = 25
		
	elif (abs(state - z_hold) < 3) :
		reward = 10
		
	elif (abs(state - z_hold) < 4) :
		reward = 5
	
	elif (abs(state - z_hold) < 5) :
		reward = 1
		
	elif (abs(state - z_hold) < 6) :
		reward = -10				
	
	elif(state<23 or state>37):
		reward = -100										
					
	else:
		reward = -50
	# if (state[0] < -0.05 or state[0] > 5.05 or state[1] < -0.05 or state[1] > 5.05):
	return reward	
	
	
	
def throttle(t):
	cmd.rc = t
	pluto_cmd.publish(cmd)

	
	
def reset_publish():
	cmd.rcAUX1=1800
	pluto_cmd.publish(cmd)
	#rospy.sleep(1)	
    	
    	
def cb_drone_state(pose):
	env.set_drone_state(pose.poses[0].position.z)   #set z value obtained in bracket
    
def subscriber():
	rospy.Subscriber('whycon/poses', PoseArray, cb_drone_state)


def arm():   # Arming condition of the drone

	cmd.rcRoll = 1500
	cmd.rcYaw = 1500
	cmd.rcPitch = 1500
	cmd.rcThrottle = 1000
	cmd.rcAUX4 = 1500
	pluto_cmd.publish(cmd)	# Publishing /drone_command
	print "Arm"
	rospy.sleep(0.1)

def disarm():
	cmd.rcThrottle = 1000
	cmd.rcAUX4 = 1000
	pluto_cmd.publish(cmd)


def roll_set_pid(self,roll):
	self.Kp[0] = roll.Kp * 0.06 
	self.Ki[0] = roll.Ki * 0.001
	self.Kd[0] = roll.Kd * 0.3


def pitch_set_pid(self,pitch):
	self.Kp[1] = pitch.Kp * 0.06 
	self.Ki[1] = pitch.Ki * 0.001
	self.Kd[1] = pitch.Kd * 0.3




num_episodes = 20000

subscriber()	   

ls_act=[]
ls_states=[]
ls_max_val_act=[]

with open('Qtable.csv','rt') as f:
	data = csv.reader(f)
	for row in data:
		ls_act = row
		ls_act=ls_act[1:]
		break
	i=0
	for row in data:
		ls_states.append(int(row[0]))
		ls_vals=row[1:]
		maxval=ls_vals[0]
		max_val_act=ls_act[0]
		for z in range(1,len(ls_vals)):
			if ls_vals[z]>maxval:
				maxval=ls_vals[z]
				max_val_act = ls_act[z]
		ls_max_val_act.append(max_val_act)
		#i+=1						
	f.close()
	
#print(ls_act)
print(ls_states)
print(ls_max_val_act)

time.sleep(2)
disarm()
rospy.sleep(0.1)
arm()


	def pid(self):
		while True:
			
			now = time.time()
			timechange = now-self.lastTime
			if timechange >= self.sample_time:

				self.errors[0] = self.setpoint[0] - self.drone_position[0]

				self.errors[1] = self.setpoint[1] - self.drone_position[1]
			
				self.errors[2] = self.setpoint[2] - self.drone_position[2]

				self.errsum[0] = (self.errsum[0] + self.errors[0])

				self.errsum[1] = (self.errsum[1] + self.errors[1])
			
				self.errsum[2] = self.errsum[2] + self.errors[2]

				self.pid_values[0] = (self.Kp[0] * self.errors[0]) + (self.Ki[0] * self.errsum[0]) + (self.Kd[0] * (self.errors[0] - self.prev_errors[0]))

				self.pid_values[1] = (self.Kp[1] * self.errors[1]) + (self.Ki[1] * self.errsum[1]) + (self.Kd[1] * (self.errors[1] - self.prev_errors[1]))
			
				#self.pid_values[2] = (self.Kp[2] * self.errors[2]) + (self.Ki[2] * self.errsum[2]) + (self.Kd[2] * (self.errors[2] - self.prev_errors[2]))

				#self.cmd.rcThrottle = 1500 - self.pid_values[2]

				self.cmd.rcRoll = 1500 + self.pid_values[0]

				self.cmd.rcPitch = 1500 - self.pid_values[1]
			
				

				if self.cmd.rcRoll > self.max_values[0]:
					self.cmd.rcRoll = self.max_values[0]

				if self.cmd.rcPitch > self.max_values[1]:
					self.cmd.rcPitch = self.max_values[1]
			
				#if self.cmd.rcThrottle > self.max_values[2]:
					#self.cmd.rcThrottle = self.max_values[2]

				if self.cmd.rcRoll < self.min_values[0]:
					self.cmd.rcRoll = self.min_values[0]

				if self.cmd.rcPitch < self.min_values[1]:
					self.cmd.rcPitch = self.min_values[1]
				
				#if self.cmd.rcThrottle < self.min_values[2]:
					#self.cmd.rcThrottle = self.min_values[2]
				
				self.command_pub.publish(self.cmd)

				self.roll_error.publish(self.errors[0])

				self.pitch_error.publish(self.errors[1])
			
				#self.alt_error.publish(self.errors[2])
			
				#self.zero_line.publish(zero)
			
				#self.fifteen.publish(fif)

				self.prev_errors[0] = self.errors[0]

				self.prev_errors[1] = self.errors[1]
			
				self.prev_errors[2] = self.errors[2]
			
				self.lastTime = now
				
				print "PID"

				if self.key_value == 0:         
					self.disarm()

			if self.key_value == 0:         
				self.disarm()
	

while not rospy.is_shutdown():
	memory_states = []
	memory_targets = []
	#validation for 100 episodes
	for _ in range(0,100):
		#state = int(round(env.reset()))
		state=30
		rospy.sleep(1)
		print(state)
		#cmd.rcAUX1=1500
		env.done = False
		total_reward = 0
		while env.done == False:	
			action = env.choose_action(state)
			env.epsilon = 0.00001
                    	#take an e-greedy action
			next_state, reward, done = env.step(action)
			total_reward += reward
			state = next_state	
		print ("reward in episode ",_," is: ",total_reward) 


