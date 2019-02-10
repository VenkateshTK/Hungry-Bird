#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.initial_setpoint = [5.6625,-1.8875,32.8375,0.01]
		self.setpoint = []
		for i in range(50):
			ea_row = []
			for j in range(4):
				ea_row.append(0)
			self.setpoint.append(ea_row)
	
		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 0
		self.cmd.rcAUX2 = 0
		self.cmd.rcAUX3 = 0
		self.cmd.rcAUX4 = 1500
		self.cmd.plutoIndex = 0

		self.mode = 0
		self.path = 1


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [0,0,0,0]
		self.Ki = [0,0,0,0]
		self.Kd = [0,0,0,0]

		self._MSG_INDEX=0
		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.last_error=[0,0,0,0]
		self.error=[0,0,0,0]
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]
		self.error_sum=[0,0,0,0]
		self.out=[0,0,0,0]



		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 






		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=3)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.zero_pub=rospy.Publisher('/zero',Float64,queue_size=5)
		self.alt_pub = rospy.Publisher('/alt_error', Float64, queue_size=10)

		self.pitch_pub = rospy.Publisher('/pitch_error', Float64, queue_size=10)
		self.roll_pub = rospy.Publisher('/roll_error', Float64, queue_size=10)
		self.yaw_pub = rospy.Publisher('/yaw_error', Float64, queue_size=10)
		self.vrep_msg = rospy.Publisher('/message_vrep', String, queue_size=1)


		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/vrep/waypoints', PoseArray, self.waypoints)





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):
		print "Hi"
		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		self.set_value()	# Publishing /drone_command
		rospy.sleep(10)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

	def waypoints(self,msg):
		for i in range(0,49):
			self.setpoint[i][0] = msg.poses[i].position.x
			self.setpoint[i][1] = msg.poses[i].position.y
			self.setpoint[i][2] = msg.poses[i].position.z
			print self.setpoint[i][0]/-7.55,self.setpoint[i][1]/-7.55,self.setpoint[i][2],self.setpoint[i][3]
		

		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def set_value(self):
		self.Kp[2] = 288 * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = 0 * 0.008
		self.Kd[2] = 1015 * 0.3

	#----------------------------D
		self.Kp[0] = 192 * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[0] = 17 * 0.008
		self.Kd[0] = 951 * 0.3

	
		self.Kp[1] = 203 * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[1] = 0 * 0.008
		self.Kd[1] = 641 * 0.3


		self.Kp[3] = 0 * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[3] = 0 * 0.008
		self.Kd[3] = 0 * 0.3


	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
		if self.mode == 0:
			for i in range(0,3):
				self.error[i]=self.drone_position[i] - self.initial_setpoint[i]
		 	for i in range(0,3):
				self.out[i]= self.Kp[i]*self.error[i]+self.Kd[i]*(self.error[i]-self.last_error[i])+self.Ki[i]*self.error_sum[i]
			
			self.cmd.rcPitch = 1500+self.out[0] 
			self.cmd.rcRoll = 1500+self.out[1]
			self.cmd.rcYaw=1500
			self.cmd.rcThrottle = 1500+self.out[2] 
			
			rospy.sleep(self.sample_time)
			
			if self.cmd.rcPitch > self.max_values[0] :
				self.cmd.rcPitch=1800

			if self.cmd.rcPitch < self.min_values[0] :
				self.cmd.rcPitch=1200

			if self.cmd.rcRoll > self.max_values[1] :
				self.cmd.rcRoll=1800

			if self.cmd.rcRoll < self.min_values[1] :
				self.cmd.rcRoll=1200

			if self.cmd.rcThrottle > self.max_values[2] :
				self.cmd.rcThrottle=1800	

		    	if self.cmd.rcThrottle < self.min_values[2] :
				self.cmd.rcThrottle=1200


			for i in range(0,3):
				self.last_error[i]=self.error[i]
				self.error_sum[i]+=self.error[i]

			if (self.error[2] <2) :
			 	self.error_sum[2]=0	
			if (self.error[0] <2) :
				self.error_sum[0]=0	
			if (self.error[1] <2) :
				self.error_sum[1]=0	
		#------------------------------------------------------------------------------------------------------------------------
			#rospy.sleep(self.sample_time)
			self.pitch_pub.publish(self.error[0])
			#rospy.sleep(self.sample_time)
			self.roll_pub.publish(self.error[1])
			#rospy.sleep(self.sample_time)
			self.alt_pub.publish(self.error[2])
			#rospy.sleep(self.sample_time)
			self.yaw_pub.publish(self.error[3])
			#rospy.sleep(self.sample_time)
			self.zero_pub.publish(0)
			self.command_pub.publish(self.cmd)

			if self.error[i] < 0.5:
				self.mode = 1
				j = 0
				self.vrep_msg.publish(self.path)

		if self.mode == 1:

			for i in range(0,3):
				self.error[i]=self.drone_position[i] - self.setpoint[j][i]
		 	for i in range(0,3):
				self.out[i]= self.Kp[i]*self.error[i]+self.Kd[i]*(self.error[i]-self.last_error[i])+self.Ki[i]*self.error_sum[i]
			
			self.cmd.rcPitch = 1500+self.out[0] 
			self.cmd.rcRoll = 1500+self.out[1]
			self.cmd.rcYaw=1500
			self.cmd.rcThrottle = 1500+self.out[2] 
			
			rospy.sleep(self.sample_time)
			
			if self.cmd.rcPitch > self.max_values[0] :
				self.cmd.rcPitch=1800

			if self.cmd.rcPitch < self.min_values[0] :
				self.cmd.rcPitch=1200

			if self.cmd.rcRoll > self.max_values[1] :
				self.cmd.rcRoll=1800

			if self.cmd.rcRoll < self.min_values[1] :
				self.cmd.rcRoll=1200

			if self.cmd.rcThrottle > self.max_values[2] :
				self.cmd.rcThrottle=1800	

		    	if self.cmd.rcThrottle < self.min_values[2] :
				self.cmd.rcThrottle=1200


			for i in range(0,3):
				self.last_error[i]=self.error[i]
				self.error_sum[i]+=self.error[i]

			if (self.error[2] <2) :
			 	self.error_sum[2]=0	
			if (self.error[0] <2) :
				self.error_sum[0]=0	
			if (self.error[1] <2) :
				self.error_sum[1]=0	
		#------------------------------------------------------------------------------------------------------------------------
			#rospy.sleep(self.sample_time)
			self.pitch_pub.publish(self.error[0])
			#rospy.sleep(self.sample_time)
			self.roll_pub.publish(self.error[1])
			#rospy.sleep(self.sample_time)
			self.alt_pub.publish(self.error[2])
			#rospy.sleep(self.sample_time)
			self.yaw_pub.publish(self.error[3])
			#rospy.sleep(self.sample_time)
			self.zero_pub.publish(0)
			self.command_pub.publish(self.cmd)

			if error[i] < 0.5:
				self.mode = 1
				j = j+1
				if j == 49:
					j = 0
					self.path = self.path+1
					if self.path < 4:
						self.vrep_msg.publish(self.path)
					else:
						self.mode = 2

		if self.mode == 2:
			disarm()



if __name__ == '__main__':

	e_drone = Edrone()
	while not rospy.is_shutdown():
		e_drone.pid()
