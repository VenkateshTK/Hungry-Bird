#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}

			# Subscribing to topic
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		# Printing the detected markers on terminal
	
    	def whycon_callback(self,msg):
		quaternion =(msg.poses[0].orientation.x,msg.poses[0].orientation.y,msg.poses[0].orientation.z,msg.poses[0].orientation.w)
    		euler = tf.transformations.euler_from_quaternion(quaternion)
    		roll = euler[0]
    		pitch = euler[1]
    		yaw = euler[2]
    		print (" roll =") 
    		print(roll)
    		print ('pitch=')  
    		print(pitch)
    		print ('yaw=')    
    		print(yaw)




if __name__=="__main__":

	marker = Marker_detect()
	while not rospy.is_shutdown():
		rospy.spin()
