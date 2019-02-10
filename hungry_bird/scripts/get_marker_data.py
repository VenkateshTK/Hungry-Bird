#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Float64

#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}

			# Subscribing to topic
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)# Subscribing to topic
		# Printing the detected markers on terminal
		self.alt_pub = rospy.Publisher('/alt_error', Float64, queue_size=10)

    	def whycon_callback(self,msg):
			self.alt_pub.publish(msg.poses[0].position.z-msg.poses[1].position.z)


if __name__=="__main__":

	marker = Marker_detect()
	while not rospy.is_shutdown():
		rospy.spin()
