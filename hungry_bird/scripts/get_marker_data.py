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

		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		for i in range(0,1):
			pose=msg.poses[i]
			self.whycon_marker[i]=[round(pose.position.x,3),round(pose.position.y,3),round(pose.position.z,3)]
		print "WhyCon_marker",self.whycon_marker


	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		


		# Printing the detected markers on terminal
		for i in range(0,1):
			marker=msg.markers[i]
			self.aruco_marker[i]=[round(marker.pose.pose.orientation.x,3),round(marker.pose.pose.orientation.y,3),round(marker.pose.pose.orientation.z,3),round(marker.pose.pose.orientation.w,3)]
		print "ArUco_marker",self.aruco_marker
		print "\n"




if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()
