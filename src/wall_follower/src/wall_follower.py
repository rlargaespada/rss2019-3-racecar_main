#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import warnings
warnings.simplefilter('ignore', np.RankWarning)

class WallFollower:
	# Import ROS parameters from the "params.yaml" file.
	# Access these variables in class functions with self:
	# i.e. self.CONSTANT
	SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
	DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
	SIDE = rospy.get_param("wall_follower/side")
	VELOCITY = rospy.get_param("wall_follower/velocity")
	DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
	

	def __init__(self):
		# TODO:
		# Initialize your publishers and
		# subscribers here
		self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback, queue_size=10)
		self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
		self.pub_line = rospy.Publisher("marker",Marker,queue_size=10)
		self.rate = 50 #laser scan messages rate [hz]


	# TODO:
	# Write your callback functions here.
	def callback(self,data):
		A = AckermannDriveStamped()
		
		#get laser scan data
		dat = data.ranges #list of distances from laser scanner
		angs = self.a_trans(data) #make list of angles from min to max

		r,th = self.data_split(dat,angs) #split data into polar coordinates (radius, theta)
		x,y = self.pol2cart(r,th) #transform coordinates to cartesian (x,y)
	
		if len(x)==0:
			#sets drive conditions if robot out of range
			A.drive.steering_angle = 0 #sets steering angle [rad]
			self.pub.publish(A) #publish steering command
			return

		m,b = np.polyfit(x,y,1) #fit linear regression line to data

		
		mark = self.make_marker(m,b,x) #generate marker message
		self.pub_line.publish(mark) #publish marker




		A.drive.speed = self.VELOCITY #sets velocity [m/s]
		u = self.controller(m,b,x) * self.SIDE #sets input steering angle from controller [rad]

		A.drive.steering_angle = u #determines input steering control
		A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]

		self.pub.publish(A) #publish steering command


	def controller(self,m,b,x):
		#combined proportional-pure persuit controller with Ackermann steering
		L = .324 #length of wheel base [m]
		a = np.arctan(m) #angle between velocity vector and desired path [rad]
		l = .5*self.VELOCITY #looh ahead dist [m]
		u_pp = self.SIDE*np.arctan(2*L*np.sin(a)/l) #control input to exactly match wall angle [rad]
		

		dist=np.amin(np.sqrt(x**2+(m*x+b)**2)) #perpenducular dist between car and closest point on wall [m]
		err = dist-self.DESIRED_DISTANCE #error from desired path [m]

		kp = 1.5 #proportional constant
		u_p = kp*err #control input to maintain desired distance from wall [rad]

		u = u_pp + u_p #total control input [rad]

		return u

	def data_split(self, dat,angs):
		#splits data into corner detecting sector
		#start/stop angles and max range are parameterized
		d = []
		a = []
		stop_angle = np.pi/8 #past-vertical ray
		start_angle = np.pi/2 #horizontal starting ray
		max_r = 3.5 #farthest distance to collect
		for n in range(len(dat)):
			if dat[n]<max_r:
				if self.SIDE>0:
					#left wall
					if -stop_angle<=angs[n]<=start_angle:
						a.append(angs[n])
						d.append(dat[n])
				
				else:
					#right wall
					if -start_angle<=angs[n]<=stop_angle:
						a.append(angs[n])
						d.append(dat[n])

		return np.array(d), np.array(a)


	def a_trans(self,data):
		#returns [list] of angles within range
		amin = data.angle_min #min angle [rad]
		amax = data.angle_max #max angle [rad]
		ainc = data.angle_increment #min angle increment [rad]
		angs = [amin]
		for i in range(len(data.ranges)):
			angs.append(angs[i]+ainc)
		return angs

	def pol2cart(self,r,th):
		#returns (x,y) in cartesian coordinate from (r,theta) polar coordinate
		#able to be run on vectors
		return r*np.cos(th), r*np.sin(th)

	def cart2pol(self,x,y):
		#returns (r,theta) pair in polar coordinates from (x,y) cartesian coordinate
		return np.sqrt(x**2+y**2), np.arctan2(y,x)



	def make_marker(self,slope,b,x):
		#generates marker message
		m = Marker()
		p1 = Point()
		p1.x=0
		p1.y=slope*np.amin(x)+b
		p1.z=0
		p2 = Point()
		p2.x=np.amax(x)
		p2.y=slope*np.amax(x)+b
		p2.z=0
		m.header.frame_id="base_link"
		m.action=0
		m.pose.orientation.w=1
		m.type=Marker.LINE_STRIP
		m.scale.x=.1
		m.color.g=1
		m.color.a=1
		m.points=[p1,p2]
		return m
		



if __name__ == "__main__":
	rospy.init_node('wall_follower')
	wall_follower = WallFollower()
	rospy.spin()
