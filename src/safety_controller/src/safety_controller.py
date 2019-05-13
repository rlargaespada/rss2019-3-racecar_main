#!/usr/bin/env python2

import numpy as np
import math
import rospy
import scipy
from sklearn.metrics import mean_squared_error
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/scan"
    DRIVE_TOPIC = "/drive"
    VELOCITY = 1

    def __init__(self):
        # TODO:
        self.pub_line = rospy.Publisher("marker",Marker,queue_size=10)
        self.out = AckermannDriveStamped()
        # self.create_message(self.VELOCITY)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        #Initialize necessary arrays for left side of scan
        self.collect_iteration = 5                  #in ticks
        #Tracks how many iterations of scans have occured
        self.track_iteration = 0.                   #in ticks
        #multiple of distance changes away an object must be to trigger stopping
        self.crash_buffer = 3                       #in m
        #Should I stop?
        self.stop = False
        #Contains the parameters of the last colision
        self.stop_data = {"side": "", "dist": 0, "ang" : 0}
        self.data = np.array([])

        self.input_ang = 0
        self.input_speed = 0
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        rospy.Subscriber("/dummy", AckermannDriveStamped, self.drive_callback)


    def callback(self, scan):
        self.data = np.array(scan.ranges)
        self.center = len(scan.ranges)/2
        self.angs = self.a_trans(scan)
        # self.decide_stop()

    def create_message(self, v, ang):
        """create optput AckermannDriveStamped message
        """
        self.out.header.stamp = rospy.Time.now()
        self.out.header.frame_id = "1"
        self.out.drive.steering_angle = ang
        self.out.drive.steering_angle_velocity = 0
        self.out.drive.speed = v
        self.out.drive.acceleration = 0
        self.out.drive.jerk = 0
    
    def check_crash(self, distances, angles):
        #Iterate through average changes, check if change indicates that next position would be in wall
        minimum = np.argmin(distances)
        # x, y = self.pol_to_cart(distances[minimum], angles[minimum])
        while self.pol_to_cart(self.data[minimum], self.angs[minimum])[0] < max(1., self.input_speed*.4)*2 and abs(self.pol_to_cart(self.data[minimum], self.angs[minimum])[1]) < .2:
            print(distances[minimum])
            rate = rospy.Rate(20)
            # print(j)
            for i in range(1):
                if angles[minimum] < 0:
                    self.create_message(min(self.input_speed, 2), 0.34)
                else:
                    self.create_message(min(self.input_speed, 2),-0.34)
                self.pub.publish(self.out)
                rate.sleep()

    def get_clearance(self, scan, dist):
        """
        Input: scan: np array of laser scan distances
               dist: threshold distance
        Output: clearance: proportion (between 0 and 1) of scans in sect that
                           are greater than the threshold distance
        """
        filtered = scan[scan > dist]
        clearance = float(len(filtered))/len(scan)

        return clearance


    def a_trans(self,data):
	# returns [list] of angles within range
    	amin = data.angle_min # min angle [rad]
    	amax = data.angle_max # max angle [rad]
    	ainc = data.angle_increment # min angle increment [rad]
    	angs = [amin]
    	for i in range(len(data.ranges)):
    		angs.append(angs[i]+ainc)
        return angs

    def drive_callback(self, data):
        self.input_ang = data.drive.steering_angle
        self.input_speed = data.drive.speed
        #Check for a crash at each of the regions 
        self.check_crash(self.data, self.angs)
        #Iterate and publish
        self.track_iteration += 1
        self.create_message(self.input_speed, self.input_ang)
        self.pub.publish(self.out)

    def pol_to_cart(self, r, th):
        #Convert a polar point to cartesian point
        return r*np.cos(th), r*np.sin(th)

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    wall_follower = WallFollower()
    rospy.spin()



