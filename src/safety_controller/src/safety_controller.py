#!/usr/bin/env python2

import numpy as np
import math
import rospy
import scipy
# from sklearn.metrics import mean_squared_error
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/scan"
    DRIVE_TOPIC = "/drive"
    VELOCITY = 1.

    def __init__(self):
        # TODO:
        self.out = AckermannDriveStamped()
        self.create_message(self.VELOCITY)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        #Initialize necessary arrays for left side of scan
        self.left_average_changes = np.array([])    #in m/($collect_iteration$ ticks)
        self.left_last_data = np.array([])          #in m
        self.left_new_data = np.array([])           #in m
        #Initialize necessary arrays for center of scan
        self.center_average_changes = np.array([])  #in m/($collect_iteration$ ticks)
        self.center_last_data = np.array([])        #in m
        self.center_new_data = np.array([])         #in m
        #Initialize necessary arrays for cetner of scan
        self.right_average_changes = np.array([])   #in m/($collect_iteration$ ticks)
        self.right_last_data = np.array([])         #in m
        self.right_new_data = np.array([])          #in m
        #Number of ticks between next evaluation of crashing
        self.collect_iteration = 5                  #in ticks
        #Tracks how many iterations of scans have occured
        self.track_iteration = 0.                   #in ticks
        #multiple of distance changes away an object must be to trigger stopping
        self.crash_buffer = 3                       #in m
        #Should I stop?
        self.stop = False
        #Contains the parameters of the last colision
        self.stop_data = {"side": "", "dist": 0, "ang" : 0}
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        pass

    def callback(self, scan):
        data = np.array(scan.ranges)
        self.center = len(scan.ranges)/2
        self.angs = self.a_trans(scan)
        #Get data from the left, cetner and right areas of the scan
        self.left_new_data, self.center_new_data, self.right_new_data = self.data_split(data, self.angs)
        #Keep track of the last iteration of distances and the average changes given the new data
        self.left_last_data, self.left_average_changes = self.distances(self.left_new_data[0], self.left_last_data, self.left_average_changes)
        self.center_last_data, self.center_average_changes = self.distances(self.center_new_data[0], self.center_last_data, self.center_average_changes)
        self.right_last_data, self.right_average_changes = self.distances(self.right_new_data[0], self.right_last_data, self.right_average_changes)
        #Check for a crash at each of the regions 
        self.check_crash(self.left_new_data, self.left_last_data, 0.15, "left")
        self.check_crash(self.center_new_data, self.center_average_changes, 0.1, "center")
        self.check_crash(self.right_new_data, self.right_average_changes, 0.15, "right")
        #Iterate and publish
        self.track_iteration += 1
        if self.stop:
            self.pub.publish(self.out)
        # self.decide_stop()

    def create_message(self, v):
        """create optput AckermannDriveStamped mssage
        """
        self.out.header.stamp = rospy.Time.now()
        self.out.header.frame_id = "1"
        self.out.drive.steering_angle = 0
        self.out.drive.steering_angle_velocity = 0
        self.out.drive.speed = 0
        self.out.drive.acceleration = 0
        self.out.drive.jerk = 0

    def distances(self, new_data, last_data, average_changes):
        #If first round initialize new_data, last_data, and average_changes with correct sizes
        if self.track_iteration == 0:
            last_data = new_data
            average_changes = np.zeros(len(new_data))
        
        #If it's time for another distance data collection, average each set of 3 points of data and find changes in distances
        if self.track_iteration % self.collect_iteration == 0:
            #Weight points farther from center as higher to account for angled crash
            average_changes = (last_data - new_data)
            last_data = new_data

        return last_data, average_changes
    
    def check_crash(self, new_data, average_changes, off_set, side_indicator):
        #Iterate through average changes, check if change indicates that next position would be in wall
        for i in range(len(average_changes)):
            #Make 3 checks: 1)check if change indicates that next position would be in wall
            #               2)check if the distance from the scan detected is <1m
            #               3)Check if the point causing the stop is actually in front of the car
            if average_changes[i]*self.crash_buffer + off_set > new_data[0][i] and new_data[0][i] < 1 and abs(self.pol_to_cart(new_data[0][i], new_data[1][i])[1]) < off_set + .1:
                rospy.loginfo((side_indicator, len(new_data[0]), i, self.pol_to_cart(new_data[0][i], new_data[1][i])))
                self.out.drive.speed = 0
                self.stop = True
                #Remember params of collision so can return to driving when safe
                self.stop_data["side"] = side_indicator
                self.stop_data["dist"] = new_data[0][i]
                self.stop_data["ang"] = i

    def a_trans(self,data):
	#returns [list] of angles within range
	amin = data.angle_min #min angle [rad]
	amax = data.angle_max #max angle [rad]
	ainc = data.angle_increment #min angle increment [rad]
	angs = [amin]
	for i in range(len(data.ranges)):
		angs.append(angs[i]+ainc)
        return angs


    def data_split(self, dat,angs):
		#splits data into corner detecting sector
		#start/stop angles and max range are parameterized
        left = [[], []]
        center = [[], []]
        right = [[], []]
        start_ang = -np.pi/3 #start of right wheel
        ang_1 = -np.pi/6 #right wheel to center
        ang_2 = np.pi/6 #center to left wheel
        stop_ang = np.pi/3 #end of left wheel
        max_r = 3.5 #farthest distance to collect
        for n in range(len(dat)):
            #left wheel

            if start_ang<=angs[n]<=ang_1:
                left[0].append(dat[n])
                left[1].append(angs[n])

            #center
            if ang_1<=angs[n]<=ang_2:
                center[0].append(dat[n])
                center[1].append(angs[n])

        #right wheel
            if ang_2<= angs[n]<=stop_ang:
                right[0].append(dat[n])
                right[1].append(angs[n])
        return [np.array(left[0]), np.array(left[1])], [np.array(center[0]), np.array(center[1])], [np.array(right[0]), np.array(right[1])]
    
    def pol_to_cart(self, r, th):
        #Convert a polar point to cartesian point
        return r*np.cos(th), r*np.sin(th)

    def decide_stop(self):
        #Create new_dist variable with the new distance corrosponding to the same scan point that caused the stop
        if self.stop_data["side"] == "left":
            now_dist = self.left_new_data[0][self.stop_data["ang"]]
        if self.stop_data["side"] == "right":
            now_dist = self.right_new_data[0][self.stop_data["ang"]]
        if self.stop_data["side"] == "center":
            now_dist = self.center_new_data[0][self.stop_data["ang"]]
        #Check if we have stopped yet
        if self.stop_data["side"] != "":
            rospy.logout((self.stop_data["side"], self.stop_data["ang"], self.stop_data["dist"], now_dist))
           #Check if the distance of that was cause for stop is still close
	    if now_dist - .1 >self.stop_data["dist"]:
                rospy.logout("still driving")
                self.stop = False

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    wall_follower = WallFollower()
    rospy.spin()




