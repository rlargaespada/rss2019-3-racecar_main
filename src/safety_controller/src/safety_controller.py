#!/usr/bin/env python2

import numpy as np
import math
import rospy
import scipy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/scan"
    DRIVE_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"
    VELOCITY = 2.
    LENGTH = 5 * 2.54

    def __init__(self):
        # TODO:
        self.out = AckermannDriveStamped()
        self.create_message(self.VELOCITY)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.average_changes = np.array([])
        self.last_data = np.array([])
        self.new_data = np.array([])
        self.scan_hz = 50.
        self.collect_iteration = 5
        self.track_iteration = 0.
        self.crash_buffer = 1
        self.mse_bound = 2
        self.center = 0
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        pass

    def callback(self, scan):
        data = np.array(scan.ranges)
        self.center = len(scan.ranges)/2
        self.distances(data[self.center - 50:self.center + 50])
        self.check_crash(data[self.center - 50:self.center + 50], scan.angle_increment)
        self.pub.publish(self.out)

    def create_message(self, v):
        """create optput AckermannDriveStamped mssage
        """
        self.out.header.stamp = rospy.Time.now()
        self.out.header.frame_id = "1"
        self.out.drive.steering_angle = 0
        self.out.drive.steering_angle_velocity = 0
        self.out.drive.speed = v
        self.out.drive.acceleration = 0
        self.out.drive.jerk = 0

    def distances(self, data):
        #If first round initialize new_data, last_data, and average_changes with correct sizes
        if self.track_iteration == 0:
            self.new_data = np.array([np.average(data[i-1:i +1]) for i in range(1, len(data) -1)])
            self.last_data = self.new_data
            self.average_changes = np.zeros(len(self.new_data))
        
        #If it's time for another distance data collection, average each set of 3 points of data and find changes in distances
        if self.track_iteration % self.collect_iteration == 0:
            self.new_data = np.array([np.average(data[i-1:i +1]) for i in range(1, len(data) -1)])
            #Weight points farther from center as higher to account for angled crash
            self.average_changes = (self.last_data - self.new_data)+np.array([abs(i - len(self.new_data)/2)*.01 for i in range(len(self.new_data))])
            self.last_data = self.new_data

        self.track_iteration += 1
    
    def check_crash(self, data, ang_inc):
        #Iterate through average changes, check if change indicates that next position would be in wall
        for i in range(len(self.average_changes)):
            if self.average_changes[i]*self.crash_buffer > self.new_data[i] and (self.new_data[i] < 1):
                #Check if there is obstacle straight ahead...
                if abs(i - len(self.new_data)/2) < 7:
                    rospy.loginfo(("IN FRONT", i, self.average_changes[i], self.new_data[i]))
                    self.out.drive.speed = 0
                #Or if the pattern from point to center is approx straight line
                elif self.check_straight(self.get_points_for_regression(data, len(self.new_data)/2, i), ang_inc) < self.mse_bound: 
                    rospy.loginfo(("GOT HERE", i, self.check_straight(self.get_points_for_regression(data, len(self.new_data)/2, i), ang_inc)))
                    self.out.drive.speed = 0


    def check_straight(self, data, ang_inc):
        #Change data to cartesian and return MSE
        cart_data = self.polar_to_cartesian(data, ang_inc)
        # rospy.loginfo(mean_squared_error(cart_data[0], cart_data[1]))
        return np.square(np.array(cart_data[0])-np.array( cart_data[1])).mean(axis=None)

    def polar_to_cartesian(self, data, ang_inc):
        #Change data to cartesian
        final = [[],[]]
        center = len(data)/2
        for i in range(len(data)):
            final[0].append( (math.cos((i - center)*ang_inc)*data[i]) )
            final[1].append( (math.sin((i - center)*ang_inc)*data[i]) )
        return final

    def get_points_for_regression(self, data, center, end):
        rospy.loginfo((center, end))
        if center > end:
            return data[max(0, end - 4):center]
        if end > center:
            return data[center: min(end + 4, len(data) - 1)]

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    wall_follower = WallFollower()
    rospy.spin()




    # def __init__(self):
    #     # TODO:
    #     self.out = AckermannDriveStamped()
    #     self.create_message(self.VELOCITY)
    #     self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
    #     self.iter_check = 5
    #     self.iterate = 0
    #     rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
    #     pass

    # def callback(self, scan):
    #     """Use PID control to create desired angle with 
    #     wall and Publish output of PID control.
    #     """
    #     data = np.array(scan.ranges)
    #     center = len(data)/2
    #     angle_inc = scan.angle_inc
    #     if iterate = 5:
    #         rospy.loginfo(np.std([data[45:55])]))
    #         close_areas = self.too_close(data[25:75])
    #     self.dont_crash(close_areas)
    #     self.pub.publish(self.out)

    # def create_message(self, v):
    #     """create optput AckermannDriveStamped mssage
    #     """
    #     self.out.header.stamp = rospy.Time.now()
    #     self.out.header.frame_id = "1"
    #     self.out.drive.steering_angle = 0
    #     self.out.drive.steering_angle_velocity = 0
    #     self.out.drive.speed = v
    #     self.out.drive.acceleration = 0
    #     self.out.drive.jerk = 0

    # def too_close(self, data):
    #     lines = []
    #     for i in range(len(data) - 10):
    #         if np.average(data[i:i+10]) < 2.5:
    #             lines.append((data[i:i+10], i))
    #     return lines

    # def dont_crash(self, close_areas):
    #     lines = self.regression(close_areas)
    #     distances_left, distances_right = self.get_distances(lines)
    #     distance_changes_left, distance_changes_right = get_distance_changes(lines, distances_left, distances_right)
    #     if any([distances_left[i] > distance_changes_left[i] for i in range(len(distances_left))]):
    #         out.drive.speed = 0
    #     if any([distances_right[i] > distance_changes_right[i] for i in range(len(distances_right))]):
    #         out.drive.speed = 0
    #     old_distances = distances

    # def regression(self, close_areas):
    #     pass

    # def get_distances(self, lines):
    #     pass

    # def get_distance_changes(self, distances_left, distances_right):
    #     pass

    # def update_tangents(self, forward, left, right):
    #     if len(self.right_tangents) >= 10:
    #         self.right_tangents.pop()
    #     if len(self.left_tangents) >= 10:
    #         self.left_tangents.pop()
    #     self.left_tangents.insert(0, math.atan(forward/left))
    #     self.right_tangents.insert(0, math.atan(forward/right))
    #     self.left_SD = np.polyfit(np.array(self.left_tangents), self.regression_base, 1)
    #     self.right_SD = np.polyfit(np.array(self.right_tangents), self.regression_base, 1)
