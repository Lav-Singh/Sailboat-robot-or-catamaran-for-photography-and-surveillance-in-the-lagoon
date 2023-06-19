#!/usr/bin/env python3

import math
import numpy
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist,PoseArray, Pose
from Tools.controller import PIDController
from Tools.utilities import  gps_to_enu, quaternion_to_euler, euler_to_quaternion, normalize_angle
from dynamic_reconfigure.server import Server
from math import atan2, sqrt, pi
"""
// Project Title   : Sailboat robot (or catamaran) for photography and surveillance in the lagoon.
// Purpose         : Development of a control system in a simulated environment so that it can be used in a working prototype. This node computes the angular and linear velocities for the WAMV
// Language        : Python and ROS
// Author          : Ramessur Lav Singh
// Github          : https://github.com/Lav-Singh/Sailboat-robot-or-catamaran-for-photography-and-surveillance-in-the-lagoon
// Date            : 19 June 2023

// Université des Mascareignes (UdM)
// Faculty of Information and Communication Technology
// Master Artificial Intelligence and Robotics
// Official Website: https://udm.ac.mu
"""

class Navigation:
    def __init__(self):
        # Initialize Navigation
        self.cur_pos      = None # Current 2D position (x, y)
        self.cur_rot      = None # Current (yaw)
        self.cur_position = None # Current 3D position (x, y, z)
        self.cur_rotation = None # Current 3D orientation (roll, pitch, yaw)
        self.wps_pos_x    = [] # List of desired position (x) of all waypoints
        self.wps_pos_y    = [] # List of desired position (y) of all waypoints
        self.wps_rot_z    = [] # List of desired orientation (yaw) of all waypoints
        self.wp_count     = 0 # Waypoint count (starts from 1)
        self.wp_index     = 0 # Waypoint index (starts from 0)
        self.desired_pos      = None # Desired 2D position (x, y)
        self.desired_yaw      = None # Desired yaw
        self.time         = None # Current timestamp
        self.config       = {} # Navigation configuration
        
        # ROS infrastructure
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

    def gps_callback(self, msg):
        if self.cur_rot is None: 
            return
        lat = msg.latitude
        lon = msg.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon) #convert GPS coordinates to East, North, Up coordinates 
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        pos_x += self.gps_offset * math.cos(self.cur_rot[2])
        pos_y += self.gps_offset * math.sin(self.cur_rot[2])
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_position = numpy.array([pos_x, pos_y, pos_z])
        self.cur_rotation = euler_to_quaternion(self.cur_rot[0], self.cur_rot[1], self.cur_rot[2])

    #get pose of boat from IMU sensor
    def imu_callback(self, msg):
        self.cur_rot = quaternion_to_euler(msg.orientation)
    
    #Testing straight line movement
    def waypoint_callbackSINGLE(self, msg):
        if msg:
            lat = msg.latitude
            lon = msg.longitude
            pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
            
            #boat's station keeping pose when arriving at desired GPS coords
            rot_z = 0.0  

            self.wp_count = 1
            self.wps_pos_x.append(pos_x)
            self.wps_pos_y.append(pos_y)
            self.wps_rot_z.append(rot_z)
            
    #Testing Automatic aquisition of waypoints        
    def waypoint_callbackAUTO(self, msg):
        if msg:
            pose_array = msg.poses
            for pose in pose_array:
                latitude = pose.position.x
                longitude = pose.position.y
                pos_x, pos_y, pos_z = gps_to_enu(latitude, longitude)

                # Boat's station keeping pose when arriving at desired GPS coordinates
                rot_z = 0.0

                if not hasattr(self, 'wp_count'):
                    self.wp_count = 1
                else:
                    self.wp_count += 1

                if not hasattr(self, 'wps_pos_x'):
                    self.wps_pos_x = []
                if not hasattr(self, 'wps_pos_y'):
                    self.wps_pos_y = []
                if not hasattr(self, 'wps_rot_z'):
                    self.wps_rot_z = []

                self.wps_pos_x.append(pos_x)
                self.wps_pos_y.append(pos_y)
                self.wps_rot_z.append(rot_z)
                
    #Testing hardcoded waypoints
    def waypoint_callbackTEST(self, msg):
        coordinates = [
            (-33.72228314943176, 150.67420776588426),
            (-33.72238622670082, 150.6743578096381),
            (-33.72262870794784, 150.67438657962234),
            (-33.72277469337388, 150.67401677094486)
        ]

        if not hasattr(self, 'wp_count'):
            self.wp_count = 1
        else:
            self.wp_count += 1

        if not hasattr(self, 'wps_pos_x'):
            self.wps_pos_x = []
        if not hasattr(self, 'wps_pos_y'):
            self.wps_pos_y = []
        if not hasattr(self, 'wps_rot_z'):
            self.wps_rot_z = []

        for coordinate in coordinates:
            lat, lon = coordinate
            pos_x, pos_y, pos_z = gps_to_enu(lat, lon)

            # Boat's station keeping pose when arriving at desired GPS coords
            rot_z = 0.0

            if (pos_x, pos_y) not in zip(self.wps_pos_x, self.wps_pos_y):
                self.wps_pos_x.append(pos_x)
                self.wps_pos_y.append(pos_y)
                self.wps_rot_z.append(rot_z)
                
    #This Function calculates the linear and angular velocities to be published to CMD_VEL for the Thruster control node. 
    def Navigation(self):
        # Open the file for writing
        file_path = "position_data.txt"  # Specify the file path
        file_mode = "a"  # Open the file in write mode

        with open(file_path, file_mode) as file:
            # navigate to the waypoint
            self.desired_pos = numpy.array([self.wps_pos_x[self.wp_index], self.wps_pos_y[self.wp_index]])
            self.desired_yaw = self.wps_rot_z[self.wp_index]
            self.time = rospy.get_time()
            if bool(self.config) and not [x for x in (self.cur_pos, self.cur_rot, self.desired_pos, self.desired_yaw, self.wp_count, self.time) if x is None]:
                err_pos = self.desired_pos - self.cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
                if numpy.linalg.norm(err_pos) > 3: # (Euclidean distance to goal as L2 norm)
                    #not in range of destination, calculate linear and angular velocities in x and z
                    lin_vel_x = numpy.linalg.norm(err_pos) * self.v_const # P controller for linear velocity
                    if lin_vel_x > self.v_limit: # limit linear velocity 
                        lin_vel_x = self.v_limit
                    err_rot = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.cur_rot[2]) # Error in orientation
                    ang_vel_z = self.pid_g2g.control(err_rot, self.time) # PID controller for angular velocity
                else:
                    # If near goal, perform fine adjustments in Vx & Wz for station-keeping
                    rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.cur_rot[2]) # G2G rotation transformation
                    err_pos = numpy.array([numpy.linalg.norm(err_pos) * math.cos(rot_tf), numpy.linalg.norm(err_pos) * math.sin(rot_tf)]) # Error in position (in local frame)
                    lin_vel_x = self.pid_station_lin.control(err_pos[0], self.time) # PID controller for linear velocity
                    err_rot = normalize_angle(self.desired_yaw - self.cur_rot[2]) # Error in orientation
                    ang_vel_z = self.pid_station_ang.control(err_rot, self.time) # PID controller for angular velocity
                print("Navigating to waypoint {} of {}...".format(self.wp_index+1, self.wp_count)) # Counting and indexing starts from 1
                print("Current Target Waypoint: {:.4} m, {:.4} m, {:.4} rad".format(self.desired_pos[0], self.desired_pos[1], self.desired_yaw))
                print()

                # Get the current ROS time
                ros_time = rospy.Time.now()

                # Write position data to text file
                file.write("Position: {:.4f} m, {:.4f} m | Desired Destination: {:.4f} m, {:.4f} m\n".format(self.cur_pos[0], self.cur_pos[1], self.desired_pos[0], self.desired_pos[1]))

                # publish Linear and angular velocity to cmd_vel for control node 
                self.cmd_vel_msg.linear.x = lin_vel_x
                self.cmd_vel_msg.angular.z = ang_vel_z
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

                # Check if current waypoint is reached
                if numpy.linalg.norm(err_pos) < self.pos_tol :
                    # Upon reaching current waypoint, target the next waypoint
                    self.wp_index += 1
                    # If all waypoints have been visited, target the first waypoint again
                    if self.wp_index >= self.wp_count:
                        self.wp_index = 0

            # Close the file
            file.close()

    #This function initializes the controllers used in the navigation.
    def config_callback(self):
        self.gps_offset = 0.85 # GPS offset w.r.t. WAM-V along X-axis
        self.pid_g2g    = PIDController(21, 0.27, 2.4, 50) # Go-to-goal PID controller(21,0.27,2.4,50)
        self.pid_station_lin  = PIDController(1.9, 0.12, 1.8, 50) # Station-keeping lin_vel PID controller
        self.pid_station_ang  = PIDController(2.0, 0.11, 1.3, 50) # Station-keeping angular_vel PID controller
        self.v_const    = 5.7 # Proportional gain for linear velocity outside goal tolerance #pre-tune 5.7
        self.v_limit    = 2.23 # maximum allowed velocity while far from goal
        self.pos_tol    = 0.5 # Position tolerance to determine whether goal is reached


if __name__ == '__main__':
    rospy.init_node('wamv_wayfinding')

    # Navigation class instance
    navigation_node = Navigation()

    # initialization of controllers
    navigation_node.config_callback()

    # Message
    navigation_node.cmd_vel_msg  = Twist()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, navigation_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, navigation_node.imu_callback)
    rospy.Subscriber('/destination_gps', PoseArray, navigation_node.waypoint_callbackTEST) 

    # Publisher
    navigation_node.cmd_vel_pub  = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size = 10)

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
    rospy.wait_for_message('/destination_gps', PoseArray)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            navigation_node.Navigation()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
