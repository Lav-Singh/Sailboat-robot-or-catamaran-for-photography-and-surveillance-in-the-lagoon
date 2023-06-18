#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from Tools.controller import PIDController
from wamv_control.cfg import NavigationConfig
from dynamic_reconfigure.server import Server
from math import atan2, sqrt, pi
import math 

# Constants
Kp = 1  # Proportional gain of PID controller
Ki = 0  # Integral gain of PID controller
Kd = 0.001  # Derivative gain of PID controller

# Variables
last_error = 0
error_sum = 0
last_time = 0
boat_heading = 0
gps_offset = 0
pid_g2g    = None
pid_sk_vx  = None
pid_sk_vy  = None
pid_sk_wz  = None
goal_tol   = None
v_const    = None
v_limit    = None
pos_tol    = None
rot_tol    = None
debug      = None
config     = {}

# Setpoint
target_latitude = None
target_longitude = None
startLat = None
startLong = None

#global variables 
slope = None 
y_intercept = None 
dyn_reconf_srv = None

#haversine formula
def distance_to_dest(lat1, lon1, lat2, lon2):
    # Radius of the earth in meters
    R = 6371000

    # Convert latitude and longitude to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Calculate the differences between the latitudes and longitudes
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Apply the Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    return distance
    
def normalize_angle(theta):
    '''
    Normalize angle within [-pi, pi).
    :param theta: Angle in radians
    :return theta_norm: Normalized angle in radians
    '''
    theta_norm = (theta + math.pi) % (2 * math.pi) - math.pi
    return theta_norm
    
def define_line_orig_dest():
    global slope, y_intercept
    # Calculate the slope and y-intercept of line AB
    slope = (target_latitude - startLat) / (target_longitude - startLong)
    print(slope)
    y_intercept = startLat - slope * startLong
    print(y_intercept)
def init_last_time():
    global last_time
    last_time = rospy.get_time()
    print(last_time)

def config_callback(self, config):
        # Handle updated configuration values
        gps_offset = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        pid_g2g    = PIDController(config['G2G_kP'], config['G2G_kI'], config['G2G_kD'], config['G2G_kS']) # Go-to-goal PID controller
        pid_sk_vx  = PIDController(config['SK_Vx_kP'], config['SK_Vx_kI'], config['SK_Vx_kD'], config['SK_Vx_kS']) # Station-keeping Vx PID controller
        pid_sk_vy  = PIDController(config['SK_Vy_kP'], config['SK_Vy_kI'], config['SK_Vy_kD'], config['SK_Vy_kS']) # Station-keeping Vy PID controller
        pid_sk_wz  = PIDController(config['SK_Wz_kP'], config['SK_Wz_kI'], config['SK_Wz_kD'], config['SK_Wz_kS']) # Station-keeping Wz PID controller
        goal_tol   = config['goal_tol'] # Goal tolerance dead-band of go-to-goal PID controller
        v_const    = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        v_limit    = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        pos_tol    = config['pos_tol'] # Position tolerance to determine whether goal is reached
        rot_tol    = config['rot_tol'] # Orientation tolerance to determine whether goal is reached
        debug      = config['debug'] # Flag to enable/disable debug messages
        config     = config
        return config        

def imu_callback(msg):
    global boat_heading
    # Extract the orientation quaternion from the IMU message
    orientation_q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    
    # Convert the orientation quaternion to Euler angles (roll, pitch, yaw)
    euler = euler_from_quaternion(orientation_q)
    
    # Extract the yaw angle (heading) from the Euler angles
    heading_sensor = euler[2]
    boat_heading = heading_sensor
    
    print("Current Boat Heading: {}".format(boat_heading))

def destination_callback(msg):
    global target_latitude,target_longitude
    if target_latitude is None or target_longitude is None:
        target_latitude = msg.latitude
        print(target_latitude)
        target_longitude = msg.longitude
        print(target_longitude)
# Callback function for GPS data
def gps_callback(msg):
    global last_error, error_sum, last_time,startLat,startLong
    
    if startLat is None or startLong is None:
        startLat = msg.latitude
        print("start lat ",startLat)
        startLong = msg.longitude
        print("start long ",startLong)
    
    if(slope is None or y_intercept is None):
        define_line_orig_dest()
        print('test')
        
    # Calculate the distance to the target point
    #distance = sqrt((target_latitude - msg.latitude)**2 + (target_longitude - msg.longitude)**2) in degrees
    distance = distance_to_dest(target_latitude,target_longitude,msg.latitude,msg.longitude)
    #print("Distance to dest: {}".format(distance))
    if distance > 4:  # If the distance is greater than 4 meter
        #print("Distance to dest: {}".format(distance))
        # Calculate the desired heading angle
        heading = atan2(target_latitude - msg.latitude, target_longitude - msg.longitude)
        print("Heading to dest: {}".format(heading))
        # Calculate the error and error derivative for the PID controller
        # Calculate the perpendicular distance from the current GPS point to line AB
        #error = ((slope * msg.longitude) - msg.latitude + y_intercept) / sqrt(slope**2 + 1)
        
        error = heading - boat_heading
        #print('error ',error)
        dt = rospy.get_time() - last_time
        error_derivative = (error - last_error) / dt
        # Update the error sum and last error
        error_sum += error
        last_error = error
        last_time = rospy.get_time()
        # Calculate the PID output
        pid_output = Kp*error + Ki*error_sum + Kd*error_derivative
        theta = normalize_angle(pid_output)
        # Create the thrust message
        thrust_msg = Twist()
        thrust_msg.linear.x = 0.6  # Set a constant speed of 2 m/s
        thrust_msg.angular.z = pid_output  # Set the angular velocity to the PID output
        # Publish the thrust message
        thrust_pub.publish(thrust_msg)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('wamv_gps_navigation')
    # initialize Time
    init_last_time()
    
    # Dynamic reconfigure server
    dyn_reconf_srv = Server(NavigationConfig, config_callback)
    print('test')
    #get current heading of the boat
    imu_sub = rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, imu_callback)
    # Subscribe to the target GPS coords
    target_gps_sub = rospy.Subscriber('/destination_gps', NavSatFix, destination_callback)
    # Subscribe to the GPS data
    gps_sub = rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, gps_callback)
    # Create the thrust publisher
    thrust_pub = rospy.Publisher('/wamv/thrusters/velocity_cmd', Twist, queue_size=10)
    # Spin the node
    rospy.spin()
