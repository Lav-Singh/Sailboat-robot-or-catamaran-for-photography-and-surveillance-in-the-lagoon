#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def constrain(input, low, high):
    '''
    Constrain input between lower and higher bounds.
    :param input: Input value to be constrained
    :param low  : Lower bound
    :param high : Higher bound
    :return output: Constrained output value
    '''
    if input < low:
      output = low
    elif input > high:
      output = high
    else:
      output = input
    return output


# Initialize ROS node
rospy.init_node('wamv_control')

# Create publisher for left thruster
left_thruster_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)

# Create publisher for right thruster
right_thruster_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)

def control_callback(cmd):
    # Extract desired linear and angular velocities from Twist message
    linear_vel = cmd.linear.x
    angular_vel = cmd.angular.z

    # Calculate desired thrust values for left and right thrusters
    left_thrust = (linear_vel - angular_vel)
    right_thrust = (linear_vel + angular_vel)

    # Constrain the thrust commands
    left_thrust = constrain(left_thrust, -1.0, 1.0)
    right_thrust = constrain(right_thrust, -1.0, 1.0)
    
    # Publish thrust commands to left and right thrusters
    left_thruster_pub.publish(left_thrust)
    right_thruster_pub.publish(right_thrust)

# Subscribe to desired velocity commands
rospy.Subscriber('/wamv/cmd_vel', Twist, control_callback)

# Spin the node
rospy.spin()
