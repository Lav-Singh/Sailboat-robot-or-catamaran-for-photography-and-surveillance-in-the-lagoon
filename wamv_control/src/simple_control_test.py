#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from threading import Timer
rospy.init_node('simple_thrut_test')
pubRightT = rospy.Publisher('/wamv/thrusters/right_thrust_cmd',Float32,queue_size = 10)
pubLeftT = rospy.Publisher('/wamv/thrusters/left_thrust_cmd',Float32,queue_size = 10)
t = None
def publish_command():
    speed = 0.5
    pubRightT.publish(speed)
    pubLeftT.publish(speed)

    global t
    t=Timer(0.1,publish_command)
    t.start()

publish_command()

rospy.sleep(10)

# Cancel the timer to stop publishing
if t is not None:
    t.cancel()
