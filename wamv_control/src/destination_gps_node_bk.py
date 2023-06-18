#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

if __name__ == '__main__':
    rospy.init_node('destination_gps_publisher')
    pub = rospy.Publisher('destination_gps',NavSatFix, queue_size=10)
    
    # Create   a NavSatFix message with lat and long
    msg = NavSatFix()
    msg.latitude =  -33.72228314943176
    msg.longitude = 150.67420776588426
    
    rate  =rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
    #publish to topic
        pub.publish(msg)
        rate.sleep()
