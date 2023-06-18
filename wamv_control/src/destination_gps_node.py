#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, Pose

if __name__ == '__main__':
    rospy.init_node('destination_gps_publisher')
    pub = rospy.Publisher('destination_gps', PoseArray, queue_size=10)

    # Create a PoseArray message
    pose_array = PoseArray()

    # Create a list of GPS coordinates
    gps_coordinates = [
        (-33.72228314943176, 150.67420776588426),
        (-33.72238622670082, 150.6743578096381),
        (-33.72262870794784, 150.67438657962234),
        (-33.72277469337388, 150.67401677094486)
    ]

    # Populate the PoseArray message with GPS coordinates
    for latitude, longitude in gps_coordinates:
        pose = Pose()
        pose.position.x = latitude
        pose.position.y = longitude
        pose_array.poses.append(pose)

    rate = rospy.Rate(1)  # 1 per hour
    while not rospy.is_shutdown():
        # Publish the PoseArray message
        pub.publish(pose_array)
        rate.sleep()
