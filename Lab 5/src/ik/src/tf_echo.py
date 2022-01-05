#!/usr/bin/env python
import tf2_ros
import sys
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
def listener():
	target_frame = sys.argv[1]
	source_frame = sys.argv[2]
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	while not(rospy.is_shutdown()):
		try:
			trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
			print(trans.transform)
		except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

if __name__ == '__main__':

	# Run this program as a new node in the ROS computation graph called
	# /listener_<id>, where <id> is a randomly generated numeric string. This
	# randomly generated name means we can start multiple copies of this node
	# without having multiple nodes with the same name, which ROS doesn't allow.
	rospy.init_node('listener', anonymous=True)

	listener()



