#!/usr/bin/env python
import rospy
from uuv_sensor_ros_plugins_msgs.msg import DVL

def callback(data):
    rospy.loginfo(data.velocity)
    
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/triton/dvl", DVL, callback)
	rospy.spin()

if __name__ == '__main__':
    listener()