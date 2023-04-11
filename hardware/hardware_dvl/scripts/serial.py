#!/usr/bin/python3

from wldvl import WlDVL
import rospy
from std_msgs.msg import String
from uuv_sensor_ros_plugins_msgs.msg import DVL


def talker():
    pub = rospy.Publisher('dvla50', DVL, queue_size=10)
    rospy.init_node('dvl_reader', anonymous=True)
    rate = rospy.Rate(4) # 10hz

    dvl = WlDVL("/dev/ttyUSB0") # CHECK NAME

    # #Attempt to open a connection
    try:
        dvl = WlDVL("/dev/ttyUSB0")
        while not rospy.is_shutdown():
            report = dvl.read()

            if report is not None:
                dvl_msg = DVL()

                dvl_msg.header.stamp = rospy.Time.now()

                dvl_msg.velocity.x = report['vx']
                dvl_msg.velocity.y = report['vy']
                dvl_msg.velocity.z = report['vz']
                dvl_msg.altitude = report['altitude']
                if (dvl_msg.velocity.x != 0.0 and dvl_msg.velocity.y != 0.0 and dvl_msg.velocity.z != 0.0) :
                    pub.publish(dvl_msg)

        rate.sleep()
    except:
        print("FAILED TO OPEN PORT")



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
