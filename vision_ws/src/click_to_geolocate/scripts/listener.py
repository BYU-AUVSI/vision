#!/usr/bin/env python

##_TO_DO
##_Subscribe to all of the relevant topics for calculating rough geolocation
##_Synchronize so that when new 'pixel_data' comes in, the most recent data from the other topics gets captured
##_Do the geolocation calculation
##_publish the new location

import rospy
from std_msgs.msg import String
from click_to_geolocate.msg import IntList


def callback(data):
    print data.data
    #rospy.loginfo(data)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('pixel_data', IntList, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
