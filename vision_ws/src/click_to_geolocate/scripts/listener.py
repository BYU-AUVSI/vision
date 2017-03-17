#!/usr/bin/env python

##_TO_DO
##_Subscribe to all of the relevant topics for calculating rough geolocation
##_Synchronize so that when new 'pixel_data' comes in, the most recent data from the other topics gets captured
##_Do the geolocation calculation
##_publish the new location

import rospy
from std_msgs.msg import String
from click_to_geolocate.msg import FloatList
import numpy as np


def callback(data):
    earth_radius_utah = 6370000
    lat_home = 39.9785960
    lon_home = -112.0037920
    pn = data.data[0]
    pe = data.data[1]

    lat = (pn/earth_radius_utah)*180/np.pi + lat_home
    lon = (pe/(earth_radius_utah*np.cos(lat_home*np.pi/180)))*180/np.pi + lon_home

    print data.data

    print('Latitude = ' + str(lat))
    print('Longitude = ' + str(lon))
    #rospy.loginfo(data)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/pixel_data', FloatList, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
