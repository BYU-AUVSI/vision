#!/usr/bin/env python

import rospy
import numpy as np
from click_to_geolocate.msg import FW_State
import sys

class state_publisher:
    def __init__(self):

        self.statePub = rospy.Publisher('/state',FW_State,queue_size=1)

        self.msg = FW_State()

        # self.msg.header.stamp = rospy.Time.now()
        self.msg.position = [150,0,-40]
        self.msg.Va = 0
        self.msg.alpha = 0
        self.msg.beta = 0
        self.msg.phi = 0.0*np.pi/180.0
        self.msg.theta = 0.0*np.pi/180.0
        self.msg.psi = 0.0*np.pi/180.0
        self.msg.chi = 0
        self.msg.p = 0
        self.msg.q = 0
        self.msg.r = 0
        self.msg.Vg = 0
        self.msg.wn = 0
        self.msg.we = 0
        self.msg.quat = [0,0,0,1]
        self.msg.quat_valid = 0

        self.statePub.publish(self.msg)

def main(args):
    rospy.init_node('state_pub',anonymous=True)
    # imageSer=state_publisher()

    statePub = rospy.Publisher('/state',FW_State,queue_size=1)
    rate = rospy.Rate(10) # 10hz

    msg = FW_State()
    t = rospy.Time.now()
    print(t)

    msg.header.stamp = t
    msg.position = [150,0,-40]
    msg.Va = 0
    msg.alpha = 0
    msg.beta = 0
    msg.phi = 0.0*np.pi/180.0
    msg.theta = 0.0*np.pi/180.0
    msg.psi = 0.0*np.pi/180.0
    msg.chi = 0
    msg.p = 0
    msg.q = 0
    msg.r = 0
    msg.Vg = 0
    msg.wn = 0
    msg.we = 0
    msg.quat = [0,0,0,1]
    msg.quat_valid = 0

    print(msg)

    while not rospy.is_shutdown():
        print(msg)
        statePub.publish(msg)
        rate.sleep()

    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
