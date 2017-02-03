#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import time
from uav_msgs.msg import stampedImage
import sys

class imageStamper:
    def __init__(self):
        self.imageSub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.iSPub = rospy.Publisher("/image_stamped",stampedImage,queue_size=1)
        #when we have something publishing states, will need subscribers for that

        self.SI = stampedImage()

    def callback(self,data):
        #we actually won't use velocities and angles will be quaternions
        pn = 1
        pe = 2
        pd = 3
        u = 4
        v = 5
        w = 6
        phi = 7
        theta = 8
        psi = 9
        p = 10
        q = 11
        r = 12

        self.SI.pn = pn
        self.SI.pe = pe
        self.SI.pd = pd
        self.SI.u = u
        self.SI.v = v
        self.SI.w = w
        self.SI.phi = phi
        self.SI.theta = theta
        self.SI.psi = psi
        self.SI.p = p
        self.SI.q = q
        self.SI.r = r

        self.iSPub.publish(self.SI)

def main(args):
    rospy.init_node('stamper',anonymous=True)
    imageSer=imageStamper()
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
