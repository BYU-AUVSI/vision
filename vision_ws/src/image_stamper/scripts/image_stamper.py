#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import time
from uav_msgs.msg import stampedImage
import sys

class imageStamper:
    def __init__(self,args):
        self.imageSub = rospy.Subscriber(args[1],Image,self.callback)
        self.iSPub = rospy.Publisher(args[2],stampedImage,queue_size=1)
        #when we have something publishing states, will need subscribers for that

        self.SI = stampedImage()

    def callback(self,data):
        #we actually won't use velocities and angles will be quaternions
        pn = 0
        pe = 0
        pd = -100
        u = 0
        v = 0
        w = 0
        phi = 0.0*np.pi/180.0
        theta = 0.0*np.pi/180.0
        psi = 0.0*np.pi/180.0
        p = 0
        q = 0
        r = 0
        alpha_az = 0.0*np.pi/180.0
        alpha_el = -45.0*np.pi/180.0

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
        self.SI.alpha_az = alpha_az
        self.SI.alpha_el = alpha_el
        self.SI.image = data

        self.iSPub.publish(self.SI)

def main(args):
    rospy.init_node('stamper',anonymous=True)
    imageSer=imageStamper(args)
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: rosrun image_stamper image_stamper.py <input_image_topic> <output_stamped_topic>')
    else:
        main(sys.argv)
