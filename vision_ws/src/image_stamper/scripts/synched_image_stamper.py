#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import time
from uav_msgs.msg import stampedImage, FW_State
# from uav_msgs.msg import stampedImage
# from ~/git/rosplane_ws/src/fcu_common.msg import FW_State
import sys
from message_filters import TimeSynchronizer, Subscriber

class imageStamper:
    def __init__(self,args):

        self.iSPub = rospy.Publisher(args[3],stampedImage,queue_size=1)

        # image_sub = Subscriber(args[1], Image)
        # state_sub = Subscriber(args[2], FW_State)
        # self.tss = TimeSynchronizer([image_sub,state_sub],10)

        self.tss = TimeSynchronizer([Subscriber(args[1], Image),
                                    Subscriber(args[2], FW_State)],10)


        self.tss.registerCallback(self.callback)
        self.SI = stampedImage()

    def callback(self,cam_image,state):
        print('achieved')
        assert cam_image.header.stamp - state.timestamp <= 1
        print('here')
        self.SI.pn = state.position[0]
        self.SI.pe = state.position[1]
        self.SI.pd = state.position[2]
        self.SI.phi = state.phi
        self.SI.theta = state.theta
        self.SI.psi = state.psi

        self.SI.alpha_az = alpha_az
        self.SI.alpha_el = alpha_el
        self.SI.image = cam_image

        self.iSPub.publish(self.SI)

def main(args):
    rospy.init_node('stamper',anonymous=True)
    imageSer=imageStamper(args)
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print('Usage: rosrun image_stamper image_stamper.py <input_image_topic> <input_states> <output_stamped_topic>')
    else:
        main(sys.argv)
