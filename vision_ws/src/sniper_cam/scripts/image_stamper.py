#! /usr/bin/env python

## Simple ROS node that:
## -subscribes to CompressedImage from PointGrey Camera
## -pairs the CompressedImage with the relevant state data
## -pubishes a state-stamped image at 1 hz


import rospy
from sensor_msgs.msg import CompressedImage
from sniper_cam.msg import state_image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np

class ImageStamper(object):
    # Simple class that takes in images and adds the current state data to them with a custom message type

    def __init__(self):
        self.image_subscriber = rospy.Subscriber('image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)

        # initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.chi = 0.0

        self.alpha_az = 0.0
        self.alpha_el = 0.0

        # initialize image parameters
        self.img_width = 0.0
        self.img_height = 0.0
        self.fov_w = 60.0
        self.fov_h = 45.0

        # initialize current image
        shape = 964, 1288, 3
        self.img_current = np.zeros(shape, np.uint8)



    def image_callback(self, msg):
        # pull off the state info from the message
        self.pn = 0.0
        self.pe = 0.0
        self.pd = -100.0

        self.phi = math.radians(22.5)   #22.5
        self.theta = math.radians(0.0)
        self.psi = math.radians(0.0)

        self.alpha_az = math.radians(90.0)    #90
        self.alpha_el = math.radians(-22.5) #-22.5

        # direct conversion to CV2 of the image portion of the message
        np_arr = np.fromstring(msg.data, np.uint8)
        img_np = cv2.imdecode(np_arr, 1)
        self.img_current = cv2.imdecode(np_arr, 1)
        # make a copy of img_np for saving and accessing elsewhere in the class
        #self.img_current = img_np

        # get the width and height of the image
        height, width, channels = img_np.shape
        self.img_width = width
        self.img_height = height

        # get the time
        self.get_current_time()

        # display the image
        cv2.rectangle(img_np,(0,0),(200,45),(0,0,0),-1)
        cv2.putText(img_np,"Status: " + self.status,(5,20),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0))
        cv2.putText(img_np,self.time_str,(5,40),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0))
        cv2.imshow('sniper cam image', img_np)
        # wait about a second
        cv2.waitKey(999)



def main():
    #initialize the node
    rospy.init_node('image_stamper')

    #create instance of class that subscribes to the stamped_image
    subscriber = ImageStamper()
    #spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    #OpenCV cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
