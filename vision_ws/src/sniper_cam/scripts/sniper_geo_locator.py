#! /usr/bin/env python

## Simple ROS node that:
## -subscribes to stamped_image topic
## -displays the image portion of the message
## -writes image to 'backup' file
## -if user clicks target in image:
##  -capture x,y pixel coord
##  -calculate target NED location
##  -wirte image to sorted file *1
##  -write NED location and heading to file *2
## -pressing number key (1,2,...,9) changes the target number

## Geolocation Based on Chapter 13 of Small Unmanned Aircraft by Beard and McLain
## Jesse Wynn AUVSI '17
## Peter Schleede AUVSI '17

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np



class SniperGeoLocator(object):
    # Simple class that does cool stuff

    def __init__(self):
        self.stamped_image_subscriber = rospy.Subscriber('sniper_cam/image/compressed', CompressedImage, self.image_callback, queue_size=1)

        # setup mouse click callback
        cv2.namedWindow('sniper cam image')
        cv2.setMouseCallback('sniper cam image', self.click_and_locate)

        #transform from gimbal center to MAV center (expressed in body frame)
        self.MAV_to_gimbal = np.array([[gimbal_pos[0]],[gimbal_pos[1]],[gimbal_pos[2]]])

        # initialize variables
        self.pn = 0
        self.pe = 0
        self.pd = 0

        self.phi = 0
        self.theta = 0
        self.psi = 0

        self.alpha_az = 0
        self.alpha_el = 0

        # image parameters
        self.width = 0
        self.height = 0
        self.fov_w = 60
        self.fov_h = 45


    def image_callback(self, msg):
        # pull off the state info from the message
        self.pn = 0
        self.pe = 0
        self.pd = -100

        self.phi = math.radians(22.5)
        self.theta = math.radians(0)
        self.psi = math.radians(0)

        self.alpha_az = math.radians(90)
        self.alpha_el = math.radians(-22.5)

        # direct conversion to CV2 of the image portion of the message
        np_arr = np.fromstring(msg.data, np.uint8)
        img_np = cv2.imdecode(np_arr, 1)

        # get the width and height of the image
        height, width, channels = img_np.shape
        self.width = width
        self.height = height

        # display the image
        cv2.imshow('sniper cam image', img_np)
        # wait about a second
        cv2.waitKey(999)


    def click_and_locate(self, event, x, y, flags, param):
        # if user clicks on target in the image frame
        if event == cv2.EVENT_LBUTTONDOWN:
            # position of the UAV
            position = np.array([[self.pn],[self.pe],[self.pd]])

            #capture pixel coordinates
            px = x;
            py = y;

            # convert to pixel locations measured from image center (0,0)
            eps_x = px - self.width/2.0
            eps_y = py - self.height/2.0

            # define rotation from body to inertial frame
            R_b_i = np.array([[np.cos(self.theta)*np.cos(self.psi),np.cos(self.theta)*np.sin(self.psi),-np.sin(self.theta)], \
                                   [np.sin(self.phi)*np.sin(self.theta)*np.cos(self.psi)-np.cos(self.phi)*np.sin(self.psi),np.sin(self.phi)*np.sin(self.theta)*np.sin(self.psi) \
                                    +np.cos(self.phi)*np.cos(self.psi),np.sin(self.phi)*np.cos(self.theta)],[np.cos(self.phi)*np.sin(self.theta)*np.cos(self.psi) \
                                    +np.sin(self.phi)*np.sin(self.psi),np.cos(self.phi)*np.sin(self.theta)*np.sin(self.psi) \
                                    -np.sin(self.phi)*np.cos(self.psi), np.cos(self.phi)*np.cos(self.theta)]]).T

            # define rotation from gimbal frame to body frame
            R_g_b = np.array([[np.cos(self.alpha_el)*np.cos(self.alpha_az),np.cos(self.alpha_el)*np.sin(self.alpha_az),-np.sin(self.alpha_el)], \
                           [-np.sin(self.alpha_az),np.cos(self.alpha_az),0],[np.sin(self.alpha_el)*np.cos(self.alpha_az), \
                            np.sin(self.alpha_el)*np.sin(self.alpha_az),np.cos(self.alpha_el)]]).T

            # define rotation from camera to gimbal frame
            R_c_g = np.array([[0,1,0],[0,0,1],[1,0,0]]).T

            # define vector k_i as unit vector aligned with inertial down axis
            k_i = np.array([[0],[0],[1]])

            # define height above ground h, as -pd
            h = -self.pd

            # now we need to define vector el_hat_c (here we break it into el_hat_c_w and el_hat_c_h because camera frame is rectangular instead of square)
            f_w = self.width/(2.0*np.tan(self.fov_w/2.0))
            f_h = self.height/(2.0*np.tan(self.fov_h/2.0))

            F_w = np.sqrt(f_w**2 + eps_x**2 + eps_y**2)
            F_h = np.sqrt(f_h**2 + eps_x**2 + eps_y**2)

            el_hat_c_w = (1/F_w)*np.array([[eps_x],[eps_y],[f_w]])
            el_hat_c_h = (1/F_h)*np.array([[eps_x],[eps_y],[f_h]])

            big_term_w = R_b_i.dot(R_g_b.dot(R_c_g.dot(el_hat_c_w)))
            den_w = np.dot(k_i.T,big_term_w)

            big_term_h = R_b_i.dot(R_g_b.dot(R_c_g.dot(el_hat_c_h)))
            den_h = np.dot(k_i.T,big_term_h)

            #calculate the location of the target






















def main():
    #initialize the node
    rospy.init_node('sniper_geo_locator')

    #create instance of class that subscribes to the stamped_image
    subscriber = SniperGeoLocator()
    #spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    #OpenCV cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
