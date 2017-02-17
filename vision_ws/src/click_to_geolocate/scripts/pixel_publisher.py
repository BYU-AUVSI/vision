#! /usr/bin/env python

## Simple publisher that publishes pixel coordinates on mouse click
## Publishes to the 'pixel_data' topic
## Jesse Wynn AUVSI '17
## Camera clicker class provides way to create a camera object to get NED coordinates
## of a target from. based on Chapter 13 of Small Unmanned Aircraft by Beard and McLain
## Peter Schleede AUVSI '17

import rospy
import cv2
import numpy as np
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from click_to_geolocate.msg import IntList
from sensor_msgs.msg import Image

'''
class to perform calculations on camera image
'''
class camClick:

    '''
    initializes camera parameter
    Imputs:
        gimbal_pos:     3x1 list, [north,east,down].T, position of gimbal
                        center relative to MAV body center
        v:              float, field of view of camera in degrees
    '''
    def __init__(self,gimbal_pos,v):

        #transform from gimbal center to MAV center (expressed in body frame)
        self.MAV_to_gimbal = np.array([[gimbal_pos[0]],[gimbal_pos[1]],[gimbal_pos[2]]])

        #set default gimbal angles. will be overwritten when getNED called
        self.alpha_az = 0.0
        self.alpha_el = -np.pi/4

        #set field of view
        self.v = v*np.pi / 180.0

    '''
    called by getNED function
    Inputs:
        gimbal_angles:  2x1 list, [alpha_az,alpha_el].T, gimbal position
                        angles
                        alpha_az is a right-handed roation about k^b
                        negative alpha_el points towards ground
    '''
    def setAngles(self, gimbal_angles):
        self.alpha_az = gimbal_angles[0]
        self.alpha_el = gimbal_angles[1]

    '''
    gets position of the object in the inertial frame
    Inputs:
        states_image:   6x1 ndarray, [pn,pe,pd,phi,theta,psi].T, current MAV states
        gimbal_angles:  2x1 ndarray, [alpha_az,alpha_el].T, gimbal position
                        angles
                        alpha_az is a right-handed roation about k^b
                        negative alpha_el points towards ground
        pixel_pt:       2x1 ndarray, [u,v].T, pixel location from origin in top
                        left corner
        image_size:     2x1 ndarray, [height,width].T, size of image
        R_b_i:          3x3 ndarray, rotation matrix from body to inertial frame
                        currently doesn't work if initialized in this method,
                        must be passed in
    '''
    def getNED(self,states_image,gimbal_angles,pixel_pt,image_size,R_b_i):
        pn = states_image[0]
        pe = states_image[1]
        pd = states_image[2]
        phi = states_image[3]
        theta = states_image[4]
        psi = states_image[5]

        self.setAngles(gimbal_angles)

        position = np.array([[pn],[pe],[pd]])

        #used in transformation
        h = -pd
        k_i = np.array([[0],[0],[1]])

        #rotation matrices. nomenclature is R_[initial fram]_[final frame]
        #for some reason, if I create R_b_i here, the method doesn't work
        #passing it in works so pass it in

        R_g_b = np.array([[np.cos(self.alpha_el)*np.cos(self.alpha_az),np.cos(self.alpha_el)*np.sin(self.alpha_az),-np.sin(self.alpha_el)], \
                           [-np.sin(self.alpha_az),np.cos(self.alpha_az),0],[np.sin(self.alpha_el)*np.cos(self.alpha_az), \
                            np.sin(self.alpha_el)*np.sin(self.alpha_az),np.cos(self.alpha_el)]]).T

        R_c_g = np.array([[0,1,0],[0,0,1],[1,0,0]]).T

        #get image size
        height = image_size[0]
        width = image_size[1]

        pt_x = pixel_pt[0]
        pt_y = pixel_pt[1]

        #l-unit vector from camera towards click (assumes camera FOV is square wrt pixels)
        #look in literature for how to do this in a rectangular image

        M = height
        f = M / (2.0*np.tan(self.v/2.0))

        #distances in pixels from origin of camera frame. origin is center of image.
        #don't forget that y is down in camera frame

        eps_x = pt_x - width / 2.0
        eps_y = pt_y - height / 2.0

        F = np.sqrt(f**2 + eps_x**2 + eps_y**2)

        l_c = (1/F)*np.array([[eps_x],[eps_y],[f]])

        big_term = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c)))

        den = np.dot(k_i.T,big_term)

        p_obj = position + R_b_i.dot(self.MAV_to_gimbal) + h*big_term/den

        for i in range(0,3):
            p_obj[i] = float(p_obj[i])

        return p_obj

'''
ROS class for managing data
'''
class listen_and_locate:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw',Image,self.image_cb)
        self.pub = rospy.Publisher('pixel_data', IntList, queue_size=10)
        self.bridge = CvBridge()

        #parameters for camera (eventually will be obtained on initialization)
        gimbal_pos = [0.5,0,0.1]
        v = 45.0
        self.camera = camClick(gimbal_pos,v)

        self.refPt = IntList()

    def image_cb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #creates a named window for our camera, waits for mouse click
        cv2.namedWindow('spotter_cam')
        cv2.setMouseCallback('spotter_cam', self.click_and_pub_pixel_data)
        cv2.imshow('spotter_cam',self.cv_image)
        cv2.waitKey(1)

    '''
    mouse click callback. uses self.camera to publish an NED coordinate
    requires stampedImages to be subscribed to
    '''
    def click_and_pub_pixel_data(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt.data = [x,y]

            #these things all should come from the stampedImage
            states_image = [100.0,10.0,-60.0,0.0*np.pi/180.0,0.0*np.pi/180.0,0.0*np.pi/180.0]
            gimbal_angles = [0.0,-45.0*np.pi/180.0]

            #still annoyingly necessary
            phi = states_image[3]
            theta = states_image[4]
            psi = states_image[5]

            R_b_i = np.array([[np.cos(theta)*np.cos(psi),np.cos(theta)*np.sin(psi),-np.sin(theta)], \
                                   [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi),np.sin(phi)*np.sin(theta)*np.sin(psi) \
                                    +np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)],[np.cos(phi)*np.sin(theta)*np.cos(psi) \
                                    +np.sin(phi)*np.sin(psi),np.cos(phi)*np.sin(theta)*np.sin(psi) \
                                    -np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]]).T

            size = self.cv_image.shape
            image_size = [size[0],size[1]]

            self.refPt.data = self.camera.getNED(states_image,gimbal_angles,self.refPt.data,image_size,R_b_i)

            self.pub.publish(self.refPt)

def main(args):
    rospy.init_node('locator', anonymous=True)
    listen = listen_and_locate()
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
