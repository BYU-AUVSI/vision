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
import tf
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from click_to_geolocate.msg import FloatList
from uav_msgs.msg import stampedImage

'''
class to perform calculations on camera image
'''
class camClick:

    '''
    initializes camera parameter
    Imputs:
        gimbal_pos:     3x1 list, [north,east,down].T, position of gimbal
                        center relative to MAV body center
        v:              2x1 float, [v_x,v_y].T, field of view of camera in degrees
    '''
    def __init__(self,gimbal_pos,v):

        #transform from gimbal center to MAV center (expressed in body frame)
        self.MAV_to_gimbal = np.array([[gimbal_pos[0]],[gimbal_pos[1]],[gimbal_pos[2]]])

        #set default gimbal angles. will be overwritten when getNED called
        self.alpha_az = 0.0
        self.alpha_el = -np.pi/4

        #set field of view
        self.v_w = v[0]*np.pi / 180.0
        self.v_h = v[1]*np.pi / 180.0

    '''
    Inputs:
        states_image:   6x1 list, [pn,pe,pd,phi,theta,psi].T, states of the
                        MAV (position and attitude only)
        gimbal_angles:  2x1 list, [alpha_az,alpha_el].T, gimbal position
                        angles
                        alpha_az is a right-handed roation about k^b
                        negative alpha_el points towards ground
    '''
    def setStatesandAngles(self, states_image, gimbal_angles):
        self.pn = states_image[0]
        self.pe = states_image[1]
        self.pd = states_image[2]
        self.phi = states_image[3]
        self.theta = states_image[4]
        self.psi = states_image[5]

        self.alpha_az = gimbal_angles[0]
        self.alpha_el = gimbal_angles[1]

    '''
    gets position of the object in the inertial frame
    Required Set Previously:
        self.states_image   [pn,pe,pd,phi,theta,psi]
        self.gimbal_angles  [alpha_az,alpha_el]
                            alpha_az is a right-handed rotation about k^b
                            negative alpha_el points towards ground
    Inputs:
        pixel_pt:       2x1 ndarray, [u,v].T, pixel location from origin in top
                        left corner
        image_size:     2x1 ndarray, [height,width].T, size of image
        R_b_i:          3x3 ndarray, rotation matrix from body to inertial frame
                        currently doesn't work if initialized in this method,
                        must be passed in
    '''
    def getNED(self,pixel_pt,image_size,R_b_i):
        position = np.array([[self.pn],[self.pe],[self.pd]])

        #used in transformation
        h = -self.pd
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

        M_w = width
        f_w = M_w / (2.0*np.tan(self.v_w/2.0))
        M_h = height
        f_h = M_h / (2.0*np.tan(self.v_h/2.0))

        #distances in pixels from origin of camera frame. origin is center of image.
        #don't forget that y is down in camera frame

        eps_x = pt_x - width / 2.0
        eps_y = pt_y - height / 2.0

        F_w = np.sqrt(f_w**2 + eps_x**2 + eps_y**2)
        F_h = np.sqrt(f_h**2 + eps_x**2 + eps_y**2)

        l_c_w = (1/F_w)*np.array([[eps_x],[eps_y],[f_w]])
        big_term_w = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c_w)))
        den_w = np.dot(k_i.T,big_term_w)
        l_c_h = (1/F_h)*np.array([[eps_x],[eps_y],[f_h]])
        big_term_h = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c_h)))
        den_h = np.dot(k_i.T,big_term_h)

        p_obj_w = position + R_b_i.dot(self.MAV_to_gimbal) + h*big_term_w/den_w
        p_obj_h = position + R_b_i.dot(self.MAV_to_gimbal) + h*big_term_h/den_h
        p_obj = [float(p_obj_h[0]),float(p_obj_w[1]),float(p_obj_w[2])]

        return p_obj

'''
ROS class for managing data
'''
class listen_and_locate:

    def __init__(self,gimbal_pos,v):
        self.image_sub = rospy.Subscriber('/image_stamped',stampedImage,self.image_cb)
        self.pub = rospy.Publisher('pixel_data', FloatList, queue_size=10)
        self.bridge = CvBridge()

        self.camera = camClick(gimbal_pos,v)

        self.pixPt = []
        self.refPt = FloatList()

        self.br_1 = tf.TransformBroadcaster()


    def image_cb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data.image, "bgr8")
        except CvBridgeError as e:
            print(e)

        gimbal_pos = [data.alpha_az, data.alpha_el]
        states_image = [data.pn,data.pe,data.pd,data.phi,data.theta,data.psi]

        self.camera.setStatesandAngles(states_image,gimbal_pos)

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
            self.pixPt = [x,y]
            print(self.pixPt)

            #still annoyingly necessary
            phi = self.camera.phi
            theta = self.camera.theta
            psi = self.camera.psi

            R_b_i = np.array([[np.cos(theta)*np.cos(psi),np.cos(theta)*np.sin(psi),-np.sin(theta)], \
                                   [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi),np.sin(phi)*np.sin(theta)*np.sin(psi) \
                                    +np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)],[np.cos(phi)*np.sin(theta)*np.cos(psi) \
                                    +np.sin(phi)*np.sin(psi),np.cos(phi)*np.sin(theta)*np.sin(psi) \
                                    -np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]]).T

            size = self.cv_image.shape
            image_size = [size[0],size[1]]

            self.refPt.data = self.camera.getNED(self.pixPt,image_size,R_b_i)

            self.pub.publish(self.refPt)
            refPt = self.refPt
            '''

            put something here that averages all the points in a target
            stuff above should execute in a loop
            '''
            self.br_1.sendTransform((refPt.data[0],refPt.data[1],refPt.data[2]),
                                    tf.transformations.quaternion_from_euler(0,0,0),
                                    rospy.Time.now(),
                                    "target_1",
                                    "base")


def main(args):
    rospy.init_node('locator', anonymous=True)

    #parameters for camera (eventually will be obtained on initialization)
    gimbal_pos = [0.5,0,0.1]
    v = [45.0,45.0]

    listen = listen_and_locate(gimbal_pos,v)
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
