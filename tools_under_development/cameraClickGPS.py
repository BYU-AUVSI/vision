"""
Created on Tue Feb 7, 2017

Written by: Peter Schleede for the BYU AUVSI team
This script implements a class that accepts parameters of a camera on a MAV
and can be used for obtaining GPS coordinates from a clicked image
"""

import numpy as np
import cv2

class camClick:

    '''
    initializes camera parameter
    Imputs:
        gimbal_pos:     3x1 numpy array, [north,east,down].T, position of gimbal
                        center relative to MAV body center
        gimbal_fixed:   boolean, is the gimbal fixed
        gimbal_angles:  2x1 numpy array, [alpha_az,alpha_el].T, gimbal position
                        angles. if gimbal_fixed == false, put [0,0].T
                        alpha_az is a right-handed roation about k^b
                        negative alpha_el points towards ground
    '''
    def __init__(self,gimbal_pos, gimbal_fixed, gimbal_angles):

        #transform from gimbal center to MAV center (expressed in body frame)
        self.MAV_to_gimbal = gimbal_pos

        #set gimbal angles based on if the gimbal is fixed
        if gimbal_fixed:
            self.alpha_az = gimbal_angles[0]
            self.alpha_el = gimbal_angles[1]
        else:
            #set some angles just so things don't get weird
            self.alpha_az = 0.0
            self.alpha_el = -np.pi/4

        self.point = []

    #get (x,y) from image
    def returnXY(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = [x,y]

    #gets position of the object in the inertial frame
    #requires a 6x1 ndarray of current MAV states.
    #--first 3 are NED in inertial frame
    #--second 3 are Euler angles
    def getNED(self,states_image,R_b_i):
        phi = states_image[3]
        theta = states_image[4]
        psi = states_image[5]

        #used in transformation
        h = -states_image[2]
        k_i = np.array([[0],[0],[1]])

        #rotation matrices. nomenclature is R_[initial fram]_[final frame]
        #for some reason, if I create R_b_i here, the method doesn't work
        #passing it in works so pass it in

        R_g_b = np.array([[np.cos(self.alpha_el)*np.cos(self.alpha_az),np.cos(self.alpha_el)*np.sin(self.alpha_az),-np.sin(self.alpha_el)], \
                           [-np.sin(self.alpha_az),np.cos(self.alpha_az),0],[np.sin(self.alpha_el)*np.cos(self.alpha_az), \
                            np.sin(self.alpha_el)*np.sin(self.alpha_az),np.cos(self.alpha_el)]]).T

        R_c_g = np.array([[0,1,0],[0,0,1],[1,0,0]]).T

        #this needs to be fixed in some way to make the image actually square in pixels
        #also do the squaring before showing the image (so right before loop)
        #then do the calculations based on the size of the image
        #then edit the eps_x and eps_y

        #load image
        image = cv2.imread('campus.jpg')

        #get image size for cropping it to be square
        size = image.shape
        height = size[0]
        width = size[1]

        # cropped
        cv2.namedWindow("Campus")
        cv2.setMouseCallback("Campus",self.returnXY)
        while True:
            cv2.imshow("Campus",image)
            #I don't know why this waitKey is necessary but the loop doesn't work without it,
            #even though you do nothing with the key
            key = cv2.waitKey(1) & 0xFF

            if self.point != []:
                break

        # print("Point: " + str(self.point))

        pt_x = self.point[0]
        pt_y = self.point[1]

        size = image.shape

        height = size[0]
        width = size[1]

        #l-unit vector from camera towards click (assumes camera FOV is square wrt pixels)
        #focal length f, pixels M, and field of view v are all related. But if you already
        #know f, just use that. However, f is in units of pixels
        M = 150.0
        v = 45.0*np.pi / 180.0
        f = M / (2.0*np.tan(v/2.0))

        #distances in pixels from origin of camera frame. origin is center of image.
        #don't forget that y is down in camera frame

        eps_x = pt_x - width / 2.0
        eps_y = pt_y - height/2.0

        F = np.sqrt(f**2 + eps_x**2 + eps_y**2)

        l_c = (1/F)*np.array([[eps_x],[eps_y],[f]])

        big_term = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c)))

        den = np.dot(k_i.T,big_term)

        p_obj = states_image[0:3] + R_b_i.dot(self.MAV_to_gimbal) + h*big_term/den

        for i in range(0,3):
            p_obj[i] = float(p_obj[i])

        return p_obj

if __name__ == '__main__':
    pn = 100
    pe = 10
    pd = -60
    phi = 0.0*np.pi/180.0
    theta = 0.0*np.pi/180.0
    psi = 0.0*np.pi/180.0

    R_b_i = np.array([[np.cos(theta)*np.cos(psi),np.cos(theta)*np.sin(psi),-np.sin(theta)], \
                           [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi),np.sin(phi)*np.sin(theta)*np.sin(psi) \
                            +np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)],[np.cos(phi)*np.sin(theta)*np.cos(psi) \
                            +np.sin(phi)*np.sin(psi),np.cos(phi)*np.sin(theta)*np.sin(psi) \
                            -np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]]).T

    states = np.array([[pn,pe,pd,phi,theta,psi]]).T

    gimbal_north = 0.5
    gimbal_east = 0
    gimbal_down = 0
    MAV_to_gimbal = np.array([[gimbal_north,gimbal_east,gimbal_down]]).T

    gimbal_fixed = True
    alpha_az = 0
    alpha_el = -np.pi/4
    gimbal_angles = np.array([[alpha_az,alpha_el]]).T

    cam = camClick(MAV_to_gimbal,gimbal_fixed,gimbal_angles)

    p_obj = cam.getNED(states,R_b_i)
    print(p_obj)
