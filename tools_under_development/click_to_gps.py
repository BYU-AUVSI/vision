# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 04:36:30 2017

Written by: Peter Schleede for the BYU AUVSI team
This script finds the GPS coordinates of a click in an image taken from the MAV
It will eventually be implemented in ROS
"""

import numpy as np
import cv2

#make it a class initialized with gimbal positions, camera calibration?
#then you can just rewrite the angles if you need to and use the conversion as
#a method where you give it the states and pixel location

#also need a method (probably after construction) that sets the transform from
#the gimbal to the center of the MAV


point = []
#get (x,y) from image
def returnXY(event, x, y, flags, param):
    global point

    if event == cv2.EVENT_LBUTTONDOWN:
        point = [x,y]

#gets position of the object in the inertial frame
#requires a 6x1 ndarray of current MAV states.
#--first 3 are NED in inertial frame
#--second 3 are Euler angles
def getNED(states_image,R_b_i):
    phi = states_image[3]
    theta = states_image[4]
    psi = states_image[5]

    #used in transformation
    h = -states_image[2]
    k_i = np.array([[0],[0],[1]])

    #gimbal orientation
    alpha_az = 0
    alpha_el = -np.pi/4

    #rotation matrices. nomenclature is R_[initial fram]_[final frame]
    #for some reason, R_b_i isn't working in here. So pass it in instead

    R_g_b = np.array([[np.cos(alpha_el)*np.cos(alpha_az),np.cos(alpha_el)*np.sin(alpha_az),-np.sin(alpha_el)], \
                       [-np.sin(alpha_az),np.cos(alpha_az),0],[np.sin(alpha_el)*np.cos(alpha_az), \
                        np.sin(alpha_el)*np.sin(alpha_az),np.cos(alpha_el)]]).T

    R_c_g = np.array([[0,1,0],[0,0,1],[1,0,0]]).T

    #this needs to be fixed in some way to make the image actually square in pixels
    #also do the squaring before showing the image (so right before loop)
    #then do the calculations based on the size of the image
    #then edit the eps_x and eps_y

    #l-unit vector from camera towards click (assumes camera FOV is square wrt pixels)
    #focal length f, pixels M, and field of view v are all related. But if you already
    #know f, just use that. However, f is in units of pixels
    M = 150.0
    v = 45.0*np.pi / 180.0
    f = M / (2.0*np.tan(v/2.0))

    image = cv2.imread('campus.jpg')
    cv2.namedWindow("Campus")
    cv2.setMouseCallback("Campus",returnXY)
    while True:
        cv2.imshow("Campus",image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("z"):
            break

    print("Point: " + str(point))

    pt_x = point[0]
    pt_y = point[1]

    height = np.size(image, 0)
    width = np.size(image, 1)

    print(str(height) + 'x' + str(width))

    # print('Width: ' + str(width))
    # print('Height: ' + str(height))

    #distances in pixels from origin of camera frame. origin is center of image.
    #don't forget that y is down in camera frame

    eps_x = pt_x - width / 2.0
    eps_y = pt_y - height/2.0

    F = np.sqrt(f**2 + eps_x**2 + eps_y**2)

    l_c = (1/F)*np.array([[eps_x],[eps_y],[f]])

    big_term = R_b_i.dot(R_g_b.dot(R_c_g.dot(l_c)))

    den = np.dot(k_i.T,big_term)

    p_obj = states_image[0:3] + h*big_term/den

    #since we are using the flat earth model and to avoid the system freaking out
    #with division, set p_obj[2]=0
    p_obj[2] = 0

    return p_obj


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

p_obj = getNED(states,R_b_i)
print(p_obj)
