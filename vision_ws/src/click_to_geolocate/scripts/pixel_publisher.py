#! /usr/bin/env python

## Simple publisher that publishes pixel coordinates on mouse click
## Publishes to the 'pixel_data' topic
## Jesse Wynn AUVSI '17

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from click_to_geolocate.msg import IntList
from click_to_geolocate.msg import FloatList

#camera calibration parameters (this calibration could be re-done to be more accurate)
#intrinsic parameters
fx = 474.788647 #x focal length in pixels
ox = 311.008804 #x coordinate of optical center in pixels
fy = 467.476429 #y focal length in pixels
oy = 212.330799 #y coordinate of optical center in pixels

#distortion coefficients
k1 = -4.67962e-01
k2 = 2.92767e-01
p1 = 1.810e-03
p2 = 1.383e-03
k3 = -1.19120e-01

camMatrix = np.array([[fx, 0.0, ox],
                      [0.0, fy, oy],
                      [0.0, 0.0, 1.0]], dtype = np.float64)

distCoeff = np.array([k1, k2, p1, p2, k3], dtype = np.float64)


#define mouse callback function to capture and publish pixel data
def click_and_pub_pixel_data(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = FloatList()

        #Only publish the pixel if it's in the rectangle
        if 80 <= x <= 560 and 80 <= y <= 400:
            src = np.array([[[x, y]]], dtype = np.float64)  #src is input pixel coordinates

            #undistortPoints() returns a 3D array of the projection of the pixel, to the image sensor
            undistortedPixel = cv2.undistortPoints(src,camMatrix,distCoeff)

            #multiply the projection by the focal length and then add the offset to convert back to pixels
            undistortedPixel1 = undistortedPixel[0][0][0]*fx + ox
            undistortedPixel2 = undistortedPixel[0][0][1]*fy + oy

            #the new undistorted pixel values
            x_new = undistortedPixel1
            y_new = undistortedPixel2

            #populate the refPt
            refPt.data = [x_new,y_new]

            #rospy.loginfo(refPt)
            pub.publish(refPt)
        else:
            pass

#setup an OpenCV window and set mouse Callback
cv2.namedWindow('spotter_cam')
cv2.setMouseCallback('spotter_cam', click_and_pub_pixel_data)
#create a video capture object
cap = cv2.VideoCapture(0)

#define the main talker function
def talker():
    global pub
    pub = rospy.Publisher('pixel_data', FloatList, queue_size=10)
    rospy.init_node('pixel_publisher', anonymous=True)
    rate = rospy.Rate(30) #approximate "frame rate"
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # draw a 'click in this region only' rectangele on the frame
        cv2.rectangle(frame, (80,80), (560,400), (0,0,255), 2)
        cv2.imshow('spotter_cam', frame)
        cv2.waitKey(1)
        rate.sleep()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
