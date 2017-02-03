#! /usr/bin/env python

## Simple publisher that publishes pixel coordinates on mouse click
## Publishes to the 'pixel_data' topic
## Jesse Wynn AUVSI '17

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from click_to_geolocate.msg import IntList

#define mouse callback function to capture and publish pixel data
def click_and_pub_pixel_data(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = IntList()
        refPt.data = [x,y]
        #rospy.loginfo(refPt)
        pub.publish(refPt)

#setup an OpenCV window and set mouse Callback
cv2.namedWindow('spotter_cam')
cv2.setMouseCallback('spotter_cam', click_and_pub_pixel_data)
#create a video capture object
cap = cv2.VideoCapture(0)

#define the main talker function
def talker():
    global pub
    pub = rospy.Publisher('pixel_data', IntList, queue_size=10)
    rospy.init_node('pixel_publisher', anonymous=True)
    rate = rospy.Rate(30) #approximate "frame rate"
    while not rospy.is_shutdown():
        ret, frame = cap.read()
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
