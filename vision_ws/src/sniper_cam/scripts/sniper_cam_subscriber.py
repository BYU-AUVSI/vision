#! /usr/bin/python

## Simple subscriber that subscribes to ROS Image data
## Subscribes to the 'sniper_image' topic
## Westley Barragan and Jesse Wynn AUVSI '17

## TO DO:
## -figure out how we want to save the images
## -maybe subscribe to compressed image data??

import rospy
import os.path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# create CvBridge object
bridge = CvBridge()
global_var={'image_n':0}
globals().update(global_var)
def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Display the image
        cv2.imshow('sniper_cam', cv2_img)
        cv2.waitKey(1)
        # Save the image
        cv2.imwrite('images/image' + str(image_n)+ '.jpeg', cv2_img)
	global image_n
	image_n +=1

def image_subscriber():
    rospy.init_node('sniper_image_subscriber')
    # Subscribe to 'sniper_image' topic and define its callback
    rospy.Subscriber('sniper_image', Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    image_subscriber()
    imagedir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'images')
    if not os.path.isdir(imagedir):
	os.makedires(imagedir)
