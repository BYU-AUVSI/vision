#! /usr/bin/python

## Simple subscriber that subscribes to ROS Image data
## Subscribes to the 'sniper_image' topic
## Westley Barragan and Jesse Wynn AUVSI '17

## TO DO:
## -figure out how we want to save the images
## -maybe subscribe to compressed image data??

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

VERBOSE = False
# create CvBridge object
bridge = CvBridge()

def image_callback(msg):

    if VERBOSE:
        print 'recieved image of type: "%s"' % msg.format

    # direct conversion to CV2

    np_arr = np.fromstring(msg.data, np.uint8)
    img_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    cv2.imshow('recieved image', img_np)
    cv2.waitKey(1)

    # Feature detectors using 

    # try:
    #     # Convert your ROS Image message to OpenCV2
    #     cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # except CvBridgeError, e:
    #     print(e)
    # else:
    #     # Display the image
    #     cv2.imshow('sniper_cam', cv2_img)
    #     cv2.waitKey(1)
    #     # Save the image
    #     #cv2.imwrite('camera_image.jpeg', cv2_img)

def image_subscriber():
    rospy.init_node('sniper_image_subscriber')
    # Subscribe to 'sniper_image' topic and define its callback
    rospy.Subscriber('sniper_cam/image/compressed', CompressedImage, image_callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    image_subscriber()
