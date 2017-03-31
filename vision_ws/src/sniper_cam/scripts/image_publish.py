#! /usr/bin/env python			

##Simple publisher that publishes ROS Image data
## Publishes to the 'interop_images' topic
## Gmoney AUVSI '17
	
## TO DO:
## Publish final images as a ROS topic that interop can use
	
	
import cv2
import numpy as np					
import rospy
import roslib
import sys
import numpy as np					
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sniper_cam.msg import interopImages



from cv_bridge import CvBridge, CvBridgeError		

def image_transfer():

	#Publish to 'interop_images'
	pub = rospy.Publisher('plans', interopImages, queue_size =  10) 	
	bridge = CvBridge()
	msg = interopImages()

	rospy.init_node('death_star', anonymous=True)
	rate = rospy.Rate(1)					#One image/sec

#	with open('Path/to/file', 'r') as content_file:
#	    content = content_file.read()

#	with open('Read In/target_color.txt', 'r') as f:
#   	    for line in f:
#	        target_color = line.split()
        	

##	f1 = open("Read In/image.jpg")				
	file = open("Read In/lat.txt")				
#	f3 = open("Read In/longi.txt")				
#	f4 = open("Read In/target_color.txt")			
#	f5 = open("Read In/target_shape.txt")			
#	f6 = open("Read In/symbol.txt")				
#	f7 = open("Read In/symbol_color.txt")			
#	f8 = open("Read In/orientation.txt")			

##	image = f1.read					
	lat = file.readline()				
	longi = file.readline()				
	target_color = file.readline()			
	target_shape = file.readline()			
	symbol = file.readline()
	symbol_color = file.readline()
	orientation = file.readline()			

	image = cv2.imread("Read In/image.jpg")
	try:
	    image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
	except CvBridgeError as e:
	    print(e)

#	msg.image = image_msg
	msg.gps_lati = float(lat)
	msg.gps_longit = float(longi)
	msg.target_color = target_color
	msg.target_shape = target_shape
	msg.symbol = symbol
	msg.symbol_color = symbol_color
	msg.orientation = orientation
	
	#print(type(float(lat)))
	#print(msg)
	

	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
		try:
			image_transfer()
		except rospy.ROSInterruptException: pass
	
	
