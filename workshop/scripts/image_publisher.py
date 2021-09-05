#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# StopSign class definition
class ImagePub():
    def __init__(self):
        """Publish an image to the camera topic"""

        # Define detection publishers
        self.pub_image = rospy.Publisher('/camera/image_raw', Image,
                                         queue_size=1)

        # Define ROS rate
        self.rate = rospy.Rate(30)

        # Read the image
        fname = '/home/gderose/Pictures/Webcam/2021-06-20-082801.jpg'
        fname = '/home/gderose/Pictures/Webcam/2021-06-20-082302.jpg'
        img = cv.imread(fname)

        # Set parameters
        br = CvBridge()

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Publish image
            self.pub_image.publish(br.cv2_to_imgmsg(img, "bgr8"))

            # Sleep for time step
            self.rate.sleep()
            
        return

    

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('image_publisher_node')
    print("Image publisher node initialized")
    
    # Start tester
    try:
        ImagePub()
    except rospy.ROSInterruptException:
        pass



    
