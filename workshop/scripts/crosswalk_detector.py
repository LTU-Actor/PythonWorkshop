#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from workshop.cfg import CrosswalkDynCfgConfig

ACTIVE_WINDOWS = []

# Crosswalk class definition
class Crosswalk():
    def __init__(self):
        """Use the forward facing camera to detect a crosswalk"""

        # Define the detection flag
        self.crosswalk_detected = False
        
        # Initialize dynamic configure
        self.dyn_config = []
        self.debug = False
        self.img_threshold = 255
        self.detect_threshold = 1
        self.window_width = 0.5
        self.window_height = 0.5
        self.x_pos = 0.5
        self.y_pos = 0.5
        
        # Define detection publishers
        self.pub_detect_crosswalk = rospy.Publisher('/detect_crosswalk',
                                                    Bool, queue_size=1)

        # Define the image subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image,
                                          self.camera_callback)

        # Set up dynamics reconfigure
        self.srv = Server(CrosswalkDynCfgConfig, self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            
            # Fill message
            msg = Bool()
            msg.data = self.crosswalk_detected
                        
            # Publish commands
            self.pub_detect_crosswalk.publish(msg)

            # Sleep for time step
            self.rate.sleep()
            
        return

    
    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.debug = config['debug']
        self.img_threshold = config['img_threshold']
        self.detect_threshold = config['detect_threshold']
        self.window_width = config['window_width']
        self.window_height = config['window_height']
        self.x_pos = config['x_pos']
        self.y_pos = config['y_pos']
        self.dyn_config = config
        return config

    
    #########################
    # Camera image callback
    #########################
    def camera_callback(self, rgb_msg):

        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        self.display_image('Crosswalk: Camera View - Orig', img, self.debug)
        tmp = img.shape
        orig_rows = tmp[0]
        orig_cols = tmp[1]
        
        # Mask the image for the roi
        img_mask = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_mask[:,:] = 0
        rows, cols = img_mask.shape
        xc = int(rows * self.x_pos)
        yc = int(cols * self.y_pos)
        half_width = int(cols * self.window_width / 2.0)
        half_height = int(rows * self.window_height / 2.0)
        mask_poly = np.array([[ [yc-half_width,xc-half_height],
                                [yc+half_width,xc-half_height],
                                [yc+half_width,xc+half_height],
                                [yc-half_width,xc+half_height] ]])
        cv.fillPoly(img_mask, mask_poly, 255)
        self.display_image('Crosswalk: Camera View - Mask', img_mask,
                           self.debug)

        # Convert image to a grayscale image and apply mask
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_gray = cv.bitwise_and(img_gray, img_mask)
        self.display_image('Crosswalk: Camera View - Gray', img_gray,
                           self.debug)
        
        # Apply threshold to the image
        img_thres = cv.inRange(img_gray, self.img_threshold, 255)
        self.display_image('Crosswalk: Camera View - Gray Thres', img_thres,
                           self.debug)
        
        # Compute the percentage of pixels that are white
        num_pixels = cv.countNonZero(img_mask)
        num_white_pixels = cv.countNonZero(img_thres)
        percent_white = (1.0*num_white_pixels) / num_pixels
        #rospy.loginfo('Orig: %d Stats: %d / %d = %.2f' % (orig_rows*orig_cols,
        #                                                  num_white_pixels,
        #                                                  num_pixels,
        #                                                  percent_white ))

        # Check if the crosswalk is detected
        self.crosswalk_detected = False
        if( percent_white > self.detect_threshold ):
            self.crosswalk_detected = True
        return


    ####################
    # Display an image
    ####################
    def display_image(self, title_str, img, disp_flag ):

        if( disp_flag ):
            # Display the given image
            cv.namedWindow(title_str, cv.WINDOW_NORMAL)
            cv.imshow(title_str, img)
            cv.waitKey(3)

            # Add window to active window list
            if not ( title_str in ACTIVE_WINDOWS ):
                ACTIVE_WINDOWS.append(title_str)
        else:
            if( title_str in ACTIVE_WINDOWS):
                cv.destroyWindow(title_str)
                ACTIVE_WINDOWS.remove(title_str)
        return


#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('crosswalk_detector_node')
    print("Crosswalk dectector node initialized")
    
    # Start tester
    try:
        Crosswalk()
    except rospy.ROSInterruptException:
        pass
