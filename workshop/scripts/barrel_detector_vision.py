#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from workshop.cfg import BarrelDynCfgConfig

ACTIVE_WINDOWS = []

# BarrelDetect class definition
class BarrelDetect():
    def __init__(self):
        """Use the forward facing camera to detect a barrel (orange object)"""

        # Define the detection flag
        self.barrel_detected = False
        
        # Initialize dynamic configure
        self.dyn_config = []
        self.debug = False
        self.detect_threshold = 1
        self.window_width = 0.5
        self.window_height = 0.5
        self.x_pos = 0.5
        self.y_pos = 0.5
        self.h_low = 0
        self.h_high = 179
        self.s_low = 0
        self.s_high = 255
        self.v_low = 0
        self.v_high = 255


        
        # Define detection publishers
        self.pub_detect_barrel = rospy.Publisher('/detect_barrel_vision',
                                                 Bool, queue_size=1)

        # Define the image subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image,
                                          self.camera_callback)

        # Set up dynamics reconfigure
        self.srv = Server(BarrelDynCfgConfig, self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(50)  # Simulator rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            
            # Fill message
            msg = Bool()
            msg.data = self.barrel_detected
                        
            # Publish commands
            self.pub_detect_barrel.publish(msg)

            # Sleep for time step
            self.rate.sleep()
            
        return

    
    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.debug = config['debug']
        self.detect_threshold = config['detect_threshold']
        self.window_width = config['window_width']
        self.window_height = config['window_height']
        self.x_pos = config['x_pos']
        self.y_pos = config['y_pos']
        self.h_low = config['hue_low']
        self.h_high = config['hue_high']
        self.s_low = config['sat_low']
        self.s_high = config['sat_high']
        self.v_low = config['val_low']
        self.v_high = config['val_high']
        self.dyn_config = config
        return config

    
    #########################
    # Camera image callback
    #########################
    def camera_callback(self, rgb_msg):

        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        self.display_image('Barrel: Camera View - Source', img, self.debug)
        
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
        self.display_image('Barrel: Camera View - Mask', img_mask, self.debug)

        # Transform image to HSV
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        # Apply filters and mask
        img_thres_hsv = cv.inRange(img_hsv,
                                   (self.h_low,self.s_low,self.v_low),
                                   (self.h_high,self.s_high,self.v_high))
        img_thres_hsv = cv.bitwise_and(img_thres_hsv, img_mask)

        # Displaya the HSV threshold image
        self.display_image('Barrel: Camera View - HSV', img_thres_hsv,
                           self.debug )
        
        # Compute the percentage of pixels that are white
        (rows, cols) = img_thres_hsv.shape
        num_pixels = cv.countNonZero(img_mask)
        num_white_pixels = cv.countNonZero(img_thres_hsv)
        percent_white = (1.0*num_white_pixels) / num_pixels
        #rospy.loginfo('Stats: %d / %d = %.2f' % (num_white_pixels, num_pixels,
        #                                         percent_white ))

        # Check if the barrel is detected
        self.barrel_detected = False
        if( percent_white > self.detect_threshold ):
            self.barrel_detected = True
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
    rospy.init_node('barrel_vision_node')
    print("Barrel vision dectector node initialized")
    
    # Start tester
    try:
        BarrelDetect()
    except rospy.ROSInterruptException:
        pass



    
