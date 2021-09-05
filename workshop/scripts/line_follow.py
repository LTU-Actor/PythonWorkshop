#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from workshop.cfg import LineFollowDynCfgConfig

ACTIVE_WINDOWS = []

# Crosswalk class definition
class LineFollow():
    def __init__(self):
        """Use the forward facing camera to follow a line"""

        # Define the line following desired yaw rate
        self.yaw_rate = 0.0
        
        # Initialize dynamic configure
        self.dyn_config = []
        self.debug = False
        self.img_threshold = 255
        self.window_width = 0.5
        self.window_height = 0.5
        self.x_pos = 0.5
        self.y_pos = 0.5

        
        # Define detection publishers
        self.pub_line_follow = rospy.Publisher('/line_follow',
                                               Float32, queue_size=1)

        # Define the image subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image,
                                          self.camera_callback)

        # Set up dynamics reconfigure
        self.srv = Server(LineFollowDynCfgConfig, self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Build message
            msg = Float32()
            msg.data = self.yaw_rate
            
            # Publish commands
            self.pub_line_follow.publish(msg)

            # Sleep for time step
            self.rate.sleep()
            
        return

    
    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.debug = config['debug']
        self.img_threshold = config['img_threshold']
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
        self.display_image('Line Follow: Camera View - Orig', img, self.debug)

        # Mask the image for the line region
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
        self.display_image('Line Follow: Camera View - Mask', img_mask, self.debug)
        
        # Convert image to a grayscale image
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_gray = cv.bitwise_and(img_gray, img_mask)
        self.display_image('Line Follow: Camera View - Gray', img_gray, self.debug)
        
        # Apply threshold to the image
        img_thres = cv.inRange(img_gray, self.img_threshold, 255)
        self.display_image('Line Follow: Camera View - Gray Thres', img_thres,
                           self.debug)
        
        # Compute the centroid of the masked threshold image
        M = cv.moments(img_thres)
        m10 = M["m10"]
        m00 = M["m00"]
        if( m00 != 0 ):
           x_centroid = int(m10/m00)
        else:
           x_centroid = int(cols/2)
        #rospy.loginfo('Line: %d / %d / %d ' % (cols, xc, x_centroid))

        self.yaw_rate = -(x_centroid - cols/2)/ (1.0*cols)
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
    rospy.init_node('line_follow_node')
    print("Line Follow node initialized")
    
    # Start tester
    try:
        LineFollow()
    except rospy.ROSInterruptException:
        pass



    
