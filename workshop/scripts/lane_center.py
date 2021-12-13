#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from workshop.cfg import LaneCenterDynCfgConfig
#LineFollowDynCfgConfig
ACTIVE_WINDOWS = []

# Crosswalk class definition
class LaneCenter():
    def __init__(self):
        """Use the forward facing camera to center in the lane"""

        # Define the lane centering desired yaw rate
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
        self.pub_lane_center = rospy.Publisher('/lane_center',
                                               Float32, queue_size=1)

        # Define the image subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image,
                                          self.camera_callback)

        # Set up dynamics reconfigure
        self.srv = Server(LaneCenterDynCfgConfig, self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Build message
            msg = Float32()
            msg.data = self.yaw_rate
            
            # Publish commands
            self.pub_lane_center.publish(msg)

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
        disp_flag = self.debug

        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        self.display_image('lane center: Camera View - Orig', img, self.debug)

        # Mask the image for the lane region
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
        self.display_image('lane center: Camera View - Mask', img_mask, self.debug)
        
        # Convert image to a grayscale image
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_gray = cv.bitwise_and(img_gray, img_mask)
        self.display_image('lane center: Camera View - Gray', img_gray, self.debug)
        
        # Apply threshold to the image
        img_thres = cv.inRange(img_gray, self.img_threshold, 255)
        self.display_image('lane center: Camera View - Gray Thres', img_thres,
                           self.debug)
        
        # From https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
        
        #detect edges
        edges = cv.Canny(img_thres, 200, 400)
        #isolate region of interest
        h, w = edges.shape
        polygon = np.array([[
                (0, h*0.5),
                (w, h*0.5),
                (w, h),
                (0, h)
                ]], np.int32)
        
        cv.fillPoly(img_thres, polygon, 255)
        cropped_edges = cv.bitwise_and(edges, img_thres)
        
        #detect line segments
        rho = 1
        angle = np.pi / 180
        min_threshold = 10
        line_segments = cv.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength = 8, maxLineGap = 4) #tweak these if necessary
        self.display_image('lane center: Camera View - Cropped Edges', cropped_edges, self.debug)
        
        #Get average slope
        
        lane_lines = []
        frame = cropped_edges
        if line_segments is None:
                if disp_flag:
                        rospy.loginfo('No line_segment segments detected')
                return
        height, width = frame.shape
        left_fit = []
        right_fit = []
        
        boundary = 1/3
        left_bound = width * (1-boundary)
        right_bound = width * boundary
        
        for line_segment in line_segments:
                for x1, y1, x2, y2 in line_segment:
                        if x1 == x2:
                               if disp_flag:
                                        rospy.loginfo(f'skipping vertical line segment (slope=inf): {line_segment}')
                               continue
                        fit = np.polyfit((x1, x2), (y1, y2), 1)
                        slope = fit[0]
                        intercept = fit[1]
                        if slope < 0:
                                if x1 < left_bound and x2 < left_bound:
                                        left_fit.append((slope, intercept))
                        else:
                                if x1 > right_bound and x2 > right_bound:
                                        right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis = 0)
        if len(left_fit) > 0:
                lane_lines.append(self.make_points(frame, left_fit_average))
        right_fit_average = np.average(right_fit, axis = 0)
        if len(right_fit) > 0:
                lane_lines.append(self.make_points(frame, right_fit_average))
        rospy.loginfo(f'lane_lines: {lane_lines}')
        
        ##Get steering angle
        try:
                _, _, left_x2, _ = lane_lines[0][0]
                _, _, right_x2, _ = lane_lines[1][0]
                mid = int(width/2)
                x_offset = (left_x2 + right_x2) / 2 - mid
                y_offset = int(height/2)
                if disp_flag:
                       rospy.logdebug(f'Two segments: {lane_lines[0][0]} and {lane_lines[1][1]}')
        except:
                x1, _, x2, _ = lane_lines[0][0]
                x_offset = x2-x1
                y_offset = int(height/2)
                if disp_flag:
                       rospy.logdebug(f'One segment: {lane_lines[0][0]}')
                
        angle_to_mid = math.atan(x_offset / y_offset)
        if disp_flag:
                rospy.loginfo(f'Angle to middle: {angle_to_mid}')
        self.yaw_rate = angle_to_mid
        return

    
    
    ##Make points - helper for getting the average slope intercept
    def make_points(self, frame, line):
            height, width = frame.shape
            slope, intercept = line
            y1 = height
            y2 = int(y1*0.5)
            x1 = max(-width, min(2*width, int((y1-intercept)/slope)))
            x2 = max(-width, min(2*width, int((y2-intercept)/slope)))
            return [[x1, y1, x2, y2]]
    
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
    rospy.init_node('lane_center_node')
    print("Lane Center node initialized")
    
    # Start tester
    try:
        LaneCenter()
    except rospy.ROSInterruptException:
        pass



    
