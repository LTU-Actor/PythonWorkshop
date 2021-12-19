#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from igvc_python.cfg import LaneCenterDynCfgConfig

ACTIVE_WINDOWS = []

# Using algorithm based on https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96


# Lane Centering  class definition
class LaneCenter():
    def __init__(self):
        """Use the forward facing camera lane centering"""

        # Define the lane center desired yaw rate
        self.yaw_rate = 0.0

        # Define the lane flags
        self.left_lane_line_vis = False
        self.right_lane_line_vis = False
        
        # Initialize dynamic configure
        self.dyn_config = []
        self.debug = False
        self.img_threshold = 255
        self.window_width = 0.5
        self.window_height = 0.5
        self.x_pos = 0.5
        self.y_pos = 0.5

        # Initialize the smoothing
        self.smooth_factor = 0.9
        self.yaw_rate_prev = 0.0

        
        # Define lane center steering publisher
        self.pub_lane_center = rospy.Publisher('/lane_center',
                                               Float32, queue_size=1)

        # Define lane center left & right lane status flags
        self.pub_left_lane_flag = rospy.Publisher('/lane_center_left_lane',
                                                  Bool, queue_size=1)
        self.pub_right_lane_flag = rospy.Publisher('/lane_center_right_lane',
                                                   Bool, queue_size=1)

        # Define the image subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image,
                                          self.camera_callback)

        # Set up dynamics reconfigure
        self.srv = Server(LaneCenterDynCfgConfig, self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Build message yaw rate message and publish
            msg = Float32()
            msg.data = self.yaw_rate
            self.pub_lane_center.publish(msg)

            # Build message left lane flag and publish
            msg = Bool()
            msg.data = self.left_lane_line_vis
            self.pub_left_lane_flag.publish(msg)

            # Build message right lane flag and publish
            msg = Bool()
            msg.data = self.right_lane_line_vis
            self.pub_right_lane_flag.publish(msg)

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


    ##################
    # Helper function
    ##################
    def make_points(self, frame, line):
        height, width = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]


    ###############################################################
    # Function to find left and right lane lines from segment data
    ###############################################################
    def average_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        slope_threshold = 0.1
        left_lane_line = []
        right_lane_line = []
        center_lane_line = []
        if line_segments is None:
            rospy.loginfo('No line_segment segments detected')
            return (left_lane_line, right_lane_line, center_lane_line)

        height, width = frame.shape
        left_fit = []
        right_fit = []
        left_fit_2 = []
        right_fit_2 = []

        # left lane line segment should be on left 2/3 of the screen
        # right lane line segment should be on left 2/3 of the screen
        boundary = 1/3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary 
        left_lines_len = np.array([])
        right_lines_len = np.array([])
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                dist = np.sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) )
                if( abs(slope) > slope_threshold ):
                    if slope < 0:
                        if x1<left_region_boundary and x2<left_region_boundary:
                            left_fit.append((slope, intercept))
                            left_fit_2.append((slope, intercept,dist))
                            left_lines_len = np.append(left_lines_len,dist)
                    else:
                        if x1>right_region_boundary and x2>right_region_boundary:
                            right_fit.append((slope, intercept))
                            right_fit_2.append((slope, intercept,dist))
                            right_lines_len = np.append(right_lines_len,dist)

        # Standard average
        #left_fit_average = np.average(left_fit, axis=0)
        #if len(left_fit) > 0:
        #    left_lane_line.append(self.make_points(frame, left_fit_average))

        #right_fit_average = np.average(right_fit, axis=0)
        #if len(right_fit) > 0:
        #    right_lane_line.append(self.make_points(frame, right_fit_average))


        # Use line length weighted average to determine average lane lines
        if( len(left_lines_len) > 0 ):
            left_fit_average = np.average(left_fit, weights=left_lines_len,
                                          axis=0)
            left_lane_line.append(self.make_points(frame,
                                                   (left_fit_average[0],
                                                    left_fit_average[1])))


        if( len(right_lines_len) > 0 ):
            right_fit_average = np.average(right_fit,weights=right_lines_len,
                                           axis=0)
            right_lane_line.append(self.make_points(frame,
                                                    (right_fit_average[0],
                                                     right_fit_average[1])))

        if(False):
            if len(left_fit) > 0:
                m_weighted_sum = 0
                b_weighted_sum = 0
                dist_sum = 0
                for m, b, dist in left_fit_2:
                    m_weighted_sum += dist*m
                    b_weighted_sum += dist*b
                    dist_sum += dist
                    
                m = m_weighted_sum/dist_sum
                b = b_weighted_sum/dist_sum
                
                left_lane_line.append(self.make_points(frame, (m,b)))

        

                if len(right_fit) > 0:
                    m_weighted_sum = 0
                    b_weighted_sum = 0
                    dist_sum = 0
                    for m, b, dist in right_fit_2:
                        m_weighted_sum += dist*m
                        b_weighted_sum += dist*b
                        dist_sum += dist
                    m = m_weighted_sum/dist_sum
                    b = b_weighted_sum/dist_sum

                    right_lane_line.append(self.make_points(frame, (m,b)))

                    
        # Compute center target lane line
        if( len(left_fit) > 0 and len(right_fit) > 0):
            # Case 1: both lines are detected
            self.left_lane_line_vis = True
            self.right_lane_line_vis = True
            m_left = left_fit_average[0]
            b_left = left_fit_average[1]
            m_right = right_fit_average[0]
            b_right = right_fit_average[1]
            y1 = height
            y2 = 0
            x1 = int( ( 1/m_left * (y1 - b_left) + 1/m_right * (y1 - b_right) ) / 2 )
            x2 = int( ( 1/m_left * (y2 - b_left) + 1/m_right * (y2 - b_right) ) / 2 )
            center_lane_line.append([[x1, y1, x2, y2]])

        elif(len(left_fit) > 0):
            # Case 2: only left line present
            self.left_lane_line_vis = True
            self.right_lane_line_vis = False
            m_left = left_fit_average[0]
            b_left = left_fit_average[1]
            y1 = height
            y2 = 0
            x1 = int( ( 1/m_left * (y1 - b_left) + width/2 ) / 2 )
            x2 = int( ( 1/m_left * (y2 - b_left) + width/2 ) / 2 )
            x1 = int( ( 1/m_left * (y1 - b_left) + width/2 ) )
            x2 = int( ( 1/m_left * (y2 - b_left) + width/2 ) )
            #x1 = int( 1/m_left * (height - b_left) + width/2 )
            #x2 = x1

            center_lane_line.append([[x1, y1, x2, y2]])

        elif(len(right_fit) > 0):
            # Case 3: only right line present
            self.left_lane_line_vis = False
            self.right_lane_line_vis = True
            m_right = right_fit_average[0]
            b_right = right_fit_average[1]
            y1 = height
            y2 = 0
            #x1 = int( ( 1/m_right * (y1 - b_right) - width/2 ) / 2 )
            #x2 = int( ( 1/m_right * (y2 - b_right) - width/2 ) / 2 )
            x1 = int( ( 1/m_right * (y1 - b_right) - width/2 ) )
            x2 = int( ( 1/m_right * (y2 - b_right) - width/2 ) )
            #x1 = int( 1/m_right * (height - b_right) - width/2 )
            #x2 = x1
            center_lane_line.append([[x1, y1, x2, y2]])
    
            #center_lane_line.append(self.make_points(frame, center_fit_average))
            
        return (left_lane_line, right_lane_line, center_lane_line)



    ##############################
    # Function to plot lane lines
    ##############################
    def display_lines(self, frame, left_lane_line, right_lane_line, center_lane_line, line_width=2):
        #line_image = np.zeros_like(frame)
        line_image = frame.copy()
        line_color = (255, 0, 0)
        center_color = (0, 0, 255)

        if left_lane_line is not None:
            for line in left_lane_line:
                for x1, y1, x2, y2 in line:
                    cv.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

        if right_lane_line is not None:
            for line in right_lane_line:
                for x1, y1, x2, y2 in line:
                    cv.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

        if center_lane_line is not None:
            for line in center_lane_line:
                for x1, y1, x2, y2 in line:
                    cv.line(line_image, (x1, y1), (x2, y2), center_color, line_width)
        
        return line_image

    #########################
    # Camera image callback
    #########################
    def camera_callback(self, rgb_msg):

        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        self.display_image('Lane Center: Camera View - Orig', img, self.debug)

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
        self.display_image('Lane Center: Camera View - Mask', img_mask, self.debug)
        
        # Convert image to a grayscale image
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_gray = cv.bitwise_and(img_gray, img_mask)
        self.display_image('Lane Center: Camera View - Gray', img_gray, self.debug)
        
        # Apply threshold to the image
        img_thres = cv.inRange(img_gray, self.img_threshold, 255)
        self.display_image('Lane Center: Camera View - Gray Thres', img_thres,
                           self.debug)

        
        # Get the left and right lane lines
        # Detect edges
        edges = cv.Canny(img_thres, 200, 400)

        # Hough Line Transform
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv.HoughLinesP(edges, rho, angle,
                                       min_threshold, np.array([]),
                                       minLineLength=10, maxLineGap=10)

        # If no lanes are found, go straight
        if( line_segments is None ):
            self.yaw_rate = 0.0
            return
            
        # Debug 
        img_tmp = img.copy()
        offset = 0
        for line in line_segments:
            for x1,y1,x2,y2 in line:
                cv.line(img_tmp,(x1,y1),(x2,y2),(0,80+offset,0),5)
                offset +=20
        self.display_image('Hough Transfer', img_tmp, self.debug)   
            
    
        # Get the left and right lane lines
        left_lane_line, right_lane_line, center_lane_line = self.average_slope_intercept(img_thres, line_segments)

        # Display the lane lines
        img_lanes = self.display_lines(img, left_lane_line, right_lane_line, center_lane_line, 5)
        self.display_image('Lane Center: Lane Lines', img_lanes, self.debug)        

        # Construct line to follow
        img_center_line = np.zeros_like(img_thres)
        if center_lane_line is not None:
            for line in center_lane_line:
                for x1, y1, x2, y2 in line:
                    cv.line(img_center_line, (x1, y1), (x2, y2), 255, 5)

        self.display_image('Lane Center: Center Lane', img_center_line, self.debug)
        
        # Compute the centroid of the masked threshold image
        M = cv.moments(img_center_line)
        m10 = M["m10"]
        m00 = M["m00"]
        if( m00 != 0 ):
           x_centroid = int(m10/m00)
        else:
           x_centroid = int(cols/2)

        # Get the new steer angle
        new_steer = -(x_centroid - cols/2)/(1.0*cols)

        # Smooth the input
        self.yaw_rate = self.yaw_rate_prev * self.smooth_factor \
            + new_steer * ( 1 - self.smooth_factor)

        self.yaw_rate = new_steer
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
    rospy.init_node('lane_center_node')
    print("Lane Center node initialized")
    
    # Start tester
    try:
        LaneCenter()
    except rospy.ROSInterruptException:
        pass



    
