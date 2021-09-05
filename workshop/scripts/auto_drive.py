#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from dynamic_reconfigure.server import Server
from workshop.cfg import AutoDriveDynCfgConfig

# CrossWalkStop class definition
class AutoDrive():
    def __init__(self):
        """Node to drive the simulator or GEM through the robot steering GUI"""

        # Define twist message
        self.msg_twist_cmd = Twist()
        self.msg_twist_cmd.linear.x = 0
        self.msg_twist_cmd.angular.z = 0

        # Initialize twist sources and flag
        self.dyncfg_linear_x = 0
        self.enable = False
        self.enable_crosswalk = False
        self.crosswalk_detected = False
        self.enable_barrel_vision = False
        self.enable_barrel_lidar = False
        self.barrel_vision_detected = False
        self.barrel_vision_lidar = False
        self.enable_line_follow = False
        self.line_follow_yaw_rate = 0.0

        # Define publishers
        self.pub_twist_cmd = rospy.Publisher('/cmd_vel',
                                             Twist, queue_size=1)

        # Define crosswalk subscriber
        rospy.Subscriber('/detect_crosswalk', Bool, self.crosswalk_callback,
                         queue_size=1)

        # Define barrel vision subscriber
        rospy.Subscriber('/detect_barrel_vision', Bool,
                         self.barrel_vision_callback, queue_size=1)

        # Define barrel lidar subscriber
        rospy.Subscriber('/detect_barrel_lidar', Bool,
                         self.barrel_lidar_callback, queue_size=1)

        # Define line follow steer
        rospy.Subscriber('/line_follow', Float32, self.line_follow_callback,
                         queue_size=1)


        # Set up dynamics reconfigure
        self.srv = Server(AutoDriveDynCfgConfig, self.dyn_reconfig_callback)


        # Define ROS rate to DWB default rate
        self.rate = rospy.Rate(20)

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Initialize the twist message
            self.msg_twist_cmd.linear.x = 0.0
            self.msg_twist_cmd.angular.z = 0.0

            # Check for enable flag
            if( self.enable ):
                
                # Fill twist message
                self.msg_twist_cmd.linear.x = self.dyncfg_linear_x
                self.msg_twist_cmd.angular.z = 0.0

                # Check for line follow
                if(self.enable_line_follow):
                    self.msg_twist_cmd.angular.z = self.line_follow_yaw_rate

                # Check for crosswalk
                if(self.enable_crosswalk):
                    if(self.crosswalk_detected):
                        self.msg_twist_cmd.linear.x = 0.0
                        self.msg_twist_cmd.angular.z = 0.0

                # Check for barrel
                if(self.enable_barrel_vision):
                    if(self.barrel_vision_detected):
                        self.msg_twist_cmd.linear.x = 0.0
                        self.msg_twist_cmd.angular.z = 0.0

                if(self.enable_barrel_lidar):
                    if(self.barrel_lidar_detected):
                        self.msg_twist_cmd.linear.x = 0.0
                        self.msg_twist_cmd.angular.z = 0.0

            # Publish
            self.pub_twist_cmd.publish(self.msg_twist_cmd)

            # Sleep for time step
            self.rate.sleep()

        return
    

    ######################
    # Drive GUI Callback
    ######################
    def dyn_reconfig_callback(self, config, level):
        """Callback function to fill twist message from drive GUI"""
       
        # Fill data from GUI
        self.dyncfg_linear_x = config['speed']
        self.enable = config['enable']
        self.enable_crosswalk = config['crosswalk']
        self.enable_barrel_vision = config['barrel_vision']
        self.enable_barrel_lidar = config['barrel_lidar']
        self.enable_line_follow = config['line_follow']

        # Set the barrel location and size
        barrel_x = config['barrel_x']
        barrel_y = config['barrel_y']
        barrel_rad = config['barrel_rad']
        rospy.set_param('/circ0', [barrel_x, barrel_y, barrel_rad])
        
        return config


    ###############################
    # Crosswalk detection callback
    ###############################
    def crosswalk_callback(self, data):
        """Crosswalk detection callback - stop vehicle at crosswalk"""
        if(data.data):
            self.crosswalk_detected = True
        else:
            self.crosswalk_detected = False
        return


    ####################################
    # Barrel vision detection callback
    ####################################
    def barrel_vision_callback(self, data):
        """Barrel vision detection callback"""
        if(data.data):
            self.barrel_vision_detected = True
        else:
            self.barrel_vision_detected = False
        return


    ###################################
    # Barrel lidar detection callback
    ###################################
    def barrel_lidar_callback(self, data):
        """Barrel lidar detection callback"""
        if(data.data):
            self.barrel_lidar_detected = True
        else:
            self.barrel_lidar_detected = False
        return

    
    ########################
    # Line follow callback
    ########################
    def line_follow_callback(self, data):
        """Line follow callback - get desired yaw rate"""

        self.line_follow_yaw_rate = data.data
        return

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('auto_drive_node')
    print("Auto Drive node initialized")
    
    # Start tester
    try:
        AutoDrive()
    except rospy.ROSInterruptException:
        pass



    
