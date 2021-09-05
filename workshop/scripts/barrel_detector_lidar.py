#!/usr/bin/env python3

import rospy
import cv2 as cv
import math

from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from dynamic_reconfigure.server import Server
from workshop.cfg import BarrelLidarDynCfgConfig

ACTIVE_WINDOWS = []

# BarrelDetect class definition
class BarrelDetect():
    def __init__(self):
        """Use lidar sensor to detect a barrel (object in specified zone)"""

        # Define the detection flag
        self.barrel_detected = False
        
        # Initialize dynamic configure
        self.dyn_config = []
        self.debug = False
        self.dist_thres = 2.0
        self.x_min =  1.0
        self.x_max = 10.0
        self.y_min = -1.0
        self.y_max =  1.0
        self.z_min = -5.0
        self.z_max =  5.0

        
        # Define detection publishers
        self.pub_detect_barrel = rospy.Publisher('/detect_barrel_lidar',
                                                 Bool, queue_size=1)

        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2,
                                          self.lidar_callback)

        # Set up dynamics reconfigure
        self.srv = Server(BarrelLidarDynCfgConfig, self.dyn_reconfig_callback)

        # Define ROS rate
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
        self.dist_thres = config['dist_thres']
        self.x_min = config['x_min']
        self.x_max = config['x_max']
        self.y_min = config['y_min']
        self.y_max = config['y_max']
        self.z_min = config['z_min']
        self.z_max = config['z_max']
        return config

    
    #########################
    # Camera image callback
    #########################
    def lidar_callback(self, data):
        
        # Check if the barrel is detected
        gen = point_cloud2.read_points(data, field_names=("x", "y", "z"),
                                       skip_nans=True)
    
        closest_object_dist = 100
        for p in gen:
            x = p[0]
            y = p[1]
            z = p[2]
            if( x > self.x_min ):
                if( y > self.y_min and y < self.y_max ):
                    if( z > self.z_min and z < self.z_max ):
                        dist = math.sqrt(x*x + y*y + z*z)
                        if( dist < closest_object_dist ):
                            closest_object_dist = dist

        if( self.debug ):
            rospy.loginfo('Closest Obj = %8.3f' % closest_object_dist)

        self.barrel_detected = False
        if( closest_object_dist < self.dist_thres ):
            self.barrel_detected = True
        return


#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('barrel_lidar_node')
    print("Barrel lidar dectector node initialized")
    
    # Start tester
    try:
        BarrelDetect()
    except rospy.ROSInterruptException:
        pass



    
