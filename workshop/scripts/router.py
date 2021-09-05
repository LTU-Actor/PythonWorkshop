#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dataspeed_ulc_msgs.msg import UlcCmd
from dataspeed_ulc_msgs.msg import UlcReport
       

# Router class definition
class Router():
    def __init__(self):
        """Class to send velocity commands to the simulator or vehicle"""

        # Set vehicle or simulation flag
        if rospy.has_param('vehicle_flag'):
            self.vehicle_flag = rospy.get_param('vehicle_flag')
        else:
            rospy.logerr('Must define parameter: vehicle_flag')
            sys.exit(0)
        
        
        # Define publishers
        self.pub_sim_twist = rospy.Publisher('/simulator/cmd_vel',
                                             Twist, queue_size=1)
        
        self.pub_enable_cmd = rospy.Publisher('/vehicle/enable',
                                              Empty, queue_size=1)
        
        self.pub_ulc_cmd = rospy.Publisher('/vehicle/ulc_cmd',
                                           UlcCmd, queue_size=1)

        self.pub_ulc_report = rospy.Publisher('/ulc_report',
                                              UlcReport, queue_size=1) 

        # Define subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.twist_callback,
                         queue_size=1)

        rospy.Subscriber("/odom", Odometry, self.sim_odom_callback,
                         queue_size=1)

        rospy.Subscriber("/vehicle/ulc_report", UlcReport,
                         self.ulc_report_callback, queue_size=1)


        # Define ROS rate
        self.rate = rospy.Rate(20)

        # Spine
        rospy.spin()
            
        return
       

    ##################
    # Twist callback
    ##################
    def twist_callback(self, msg):
        """Publish input twist message to simulator and vehicle"""

        # Publish simulator message
        self.pub_sim_twist.publish(msg)

        # Build the vehicle message
        ulc_cmd = UlcCmd()

        # Fill the twist message information
        ulc_cmd.linear_velocity = msg.linear.x
        ulc_cmd.yaw_command = msg.angular.z
        ulc_cmd.steering_mode = UlcCmd.YAW_RATE_MODE
        ulc_cmd.steering_mode = UlcCmd.CURVATURE_MODE

        # Set other fields to default values
        ulc_cmd.clear = False
        ulc_cmd.enable_pedals = True
        ulc_cmd.enable_shifting = True
        ulc_cmd.enable_steering = True
        ulc_cmd.shift_from_park = False
        ulc_cmd.linear_accel = 0
        ulc_cmd.linear_decel = 0
        ulc_cmd.angular_accel = 0
        ulc_cmd.lateral_accel = 0

        # Create and publish enable message
        enable_msg = Empty()
        self.pub_enable_cmd.publish(enable_msg)
        
        # Publish the ULC message
        self.pub_ulc_cmd.publish(ulc_cmd)

        return



    #################
    # Odom Callback
    #################
    def sim_odom_callback(self, data):
        """Callback function to publish the ULC report based on odom"""
        
        # ==>> ONLY USE FOR SIMLATION RUNS <<==
        if( self.vehicle_flag == False ):
            msg = UlcReport()
            msg.pedals_enabled = False
            msg.steering_enabled = False
            msg.speed_preempted = False
            msg.steering_preempted = False
            msg.override_latched = False
            msg.timeout = False

            msg.steering_mode = 0
            msg.tracking_mode = 1

            msg.speed_ref = data.twist.twist.linear.x
            msg.speed_meas = data.twist.twist.linear.x
            msg.accel_ref = 0
            msg.accel_meas = 0
            msg.max_steering_angle = 0
            msg.max_steering_vel = 0
            
            self.pub_ulc_report.publish(msg)
        
        return


    #######################
    # Ulc report Callback
    #######################
    def ulc_report_callback(self, data):
        """Callback to republish the ulc report"""

        # ==>> ONLY USE FOR VEHICLE RUNS <<==
        if( self.vehicle_flag ):
            self.pub_ulc_report.publish(data)

        return
        

#################
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('router_node')
    print("Router node initialized")
    
    # Start tester
    try:
        Router()
    except rospy.ROSInterruptException:
        pass
