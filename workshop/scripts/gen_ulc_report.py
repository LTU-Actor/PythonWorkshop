#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from dataspeed_ulc_msgs.msg import UlcReport

# GemDrive class definition
class GenUlcMsg():
    def __init__(self):
        """Current simulator Odom message to a ULC message"""


        # Define ULC publisher
        self.pub_ulc_report =  rospy.Publisher('/simulator/ulc_report',
                                               UlcReport, queue_size=1)
        
        # Define odometry subscriber
        self.sub_sim_odom =  rospy.Subscriber('/odom',
                                                Odometry,
                                                self.odom_callback)

        # Define ROS rate
        self.rate = rospy.Rate(50)

        # ROS spin
        rospy.spin()
        
        return

    #################
    # ULC  Callback
    #################
    def odom_callback(self, data):
        """Callback function to publish the ULC report based on odom"""

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


    
#################
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('gen_ulc_report_node')
    print("Gen ULC report node initialized")
    
    # Start tester
    try:
        GenUlcMsg()
    except rospy.ROSInterruptException:
        pass
