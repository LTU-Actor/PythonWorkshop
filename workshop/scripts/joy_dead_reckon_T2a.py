#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from dataspeed_ulc_msgs.msg import UlcReport

# Joystick mapping
JOY_BUTTON_Y = 3 # Go forward
JOY_BUTTON_R = 5 # Enable tigger

# Vehicle states
STATES = ['Stop', 'Forward' ]

# GemDrive class definition
class GemDrive():
    def __init__(self):
        """GemDrive to enable joystick GEM driving"""

        # Set joystick enable flag
        self.joy_enable = False
        
        # Define twist message
        self.msg_twist_cmd = Twist()
        
        # Set gains - set these externally with parameters
        self.linear_x_gain = 1.0
        self.steering_gain = 0.0
        self.desired_dist = 3.0
        
        # Initialize state and commands
        self.initialize_commands()
        
        # Initialize distance
        self.distance = 0.0
        self.prev_time = rospy.Time.now()

        # Define the vehicle velocity command publisher
        self.pub_twist_cmd = rospy.Publisher('/cmd_vel',
                                             Twist, queue_size=1)

        # Define ULC subscriber
        self.sub_ulc_report =  rospy.Subscriber('/ulc_report',
                                                UlcReport,
                                                self.ulc_report_callback)

        # Define joystick subscriber
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        
        # Define ROS rate
        self.rate = rospy.Rate(20)

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            
            # Call state machine
            self.state_machine()
            
            # Publish twist command
            self.pub_twist_cmd.publish(self.msg_twist_cmd)
                
            # Sleep for time step
            self.rate.sleep()
            
        return
       

    #################
    # ULC  Callback
    #################
    def ulc_report_callback(self, data):
        """Callback function to compute the distance traveled"""

        curr_time = rospy.Time.now()
        dt = curr_time - self.prev_time
        self.distance = self.distance + data.speed_meas*dt.to_sec()
        self.prev_time = curr_time
        
        return


    #####################
    # Joystick callback
    #####################
    def joy_callback(self, data):
        """Populate twist message contents with joystick inputs"""

        # Check the enable button
        if( data.buttons[JOY_BUTTON_R] == 1 ):
            # System enabled
            self.joy_enable = True

            # Check for input
            if( data.buttons[JOY_BUTTON_Y] ):
                self.vehicle_state = STATES.index('Forward')

        else:
            self.vehicle_state = STATES.index('Stop')
        return

    
    def state_machine(self):

        ##
        ## Stop State
        ##
        if( self.vehicle_state == STATES.index('Stop') ):
            rospy.loginfo(' ==>> STOP <<==')
            self.state_started = False
            if( self.vehicle_req_state != STATES.index('Stop') ):
                self.vehicle_state = self.vehicle_req_state
            else:
                self.initialize_commands()

        ##
        ## Forward State
        ##
        if( self.vehicle_state == STATES.index('Forward') ):
            rospy.loginfo(' ==>> FORWARD <<==')
                
            # Initialize state
            if( self.state_started == False ):
                self.state_started = True
                self.start_position = self.distance

            # Drive forward until target distance is reached
            if( self.distance - self.start_position < self.desired_dist ):
                self.msg_twist_cmd.linear.x = self.linear_x_gain
                self.msg_twist_cmd.angular.z = 0.0
            else:
                self.initialize_commands()
                
        return


    def initialize_commands(self):
        """Initialize twist commands and state machine"""

        # Initialize vehicle state
        self.vehicle_state = STATES.index('Stop')
        self.vehicle_req_state = STATES.index('Stop')
        self.state_started = False

        # Twist command
        self.msg_twist_cmd.linear.x = 0.0
        self.msg_twist_cmd.linear.y = 0.0
        self.msg_twist_cmd.linear.z = 0.0
        self.msg_twist_cmd.angular.x = 0.0
        self.msg_twist_cmd.angular.y = 0.0
        self.msg_twist_cmd.angular.z = 0.0

#################
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('joy_dead_reckon_node')
    print("GEM joy dead reckon node initialized")
    
    # Start tester
    try:
        GemDrive()
    except rospy.ROSInterruptException:
        pass
