#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from dataspeed_ulc_msgs.msg import UlcReport

# Joystick mapping
JOY_BUTTON_A = 0 # Go backward 
JOY_BUTTON_B = 1 # Go forward and turn right
JOY_BUTTON_X = 2 # Go forward and turn left
JOY_BUTTON_Y = 3 # Go forward
JOY_BUTTON_L = 4
JOY_BUTTON_R = 5 # Enable tigger

JOY_AXES_LEFT_STICK_LR = 0
JOY_AXES_LEFT_STICK_UD = 1
JOY_AXES_RIGHT_STICK_LR = 3
JOY_AXES_RIGHT_STICK_UD = 4
JOY_AXES_CROSS_LR = 6
JOY_AXES_CROSS_UD = 7

JOY_TRIGGER_L = 2
JOY_TRIGGER_R = 5

# Vehicle states
STATES = ['Stop', 'Forward','Reverse', 'Left', 'Right']

# GemDrive class definition
class GemDrive():
    def __init__(self):
        """GemDrive to enable joystick GEM driving"""

        # Set joystick enable flag
        self.joy_enable = False
        
        # Define twist message
        self.msg_twist_cmd = Twist()
        
        # Set gains - set these externally with parameters
        self.linear_x_gain = 0.75
        self.steering_gain = 0.1
        self.desired_dist = 2.0
        
        # Initialize commands
        self.initialize_commands()

        # Initialize distance
        self.distance = 0.0
        self.prev_time = rospy.Time.now()

        # Define publishers
        self.pub_enable_cmd =  rospy.Publisher('/vehicle/enable',
                                               Empty, queue_size=1)
        
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
            
            # If joy enable not pressed, initialize state and speed
            #if( not self.joy_enable):
            #    self.initialize_commands()

            # Call state machine
            self.state_machine()
            
            # Publish enable command
            msg = Empty()
            self.pub_enable_cmd.publish(msg)

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
            #  - Set inputs from joystick input
            self.joy_enable = True

            # Check for input
            if( data.buttons[JOY_BUTTON_A] ):
                self.vehicle_req_state = STATES.index('Reverse')
            if( data.buttons[JOY_BUTTON_B] ):
                self.vehicle_req_state = STATES.index('Left')
            if( data.buttons[JOY_BUTTON_X] ):
                self.vehicle_req_state = STATES.index('Right')
            if( data.buttons[JOY_BUTTON_Y] ):
                self.vehicle_req_state = STATES.index('Forward')

        else:
            # Sytem disabled
            #  - Zero out messages to ensure no input
            self.initialize_commands()
            self.joy_enable = False

        return

    

    def initialize_commands(self):
        """ Initialize twist commands"""

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

        else:
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
                
            ##
            ## Reverse State
            ##
            if( self.vehicle_state == STATES.index('Reverse') ):
                rospy.loginfo(' ==>> REVERSE <<==')
                
                # Initialize state
                if( self.state_started == False ):
                    self.state_started = True
                    self.start_position = self.distance

                # Drive backward until target distance is reached
                if( self.distance - self.start_position > -self.desired_dist ):
                    self.msg_twist_cmd.linear.x = -self.linear_x_gain
                    self.msg_twist_cmd.angular.z = 0.0
                else:
                    self.initialize_commands()

            ##
            ## Left & Right States
            ##
            if( self.vehicle_state == STATES.index('Left') or
                self.vehicle_state == STATES.index('Right') ):
                rospy.loginfo(' ==>> LEFT/RIGHT <<==')

                # Set the steer direction
                steer_sign = 1
                if( self.vehicle_state == STATES.index('Left')):
                    steer_sign = -1

                # Initialize state
                if( self.state_started == False ):
                    self.state_started = True
                    self.start_position = self.distance

                # Drive forward and then turn
                if( self.distance - self.start_position < 0.25*self.desired_dist ):
                    self.msg_twist_cmd.linear.x = self.linear_x_gain
                    self.msg_twist_cmd.angular.z = 0.0
                elif( self.distance - self.start_position < self.desired_dist ):
                    self.msg_twist_cmd.linear.x = self.linear_x_gain
                    self.msg_twist_cmd.angular.z = steer_sign * self.steering_gain
                else:
                    self.initialize_commands()
        return

    
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
