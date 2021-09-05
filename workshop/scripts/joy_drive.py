#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from dbw_polaris_msgs.msg import BrakeCmd
from dbw_polaris_msgs.msg import ThrottleCmd
from dbw_polaris_msgs.msg import SteeringCmd
       

# Joystick mapping
JOY_BUTTON_A = 0
JOY_BUTTON_B = 1
JOY_BUTTON_X = 2
JOY_BUTTON_Y = 3
JOY_BUTTON_L = 4
JOY_BUTTON_R = 5 # Enable tigger

JOY_AXES_LEFT_STICK_LR = 0  # Steering (-1 -> 1)
JOY_AXES_LEFT_STICK_UD = 1
JOY_AXES_RIGHT_STICK_LR = 3
JOY_AXES_RIGHT_STICK_UD = 4 # Throttle / Braking 1 -> -1
JOY_AXES_CROSS_LR = 6
JOY_AXES_CROSS_UD = 7

JOY_TRIGGER_L = 2
JOY_TRIGGER_R = 5  


# GemDrive class definition
class GemDrive():
    def __init__(self):
        """GemDrive to enable joystick GEM driving"""

        # Set joystick enable flag
        self.joy_enable = False
        
        # Define the throttle, braking and steering commands
        self.msg_throttle_cmd = ThrottleCmd()
        self.msg_brake_cmd = BrakeCmd()
        self.msg_steering_cmd = SteeringCmd()
        self.initialize_commands()
        
        # Set gains - set these externally with parameters
        self.throttle_gain = 0.50
        self.brake_gain = 0.50
        self.steering_gain = 0.20

        # Define publishers
        self.pub_throttle_cmd = rospy.Publisher('/vehicle/throttle_cmd',
                                                ThrottleCmd, queue_size=1)
        
        self.pub_brake_cmd = rospy.Publisher('/vehicle/brake_cmd',
                                             BrakeCmd, queue_size=1)
        
        self.pub_steering_cmd = rospy.Publisher('/vehicle/steering_cmd',
                                                SteeringCmd, queue_size=1)
        
        self.pub_enable_cmd =  rospy.Publisher('/vehicle/enable',
                                               Empty, queue_size=1)
        
        self.pub_disable_cmd =  rospy.Publisher('/vehicle/disable',
                                                Empty, queue_size=1)

        # Define subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        

        # Define ROS rate
        self.rate = rospy.Rate(50)

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            # Set enable/disable message
            msg = Empty()
            if( self.joy_enable):
                self.pub_enable_cmd.publish(msg)
            else:
                self.pub_disable_cmd.publish(msg)

            # Publish commands
            self.pub_throttle_cmd.publish(self.msg_throttle_cmd)
            self.pub_brake_cmd.publish(self.msg_brake_cmd)
            self.pub_steering_cmd.publish(self.msg_steering_cmd)

            # Sleep for time step
            self.rate.sleep()
            
        return
       

    #####################
    # Joystick callback
    #####################
    def joy_callback(self, data):
        """Populate throttle, brake and steering messages with
        joystick inputs"""
        
       
        # Check the enable button
        if( data.buttons[JOY_BUTTON_R] != 1.0 ):
            # Sytem disabled
            #  - Zero out messages to ensure no input (may not be needed)
            #
            self.initialize_commands()
            self.joy_enable = False

        else:
            # System enabled
            #  - Set inputs from joystick input
            #
            self.joy_enable = True

            # Check for throttle input
            throttle_cmd = 0.0
            if( data.axes[JOY_AXES_RIGHT_STICK_UD] > 0 ):
                throttle_cmd = self.throttle_gain * \
                               data.axes[JOY_AXES_RIGHT_STICK_UD]
            
            # Check for braking input
            brake_cmd = 0.0
            if( data.axes[JOY_AXES_RIGHT_STICK_UD] < 0 ):
                brake_cmd = -1.0 * self.brake_gain * \
                            data.axes[JOY_AXES_RIGHT_STICK_UD]
            
            # Check for steering input
            steering_cmd = data.axes[JOY_AXES_LEFT_STICK_LR]
            steering_cmd = self.steering_gain * \
                           steering_cmd*SteeringCmd.TORQUE_MAX


            # Update commands
            self.msg_throttle_cmd.pedal_cmd = throttle_cmd
            self.msg_throttle_cmd.enable = True
            self.msg_brake_cmd.pedal_cmd = brake_cmd
            self.msg_brake_cmd.enable = True
            self.msg_steering_cmd.steering_wheel_torque_cmd = steering_cmd
            self.msg_steering_cmd.enable = True

        return

    

    def initialize_commands(self):
        """ Initialize throttle, brake and steer commands"""

        # Throttle
        #   pedal_cmd_type
        #      CMD_NONE
        #      CMD_PEDAL: 0.20 to 0.80
        #      CMD_PERCENT: 0.0 to 1.0
        self.msg_throttle_cmd.pedal_cmd = 0.0
        self.msg_throttle_cmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        self.msg_throttle_cmd.enable = False
        self.msg_throttle_cmd.clear = False
        self.msg_throttle_cmd.ignore = False
        self.msg_throttle_cmd.count = 0

        # Brake
        #   pedal_cmd_type
        #      CMD_PERCENT: 0.0 to 1.0
        #      CMD_TORQUE: 0 to 8000Nm
        #      CMD_TORQUE_RQ: 0 to 8000 Nm, closed-loop
        self.msg_brake_cmd.pedal_cmd = 0.0
        self.msg_brake_cmd.pedal_cmd_type = BrakeCmd.CMD_PERCENT  
        self.msg_brake_cmd.enable = False
        self.msg_brake_cmd.clear = False
        self.msg_brake_cmd.ignore = False
        self.msg_brake_cmd.count = 0

        # Steering
        #   cmd_type
        #     CMD_ANGLE
        #     CMD_TORQUE
        self.msg_steering_cmd.steering_wheel_angle_cmd = 0.0
        self.msg_steering_cmd.steering_wheel_angle_velocity = 0.0
        self.msg_steering_cmd.steering_wheel_torque_cmd = 0.0
        self.msg_steering_cmd.cmd_type = SteeringCmd.CMD_TORQUE 
        self.msg_steering_cmd.enable = False
        self.msg_steering_cmd.clear = False
        self.msg_steering_cmd.ignore = False
        self.msg_steering_cmd.calibrate = False
        self.msg_steering_cmd.quiet = False
        self.msg_steering_cmd.count = 0

        return


#################
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('joy_drive_node')
    print("GEM joy drive node initialized")
    
    # Start tester
    try:
        GemDrive()
    except rospy.ROSInterruptException:
        pass
