#!/usr/bin/env python 

import rospy
import tf
import serial
from math import cos, sin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



rospy.init_node('motor_driver_node')



current_time = rospy.Time.now()
last_time = rospy.Time.now()


prev_message = [0, 0]

class driver:
    def __init__(self,ser):
        self.ser=ser
        rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)

    def get_cmd_vel(self, data):
        x = data.linear.x
        angular = data.angular.z
        self.send_cmd_to_motorcontrol(x, angular)

    # translate x, and angular velocity to PWM signal of each wheels, and send to motor control
    def send_cmd_to_motorcontrol(self, x, angular):
        # calculate right and left wheels' signal
        go = int(x * 250)
        turn = int(angular * 125)
        # format for Roboteq controller
        sendgo = '!g 1 {}_'.format(go)
        sendturn = '!g 2 {}_'.format(turn)

        #print message
        print(sendgo)
        print(sendturn)

        # send by serial
        self.ser.write(sendgo)
        self.ser.write(sendturn)



def read_controller():
    message = ser.read_until('\r\r')
    message = message.replace('\r\r', '')
    message = message.replace('\r', '')
    message = message.replace('\n', '')
    message = message.replace(',', '')
    message = message.split("CB=") #lots of cleaning before reading
    message = message[1].split(":")        
    message = [int(message[0]), int(message[1])] 
    print(message)   
    return message



ser = serial.Serial('/dev/ttyACM0', 115200)
ser.write("# c\r")                      # Clear buffer
ser.write("?CB\r")                      # select CB for hall sensor or C for encoder
ser.write("# 10\r")                     # read data every 10ms
ser.write("!CB 1 0_!CB 2 0\r")          # set counter to 0



try:
    d = driver(ser)
except rospy.ROSInterruptException: 
    pass



while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    
    message = read_controller()
    print('message')
    print(message)
        


