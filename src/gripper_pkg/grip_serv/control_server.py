#!/usr/bin/env python3 
from __future__ import print_function

from .gripper_controller import GripperSerialController
import datetime
import logging
from gripper_pkg.srv import control
import rospy


class GripperService:
    
    def open(self):
        self.gripper.open()
    
    def close(self):
        self.gripper.close()

    def __init__(self):
        self.gripper = GripperSerialController('/dev/ttyACM0', 57600)
        self.pseudo_switch = {"110":self.open, "111":self.close}

    def handle_control_message(self, request):
        log_string = "Message recieved at %s" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:$S")
        rospy.loginfo(log_string)
        print("Request")
        print(request.operation_type)
        print(request.speed)
        print(request.position)
        self.pseudo_switch['%d'%request.operation_type]()
        return "Operation %d ended"%request.operation_type

    def handle_control_message_server(self):
        rospy.init_node('gripper_control_node')
        service = rospy.Service('control_gripper', control, self.handle_control_message)
        log_string = "service started at %s" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:$S")
        rospy.loginfo(log_string)
        rospy.spin()
        
if __name__ == "__main__":
    handle_control_message_server()
