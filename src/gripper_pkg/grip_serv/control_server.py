#!/usr/bin/env python3 
from __future__ import print_function

import datetime
import rospy
from .gripper_controller import GripperSerialController
from gripper_pkg.srv import control


class GripperService:
    
    def __init__(self):
        try:
            self.gripper = GripperSerialController('/dev/ttyACM0', 57600)
        except:
            print("COM port unavailable")
        self.pseudo_switch = {"100":self.release, "101":self.unrelease, "110":self.open, "111":self.close}

    def handle_control_message(self, request):
        log_string = "Message recieved at %s" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:$S")
        rospy.loginfo(log_string)
        print("Request")
        print("Operation type: %d"%request.operation_type)
        print("Speed: %d"%request.speed)
        print("Position %d"%request.position)
        try:
            self.pseudo_switch['%d'%request.operation_type]()
        except:
            rospy.loginfo("While operation error occured %s"%datetime.datetime.now().strftime("%Y-%m-%d %H:%M:$S"))
                    
        return "Operation %d ended"%request.operation_type

    def handle_control_message_server(self):
        rospy.init_node('gripper_control_node')
        service = rospy.Service('control_gripper', control, self.handle_control_message)
        log_string = "service started at %s" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:$S")
        rospy.loginfo(log_string)
        rospy.spin()
    
    def open(self):
        self.gripper.open()

    def close(self):
        self.gripper.close()    
    
    def release():
        self.gripper.release()

    def unrelease():
        self.gripper.unrelease()
