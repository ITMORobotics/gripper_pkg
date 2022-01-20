#!/usr/bin/env python3 
from __future__ import print_function

import datetime
import rospy
import sys
import glob
import serial
from .gripper_controller import GripperSerialController
from .gripper_controller import GripperListenerI
from gripper_pkg.srv import control
from std_msgs.msg import String

class GripperPublisher(GripperListenerI):

    def __init__(self):
        self.publisher = rospy.Publisher('from_gripper_info', String, queue_size=10)

    def process_data(self, package: bytes, type_code:int, left_val: float, right_val: float)->None:
        pub_str = "Time: %s | package: %s | type_code: %d | left_val: %f | right_val: %f"%(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), package.hex(":"), type_code, left_val, right_val)
        rospy.loginfo(pub_str)
        self.publisher.publish(pub_str)


class GripperService:
    
    def __init__(self):
        try:
            self.serial_list = self.serial_ports()
            #print(self.serial_list)
            for i in self.serial_list:
                if 'ACM' in i:
                    self.gripper = GripperSerialController(i, 57600)
            #self.gripper = GripperSerialController('/dev/ttyACM0', 57600)
            self.gripper.attach(listener=GripperPublisher())
            self.gripper.start_listening()
        except:
            rospy.loginfo("COM port connection error  %s"%datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        self.pseudo_switch = {"10":self.get_position,"20":self.get_load,"30":self.get_voltage,"40":self.get_temperature, "100":self.release, "101":self.unrelease, "110":self.open, "111":self.close,"121": self.force_close}

    def handle_control_message(self, request):
        log_string = "Message recieved at %s" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rospy.loginfo(log_string)
        request_string = "Request:| Operation type: %d | Speed: %d | Position: %d"%(request.operation_type, request.speed, request.position)
        rospy.loginfo(request_string)
        try:
            self.pseudo_switch['%d'%request.operation_type](request.speed, request.position)
        except:
            rospy.loginfo("While operation error occured %s"%datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
                    
        return "Operation %d ended"%request.operation_type

    def handle_control_message_server(self):
        rospy.init_node('gripper_control_node')
        service = rospy.Service('control_gripper', control, self.handle_control_message)
        log_string = "Service started at %s" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rospy.loginfo(log_string)
        rospy.spin()
    
    def serial_ports(self):
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        else:
            raise EnvironmentError('Unsupported platform')
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except(OSError, serial.SerialException):
                pass
        return result

    def open(self, speed, position):
        self.gripper.open()

    def close(self, speed, position):
        self.gripper.close()

    def force_close(self, speed, position):
        self.gripper.close_torque(speed)
    
    def release(self, speed, position):
        self.gripper.release()

    def unrelease(self, speed, position):
        self.gripper.unrelease()
    
    def get_temperature(self, speed, position):
        self.gripper.get_temp()
    
    def get_voltage(self, speed, position):
        self.gripper.get_voltage()
    
    def get_load(self, speed, position):
        self.gripper.get_load()

    def get_position(self, speed, position):
        self.gripper.get_position()

