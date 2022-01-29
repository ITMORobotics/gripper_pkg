#!/usr/bin/env python3 
from __future__ import print_function

import datetime
import time
import rospy
import sys
import glob
import serial
from .gripper_controller import GripperSerialController
from .gripper_controller import GripperListenerI
from gripper_pkg.srv import control
from std_msgs.msg import String

class GripperPublisher(GripperListenerI):

    def __init__(self, publisher):
        #self.publisher = rospy.Publisher("from_gripper_info", String, queue_size=10)
        self.publisher = publisher
    def process_data(self, package: bytes, type_code:int, left_val: float, right_val: float)->None:
        pub_str = "Time: %s | package: %s | type_code: %d | left_val: %f | right_val: %f"%(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), package.hex(":"), type_code, left_val, right_val)
        #rospy.loginfo(pub_str)
        self.publisher.publish(pub_str)


class GripperService:
    
    def __init__(self):
        rospy.init_node('gripper_control_node')
        self.publisher = rospy.Publisher('from_gripper_info', String, queue_size=10)
        self.gripper_list = {}
        self.gripper_ports = ["/dev/gripper_left", "/dev/gripper_right"]
        for counter, port in enumerate(self.gripper_ports):
            try:
                #rospy.loginfo("Found gripper with id %d \n"%counter)
                self.gripper_list[counter] = GripperSerialController(port, 57600)
                self.gripper_list[counter].attach(listener=GripperPublisher(self.publisher))
                self.gripper_list[counter].start_listening()
            except:
                    continue
            time.sleep(0.1)
            #self.gripper = GripperSerialController('/dev/ttyACM0', 57600)
            #self.gripper.attach(listener=GripperPublisher())
            #self.gripper.start_listening()
        
        self.pseudo_switch = {"10":self.get_position,"20":self.get_load,"30":self.get_voltage,"40":self.get_temperature, "100":self.release, "101":self.unrelease, "110":self.open, "111":self.close,"121": self.force_close}
        rospy.loginfo("Found %d grippers "%len(self.gripper_list))
        if len(self.gripper_list) != 0:
            for key in self.gripper_list.keys():
                rospy.loginfo("ID: %d"%key)

    def handle_control_message(self, request):
        log_string = "Message recieved at %s \n" %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rospy.loginfo(log_string)
        request_string = "Request:| ID:  %d | Operation type: %d | Speed: %d | Position: %d \n"%(request.id, request.operation_type, request.speed, request.position)
        rospy.loginfo(request_string)
        try:
            self.pseudo_switch['%d'%request.operation_type](request.id, request.speed, request.position)
        except:
            rospy.loginfo("While operation error occured %s\n"%datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        
        while not self.gripper_list[request.id].last_move_status:
            rospy.loginfo("Operation is still in progress %s"%datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            time.sleep(1)

        rospy.loginfo("Operation %d ended %s"%(request.operation_type,datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        return "Operation %d ended %s"%(request.operation_type, datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    def handle_control_message_server(self):
        service = rospy.Service('control_gripper', control, self.handle_control_message)
        log_string = "Service started at %s " %datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
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

    def open(self, id,  speed, position):
        self.gripper_list[id].open()

    def close(self, id, speed, position):
        self.gripper_list[id].close()

    def force_close(self, id, speed, position):
        self.gripper_list[id].close_torque(speed)
    
    def release(self, id, speed, position):
        self.gripper_list[id].release()

    def unrelease(self, id,  speed, position):
        self.gripper_list[id].unrelease()
    
    def get_temperature(self, id, speed, position):
        self.gripper_list[id].get_temp()
    
    def get_voltage(self, id, speed, position):
        self.gripper_list[id].get_voltage()
    
    def get_load(self, id, speed, position):
        self.gripper_list[id].get_load()

    def get_position(self, id,  speed, position):
        self.gripper_list[id].get_position()

