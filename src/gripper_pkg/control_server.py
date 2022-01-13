#!/usr/bin/env python3 

from __future__ import print_function

from gripper_pkg.srv import control
import rospy


class GripperService:

    def __init__(self):
        pass

    def handle_control_message(self, message):
        log_string = "Message recieved at %s" %rospy.get_time()
        rospy.loginfo(log_string)
        print("Message")
        print(message.operation_type)
        print(message.speed)
        print(message.position)

    def handle_control_message_server(self):
        rospy.init_node('gripper_control_node')
        service = rospy.Service('control_gripper', control, self.handle_control_message)
        log_string = "service started at %s" %rospy.get_time()
        rospy.loginfo(log_string)
        rospy.spin()

if __name__ == "__main__":
    handle_control_message_server()
