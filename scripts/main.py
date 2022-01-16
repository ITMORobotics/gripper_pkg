#!/usr/bin/env python3
import gripper_pkg.grip_serv as gp_pk

if __name__ == "__main__":
    gp_sv = gp_pk.GripperService()
    gp_sv.handle_control_message_server()
