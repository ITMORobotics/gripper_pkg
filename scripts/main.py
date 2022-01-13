#!/usr/bin/env python3
import gripper_pkg

if __name__ == "__main__":
    gp_sv = gripper_pkg.GripperService()
    gp_sv.handle_control_message_server()
