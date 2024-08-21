#!/usr/bin/env python3

import can
from innotrac_msgs.msg import SCmd1

def send_scmd1(bus, auto_mode_req_value):
    
    can_data = [auto_mode_req_value]

    can_msg = can.Message(
        arbitration_id = 257,  # decimal of 0x101
        data = can_data,
        is_extended_id = False
    )
    try:
        bus.send(can_msg)
        print(f"SCmd1 send:{can_data[0]}")
    except can.CanError as e:
        print(f"Failed to send SCmd1:{e}")
