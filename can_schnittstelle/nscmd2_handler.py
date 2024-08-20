#!/usr/bin/env python3

import can
from innotrac_msgs.msg import NSCmd2

def send_nscmd2(bus, cmd: NSCmd2):
    can_data = [0] * 8

    # command data into CAN message 
    can_data[0] = cmd.front_pow_lift_hgt & 0xFF
    can_data[1] = (cmd.front_pow_lift_hgt >> 8) & 0xFF
    can_data[2] = cmd.mid_pow_lift_hgt & 0xFF
    can_data[3] = (cmd.mid_pow_lift_hgt >> 8) & 0xFF
    can_data[4] = cmd.rear_pow_lift_hgt & 0xFF
    can_data[5] = (cmd.rear_pow_lift_hgt >> 8) & 0xFF
    
    # control bits (48-56)
    if cmd.front_pow_lift_flt:
        can_data[6] |= (1 << 0)
    if cmd.mid_pow_lift_flt:
        can_data[6] |= (1 << 1)
    if cmd.rear_pow_lift_flt:
        can_data[6] |= (1 << 2)


    # send message
    can_msg = can.Message(
        arbitration_id = 402,  # decimal of 0x192
        data = can_data,
        is_extended_id = False 
    )
    bus.send(can_msg)