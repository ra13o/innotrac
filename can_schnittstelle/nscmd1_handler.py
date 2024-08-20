#!/usr/bin/env python3

import can
from innotrac_msgs.msg import NSCmd1

def send_nscmd1(bus, cmd: NSCmd1):
    can_data = [0] * 8

    # command data into CAN message 
    can_data[0] = cmd.steer_ang_magn & 0xFF
    can_data[1] = (cmd.steer_ang_magn >> 8) & 0xFF
    can_data[2] = cmd.drv_spd_magn & 0xFF
    can_data[3] = (cmd.drv_spd_magn >> 8) & 0xFF
    can_data[4] = cmd.eng_spd & 0xFF
    can_data[5] = (cmd.eng_spd >> 8) & 0xFF
    
    # control bits (48-56)
    if cmd.steer_dir_le:
        can_data[6] |= (1 << 0)
    if cmd.steer_dir_ri:
        can_data[6] |= (1 << 1)
    if cmd.drv_dir_fwd:
        can_data[6] |= (1 << 2)
    if cmd.drv_dir_bwd:
        can_data[6] |= (1 << 3)
    if cmd.front_pto:
        can_data[6] |= (1 << 4)
    if cmd.rear_pto:
        can_data[6] |= (1 << 5)

    # mission availability status into the last byte
    can_data[7] = 0X01 if cmd.mission_avail else 0x00

    # send message
    can_msg = can.Message(
        arbitration_id = 401,  # decimal of 0x191
        data = can_data,
        is_extended_id = False 
    )
    try:
        bus.send(can_msg)
        print(f"NSCmd1 sent: {can_data}")
    except can.CanError as e:
        print(f"Failed to send NSCmd1: {e}")
