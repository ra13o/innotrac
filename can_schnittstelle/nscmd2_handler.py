#!/usr/bin/env python3

import can
from innotrac_msgs.msg import NSCmd2

def send_nscmd2(bus, cmd: NSCmd2):
    can_data = [0] * 8

    # Pack the height data into CAN message
    can_data[0] = cmd.front_pow_lift_hgt & 0xFF
    can_data[1] = (cmd.front_pow_lift_hgt >> 8) & 0xFF
    can_data[2] = cmd.mid_pow_lift_hgt & 0xFF
    can_data[3] = (cmd.mid_pow_lift_hgt >> 8) & 0xFF
    can_data[4] = cmd.rear_pow_lift_hgt & 0xFF
    can_data[5] = (cmd.rear_pow_lift_hgt >> 8) & 0xFF

    # Logging the height data
    print(f"Front Lifter Height: {cmd.front_pow_lift_hgt}")
    print(f"Middle Lifter Height: {cmd.mid_pow_lift_hgt}")
    print(f"Rear Lifter Height: {cmd.rear_pow_lift_hgt}")
    
    # Set the lifter active status control bits and log their status
    if cmd.front_pow_lift_flt:
        can_data[6] |= (1 << 0)  # Set bit 0 if front lifter is active
        print("Front lifter active status: True")
    else:
        print("Front lifter active status: False")
    
    if cmd.mid_pow_lift_flt:
        can_data[6] |= (1 << 1)  # Set bit 1 if middle lifter is active
        print("Middle lifter active status: True")
    else:
        print("Middle lifter active status: False")
    
    if cmd.rear_pow_lift_flt:
        can_data[6] |= (1 << 2)  # Set bit 2 if rear lifter is active
        print("Rear lifter active status: True")
    else:
        print("Rear lifter active status: False")

    # Send the message and log the outcome
    can_msg = can.Message(
        arbitration_id=402,  # Decimal of 0x192
        data=can_data,
        is_extended_id=False 
    )
    try:
        bus.send(can_msg)
        print(f"NSCmd2 sent: {can_data}")
    except can.CanError as e:
        print(f"Failed to send NSCmd2: {e}")

