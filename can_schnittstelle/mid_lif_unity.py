#!/usr/bin/env python3
import rclpy 
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from innotrac_msgs.msg import LifterStatus
import can

class MiddleLifterUnit(Node):
    def __init__(self):
        super().__init__('mid_lift_node')

        # /power_lifter topic subscription
        self.subscription_lifter_status = self.create_subscription(LifterStatus, '/power_lifter', self.lifter_status_callback, 10)

        # CAN Interface Setup
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Initializing lifter heights from NSStatus3
        self.current_lifter_height = {'middle': 0}

        # Timer to check the status
        #self.status_check_timer = self.create_timer(0.01, self.check_nsstatus)

        # Request status for lifters
        self.front_req = 0
        self.mid_req = 0
        self.rear_req = 0

        # 100Hz Frequency 
        self.timer = self.create_timer(0.01, self.send_can_messages)

    # def check_nsstatus(self):
    #     # Read NSStatus from CAN Bus and update relevant parameters
    #     msg = self.bus.recv(timeout=0.1)

    #     if msg and msg.arbitration_id == 419:  # 0x1A3 in hexadecimal is 419 in decimal
    #         # FrontPowLiftHgt - Bits 0-15
    #         front_pow_lift_hgt = (msg.data[1] << 8) | msg.data[0]

    #         # MidPowLiftHgt - Bits 16-31
    #         mid_pow_lift_hgt = (msg.data[3] << 8) | msg.data[2]

    #         # RearPowLiftHgt - Bits 32-47
    #         rear_pow_lift_hgt = (msg.data[5] << 8) | msg.data[4]

    #         # EngSpd - Bits 48-63
    #         eng_spd = (msg.data[7] << 8) | msg.data[6]

    #         # Store or process the values as needed
    #         self.current_lifter_height['front'] = front_pow_lift_hgt
    #         self.current_lifter_height['middle'] = mid_pow_lift_hgt
    #         self.current_lifter_height['rear'] = rear_pow_lift_hgt

    #         # Log the extracted values
    #         self.get_logger().info(f"NSStatus3 - FrontPowLiftHgt: {front_pow_lift_hgt}, MidPowLiftHgt: {mid_pow_lift_hgt}, RearPowLiftHgt: {rear_pow_lift_hgt}, EngSpd: {eng_spd}")
    #     else:
    #         self.get_logger().info(f"Ignored CAN message with ID: {msg.arbitration_id}, Data: {msg.data}")

    def lifter_status_callback(self, msg):
        self.get_logger().info(f"Received message: {msg}")

        lifter_name = msg.lifter_name.lower()
        height = int(msg.height)
        lifter_active = 1 if msg.lifter_active else 0

        if lifter_name == "middle":
            self.current_lifter_height['middle'] = height
            self.mid_req = lifter_active
            self.get_logger().info(f"Updated Lifter Height: {height}, Lifter Active: {lifter_active}")

    def send_can_messages(self):
        #self.send_trigger()
        self.send_nscmd2()

    # def send_trigger(self):
    #     # Trigger message for all messages
    #     trigger_msg = can.Message(
    #         arbitration_id=257,  # 0x101 in hexadecimal
    #         data=[7], 
    #         is_extended_id=False
    #     )

    #     self.bus.send(trigger_msg)
    #     self.get_logger().info(f"Trigger Message sent: {trigger_msg.data}")

    def send_nscmd2(self):
        can_data = [0] * 8

        # # FrontPowLiftHgtReq (Blue) - Bits 0-15
        # can_data[0] = self.front_pow_lift_hgt_req & 0xFF       # Low byte
        # can_data[1] = (self.front_pow_lift_hgt_req >> 8) & 0xFF  # High byte

        # MidPowLiftHgtReq (Orange) - Bits 16-31
        can_data[2] = self.current_lifter_height['middle'] & 0xFF       # Low byte
        can_data[3] = (self.current_lifter_height['middle'] >> 8) & 0xFF  # High byte

        # # RearPowLiftHgtReq (Green) - Bits 32-47
        # can_data[4] = self.rear_pow_lift_hgt_req & 0xFF       # Low byte
        # can_data[5] = (self.rear_pow_lift_hgt_req >> 8) & 0xFF  # High byte

        # Control Bits (Bits 48-55)
        can_data[6] = (self.rear_req << 3) | (self.mid_req << 2) | (self.front_req << 1)

        # Bit 56-63 are reserved/unused and can be set to 0
        can_data[7] = 0x00

        # Convert the data to hexadecimal for logging
        hex_data = [f'0x{byte:02X}' for byte in can_data]
        self.get_logger().info(f"NSCmd2 sent as hex: {hex_data}")

        can_msg = can.Message(
            arbitration_id=402,  # 0x192 in hexadecimal
            data=can_data,
            is_extended_id=False
        )
        self.bus.send(can_msg)

def main(args=None):
    rclpy.init(args=args)
    mid_lift_node = MiddleLifterUnit()
    rclpy.spin(mid_lift_node)
    mid_lift_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
