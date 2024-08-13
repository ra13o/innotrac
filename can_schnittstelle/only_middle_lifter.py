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

        #/power_lifter topic subscription
        self.subscription_lifter_status = self.create_subscription(LifterStatus, '/power_lifter', self.lifter_status_callback, 10)

        # CAN Interface Setup
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Initialising lifter heights from NSStatus3
        self.current_lifter_height = {'middle': 0}

        self.status_check_timer = self.create_timer(0.01, self.check_nsstatus)


        # Request status for lifters
        self.front_req = 0
        self.mid_req = 0
        self.rear_req = 0

        # 100Hz Frequency 
        self.timer = self.create_timer(0.01, self.send_can_messages)

    def check_nsstatus(self):
        # Read NSStatus from CAN Bus and update current_lifter_height
        msg = self.bus.recv(timeout=0.01)

        if msg and msg.arbitration_id == 0x1A3:
            low_byte = msg.data[2]
            high_byte = msg.data[3]
            # Combine the bytes in the correct order
            self.current_lifter_height['middle'] = (high_byte << 8) | low_byte
            self.get_logger().info(f"NSStatus3 - High Byte: 0x{high_byte:02X}, Low Byte: 0x{low_byte:02X}, Updated Height: {self.current_lifter_height['middle']}")
        # else:
        #     self.get_logger().info(f"Ignored CAN message with ID: {msg.arbitration_id}, Data: {msg.data}")


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
        self.send_trigger()
        self.send_nscmd2()

    
    def send_trigger(self):
        # Trigger message for all messages
        trigger_msg = can.Message(
            arbitration_id=257,
            data=[7], 
            is_extended_id=False
        )

        self.bus.send(trigger_msg)
        self.get_logger().info(f"Trigger Message sent: {trigger_msg.data}")


    def send_nscmd2(self):
        # Process and send NSCmd2 messages
        can_data = [0] * 8

        # Populate the CAN data array with appropriate values
        can_data[2] = (self.current_lifter_height['middle'] >> 8) & 0xFF
        can_data[3] = self.current_lifter_height['middle'] & 0xFF
        
        can_data[6] = (self.front_req << 2) | (self.mid_req << 1) | self.rear_req

        # Convert the data array to hexadecimal for logging
        hex_data = [f'0x{byte:02X}' for byte in can_data]
        
        # Log the CAN data as hexadecimal
        self.get_logger().info(f"NSCmd2 sent as hex: {hex_data}")

        can_msg = can.Message(
            arbitration_id=402,
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
