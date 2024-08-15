#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from innotrac_msgs.msg import LifterStatus  
import can

class AutonomousControlUnit(Node):
    def __init__(self):
        super().__init__('acu_node')

        # Subscriptions
        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, QoSProfile(depth=10))
        self.subscription_lifter_status = self.create_subscription(LifterStatus, '/power_lifter', self.lifter_status_callback, 10)
        
        # CAN interface setup
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # Placeholder for current lifter heights from NSStatus3
        self.current_lifter_heights = {
            'front': 0,
            'middle': 0,
            'rear': 0
        }
        
        # Lifter activation states
        self.front_req = 0
        self.middle_req = 0
        self.rear_req = 0
        
        # Timer to send messages at 100Hz
        self.timer = self.create_timer(0.01, self.send_can_messages)  # 100Hz

    def check_nsstatus3(self):
        # This function reads NSStatus3 from the CAN bus and updates the current_lifter_heights
        msg = self.bus.recv(timeout=0.1)  # Non-blocking call with timeout

        if msg and msg.arbitration_id == 419:  # 0x1A3 in decimal
            self.current_lifter_heights['front'] = (msg.data[1] << 8) | msg.data[0]
            self.current_lifter_heights['middle'] = (msg.data[3] << 8) | msg.data[2]
            self.current_lifter_heights['rear'] = (msg.data[5] << 8) | msg.data[4]
            self.get_logger().info(f"Current lifter heights - Front: {self.current_lifter_heights['front']}, Middle: {self.current_lifter_heights['middle']}, Rear: {self.current_lifter_heights['rear']}")

    def cmd_vel_callback(self, msg):
        # Store the message to be sent at the next timer callback
        self.current_cmd_vel = msg
        self.message_received = True

    def lifter_status_callback(self, msg):
        # Process the /power_lifter message and send NSCmd2 based on lifter_name
        lifter_name = msg.lifter_name.lower()
        height = int(msg.height)
        lifter_active = 1 if msg.lifter_active else 0

        # Trigger autonomous mode
        self.send_trigger(7)

        # Check and update current lifter heights
        self.check_nsstatus3()

        # Send initial lifter height
        if lifter_name == "front":
            self.send_nscmd2(height, 0, 0)
            self.front_req = lifter_active
        elif lifter_name == "middle":
            self.send_nscmd2(self.current_lifter_heights['front'], height, self.current_lifter_heights['rear'])
            self.middle_req = lifter_active
        elif lifter_name == "rear":
            self.send_nscmd2(self.current_lifter_heights['front'], self.current_lifter_heights['middle'], height)
            self.rear_req = lifter_active

        # Send the goal height and activation
        self.send_nscmd2()
        
        # Activate rear PTO if necessary
        if lifter_name in ["middle", "rear"]:
            self.send_nscmd1_with_rear_pto()

        # Revert to SCmd1(0)
        self.send_trigger(0)

    def send_can_messages(self):
        # Always send SCmd1(0) unless triggered by a received message
        if not self.message_received:
            self.send_trigger(0)

        # Send command messages when new data is received
        if self.message_received:
            self.send_trigger(7)
            self.send_nscmd1(self.current_cmd_vel)
            self.message_received = False
            self.send_trigger(0)

    def send_nscmd1(self, msg=None):
        # Send NSCmd1 with the engine speed always set to maximum
        steer_angle = 0
        velocity = 0

        if msg:
            steer_angle = int(msg.angular.z * 100)  # Convert to centidegrees
            velocity = int(msg.linear.x * 100)  # Convert to required format

        engine_speed = 65535  # Always max engine speed

        can_data = [0] * 8

        # SteerAngleMagnReq (Bits 0-15)
        can_data[0] = steer_angle & 0xFF  # LSB
        can_data[1] = (steer_angle >> 8) & 0xFF  # MSB

        # DrvSpdMagnReq (Bits 16-31)
        can_data[2] = velocity & 0xFF  # LSB
        can_data[3] = (velocity >> 8) & 0xFF  # MSB

        # EngSpdReq (Bits 32-47)
        can_data[4] = engine_speed & 0xFF  # LSB
        can_data[5] = (engine_speed >> 8) & 0xFF  # MSB

        # PTO Control (Bits 48-55)
        can_data[6] = (self.front_req << 2) | (self.middle_req << 1) | self.rear_req

        # Send the CAN message
        can_msg = can.Message(
            arbitration_id=401,  # 0x191 in decimal
            data=can_data,
            is_extended_id=False
        )
        self.bus.send(can_msg)
        self.get_logger().info(f"NSCmd1 sent: {can_data}")

    def send_nscmd1_with_rear_pto(self):
        # Send NSCmd1 specifically to activate rear PTO
        can_data = [0] * 8
        engine_speed = 65535  # Always max engine speed

        # EngSpdReq (Bits 32-47)
        can_data[4] = engine_speed & 0xFF  # LSB
        can_data[5] = (engine_speed >> 8) & 0xFF  # MSB

        # Rear PTO activation
        can_data[6] = (1 << 1)  # RearPtoReq

        can_msg = can.Message(
            arbitration_id=401,  # 0x191 in decimal
            data=can_data,
            is_extended_id=False
        )
        self.bus.send(can_msg)
        self.get_logger().info(f"NSCmd1 (Rear PTO) sent: {can_data}")

    def send_trigger(self, value=0):
        # Send SCmd1 with the specified value
        trigger_msg = can.Message(
            arbitration_id=257,  # 0x101 in decimal
            data=[value],  # Data for triggering
            is_extended_id=False
        )
        self.bus.send(trigger_msg)
        self.get_logger().info(f"SCmd1 sent: {value}")

    def send_nscmd2(self, front_height=None, middle_height=None, rear_height=None):
        # Process and send NSCmd2 based on lifter status
        can_data = [0] * 8

        if front_height is not None:
            self.current_lifter_heights['front'] = front_height
        if middle_height is not None:
            self.current_lifter_heights['middle'] = middle_height
        if rear_height is not None:
            self.current_lifter_heights['rear'] = rear_height

        can_data[0] = (self.current_lifter_heights['front'] >> 8) & 0xFF
        can_data[1] = self.current_lifter_heights['front'] & 0xFF
        can_data[2] = (self.current_lifter_heights['middle'] >> 8) & 0xFF
        can_data[3] = self.current_lifter_heights['middle'] & 0xFF
        can_data[4] = (self.current_lifter_heights['rear'] >> 8) & 0xFF
        can_data[5] = self.current_lifter_heights['rear'] & 0xFF

        # Fault requests based on the active lifter
        can_data[6] = (self.front_req << 2) | (self.middle_req << 1) | self.rear_req

        can_msg = can.Message(arbitration_id=402, data=can_data, is_extended_id=False)
        self.bus.send(can_msg)
        self.get_logger().info(f"NSCmd2 sent: {can_data}")

def main(args=None):
    rclpy.init(args=args)
    acu_node = AutonomousControlUnit()
    rclpy.spin(acu_node)
    acu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

