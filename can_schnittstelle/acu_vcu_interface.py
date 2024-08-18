#!/usr/bin/env python3
import rclpy
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
        
        # Placeholder for command data
        self.current_cmd_vel = Twist()
        self.rear_pto_active = False
        self.front_pto_active = False

        # Timer to send messages at 100Hz
        self.timer = self.create_timer(0.01, self.send_can_messages)  # 100Hz

    def cmd_vel_callback(self, msg):
        # Store the velocity command data
        self.current_cmd_vel = msg
        self.message_received = True

    def lifter_status_callback(self, msg):
        lifter_name = msg.lifter_name.lower()
        height = int(msg.height)
        lifter_active = 1 if msg.lifter_active else 0

        # Map lifter names to the corresponding indices in the CAN message
        if lifter_name == "front":
            self.current_lifter_heights['front'] = height
            self.front_req = lifter_active
        elif lifter_name == "middle":
            self.current_lifter_heights['middle'] = height
            self.middle_req = lifter_active
        elif lifter_name == "rear":
            self.current_lifter_heights['rear'] = height
            self.rear_req = lifter_active

    def send_can_messages(self):
        # Check and send NSCmd1 (cmd_vel related)
        if self.message_received:
            self.send_trigger()
            self.send_nscmd1(self.current_cmd_vel)
            self.message_received = False

        # Check and send NSCmd2 (lifter status related)
        self.send_trigger()
        self.send_nscmd2()

    def send_nscmd1(self, msg):
        # Process the /cmd_vel message and send NSCmd1
        steer_angle = int(msg.angular.z * 100)  # Convert to centidegrees, multiplied by 100
        velocity = int(msg.linear.x * 100)  # Convert velocity, multiplied by 100
        engine_speed = 65535  # Engine speed is always set to max (0xFFFF)

        can_data = [0] * 8

        # Extract and convert cmd_vel data
        steer_angle = int(self.current_cmd_vel.angular.z * 100)  # Convert to centidegrees
        velocity = int(self.current_cmd_vel.linear.x * 100)  # Convert velocity

        # SteerAngleMagnReq (Bits 0-15)
        can_data[0] = steer_angle & 0xFF  # LSB
        can_data[1] = (steer_angle >> 8) & 0xFF  # MSB

        # DrvSpdMagnReq (Bits 16-31)
        can_data[2] = velocity & 0xFF  # LSB
        can_data[3] = (velocity >> 8) & 0xFF  # MSB

        # EngSpdReq (Bits 32-47)
        can_data[4] = 0xFF  # Always max engine speed (LSB)
        can_data[5] = 0xFF  # Always max engine speed (MSB)

        # Control Bits (Bits 48-55)
        if steer_angle > 0:
            can_data[6] |= (1 << 0)  # SteerDirLeReq (bit 48)
        elif steer_angle < 0:
            can_data[6] |= (1 << 1)  # SteerDirRiReq (bit 49)

        if velocity > 0:
            can_data[6] |= (1 << 2)  # DrvDirFwdReq (bit 50)
        elif velocity < 0:
            can_data[6] |= (1 << 3)  # DrvDirBwdReq (bit 51)

        # PTO Bits (52-53)
        if self.front_pto_active:
            can_data[6] |= (1 << 4)  # FrontPtoReq (bit 52)
        if self.rear_pto_active:
            can_data[6] |= (1 << 5)  # RearPtoReq (bit 53)

        # Prepare and send the CAN message
        can_msg = can.Message(
            arbitration_id=401,  # 0x191 in decimal
            data=can_data,
            is_extended_id=False
        )

        self.bus.send(can_msg)
        self.get_logger().info(f"Merged NSCmd1 sent: {can_data}")

    def send_trigger(self):
        # Send trigger message for /cmd_vel
        trigger_msg = can.Message(
            arbitration_id=257,  # 0x101 in decimal
            data=[7],  # Data for triggering
            is_extended_id=False
        )
        self.bus.send(trigger_msg)
        self.get_logger().info(f"Trigger message sent: {trigger_msg.data}")

    def send_nscmd2(self):
        # Process and send NSCmd2 based on current lifter heights and active states
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

    def monitor_height(self):
        while True:
            self.check_nsstatus3()
            if self.target_lifter == 'middle' and self.current_lifter_heights['middle'] == 800:
                self.middle_req = 1 if self.lifter_active_request else 0
                self.send_nscmd2()  # Update height
                self.monitor_lifter_activation()
                break
            self.get_logger().info(f"Waiting for middle lifter to reach 800... Current: {self.current_lifter_heights['middle']}")

    def monitor_lifter_activation(self):
        while True:
            self.check_nsstatus3()
            if self.middle_req == 1:  # If middle lifter is activated
                break
            self.get_logger().info("Waiting for middle lifter activation...")

    def check_nsstatus3(self):
        # This function reads NSStatus3 from the CAN bus and updates the current_lifter_heights
        msg = self.bus.recv(timeout=0.1)  # Non-blocking call with timeout

        if msg and msg.arbitration_id == 419:  # 0x1A3 in decimal
            self.current_lifter_heights['front'] = (msg.data[1] << 8) | msg.data[0]
            self.current_lifter_heights['middle'] = (msg.data[3] << 8) | msg.data[2]
            self.current_lifter_heights['rear'] = (msg.data[5] << 8) | msg.data[4]
            self.get_logger().info(f"Current lifter heights - Front: {self.current_lifter_heights['front']}, Middle: {self.current_lifter_heights['middle']}, Rear: {self.current_lifter_heights['rear']}")

def main(args=None):
    rclpy.init(args=args)
    acu_node = AutonomousControlUnit()
    rclpy.spin(acu_node)
    acu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
