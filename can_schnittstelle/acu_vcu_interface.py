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
        self.sending_cmd7 = False  # Flag to track if SCmd1 should send 7 instead of 0
        self.nscmd1_active = False  # Flag to track if NSCmd1 is active
        self.nscmd2_active = False  # Flag to track if NSCmd2 is active
        
        # Initialize current lifter heights dictionary
        self.current_lifter_heights = {
            'front': 0,
            'middle': 0,
            'rear': 0
        }

        # Initialize lifter request states
        self.front_req = 0
        self.middle_req = 0
        self.rear_req = 0

        # Timer to ensure messages are sent at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)  # 0.01 seconds = 100Hz

    def cmd_vel_callback(self, msg):
        # Store the velocity command data
        self.current_cmd_vel = msg
        self.sending_cmd7 = True  # Switch SCmd1 to send 7
        self.nscmd1_active = True  # Enable NSCmd1

    def lifter_status_callback(self, msg):
        lifter_name = msg.lifter_name.lower()
        self.target_lifter_height = int(msg.height)  # Store the dynamic target height
        self.lifter_active_request = msg.lifter_active
        self.target_lifter = lifter_name
        self.sending_cmd7 = True  # Switch SCmd1 to send 7
        self.nscmd2_active = True  # Enable NSCmd2

    def timer_callback(self):
        # Send SCmd1 at 100Hz
        if self.sending_cmd7:
            self.send_trigger(7)
        else:
            self.send_trigger(0)
        
        # Send NSCmd2 at 100Hz if active
        if self.nscmd2_active:
            self.send_nscmd2()
            self.nscmd1_active = True  # After NSCmd2, enable NSCmd1

        # Send NSCmd1 at 100Hz if active
        if self.nscmd1_active:
            self.send_merged_nscmd1()

    def send_trigger(self, value=0):
        # Send SCmd1 with the specified value (0 or 7)
        trigger_msg = can.Message(
            arbitration_id=257,  # 0x101 in decimal
            data=[value],  # Data for triggering
            is_extended_id=False
        )
        self.bus.send(trigger_msg)
        self.get_logger().info(f"SCmd1 sent: {value}")

    def send_merged_nscmd1(self):
        # Create the merged NSCmd1 message
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
