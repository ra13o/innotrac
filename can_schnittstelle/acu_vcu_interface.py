#! /usr/bin/env python3
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
        
        # Message received flag for /cmd_vel trigger
        self.message_received = False
        
        # Timer to send messages at 100Hz
        self.timer = self.create_timer(0.01, self.send_can_messages)  # 100Hz

    def check_nsstatus3(self):
        # This function would read NSStatus3 from the CAN bus and update the current_lifter_heights
        msg = self.bus.recv()  # Blocking call, replace with timeout for non-blocking

        if msg.arbitration_id == 419:
            self.current_lifter_heights['front'] = (msg.data[0] << 8) | msg.data[1]
            self.current_lifter_heights['middle'] = (msg.data[2] << 8) | msg.data[3]
            self.current_lifter_heights['rear'] = (msg.data[4] << 8) | msg.data[5]
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
        steer_angle = int(msg.angular.z * 100 * (180 / math.pi))  # Convert to centidegrees
        velocity = int(msg.linear.x * 360)  # Convert velocity for transmission
        engine_speed = int(msg.linear.y * 100)  # Assuming engine speed

        can_bits = [0] * 64

        if steer_angle > 0.0:
            can_bits[48] = 1  # Steer left
        elif steer_angle < 0.0:
            can_bits[49] = 1  # Steer right

        if velocity > 0.0:
            can_bits[50] = 1  # Drive forward
        elif velocity < 0.0:
            can_bits[51] = 1  # Drive backward

        # Packing the steering angle, velocity, and engine speed into the CAN data
        cast_steering_angle = abs(int(steer_angle))
        cast_velocity = abs(int(velocity))
        cast_engine_speed = abs(int(engine_speed))

        for i in range(16):
            if pow(2, 15 - i) <= cast_steering_angle:
                can_bits[15 - i] = 1
                cast_steering_angle -= pow(2, 15 - i)
            if pow(2, 15 - i) <= cast_velocity:
                can_bits[31 - i] = 1
                cast_velocity -= pow(2, 15 - i)
            if pow(2, 15 - i) <= cast_engine_speed:
                can_bits[47 - i] = 1
                cast_engine_speed -= pow(2, 15 - i)

        can_data = [0] * 8
        for i in range(8):
            byte = 0
            for j in range(8):
                byte += can_bits[8 * i + j] * pow(2, j)
            can_data[i] = byte

        can_msg = can.Message(
            arbitration_id=401,
            data=can_data,
            is_extended_id=False
        )

        self.bus.send(can_msg)
        self.get_logger().info(f"NSCmd1 sent: {can_data}")

    def send_trigger(self):
        # Send trigger message for /cmd_vel
        trigger_msg = can.Message(
            arbitration_id=257,
            data=[7],  # Data for triggering
            is_extended_id=False
        )
        self.bus.send(trigger_msg)
        self.get_logger().info(f"Trigger message sent: {trigger_msg.data}")

    def send_nscmd2(self):
        # Process and send NSCmd2 based on current lifter heights and active states
        can_data = [0] * 8
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
