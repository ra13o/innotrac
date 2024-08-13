#! /usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can

class CanSubscriber(Node):
    def __init__(self):
        super().__init__('can_subscriber')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.can_callback, 10)

    def can_callback(self, msg):
        self.publish_to_can0(msg)

    def publish_to_can0(self, msg):
        # Initialize the Peak CAN interface
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        can_bits = [0] * 64
        can_data = [0] * 8

        steering_angle = msg.angular.z
        velocity = msg.linear.x
        engine_speed = msg.linear.y

        if steering_angle > 0.0:
            can_bits[48] = 1
        elif steering_angle < 0.0:
            can_bits[49] = 1

        if velocity > 0.0:
            can_bits[50] = 1
        elif velocity < 0.0:
            can_bits[51] = 1

        steering_angle = steering_angle * 100 * (180 / math.pi)
        velocity = velocity * 360

        cast_velocity = abs(int(velocity))
        cast_steering_angle = abs(int(steering_angle))
        cast_engine_speed = abs(int(engine_speed))

        for i in range(16):
            if pow(2, 15 - i) <= cast_steering_angle:
                can_bits[15 - i] = 1
                cast_steering_angle = cast_steering_angle - pow(2, 15 - i)
            if pow(2, 15 - i) <= cast_velocity:
                can_bits[31 - i] = 1
                cast_velocity = cast_velocity - pow(2, 15 - i)
            if pow(2, 15 - i) <= cast_engine_speed:
                can_bits[47 - i] = 1
                cast_engine_speed = cast_engine_speed - pow(2, 15 - i)

        for i in range(8):
            byte = 0
            for j in range(8):
                byte += can_bits[8 * i + j] * pow(2, j)
            can_data[i] = byte

        # Create a CAN message
        can_msg = can.Message(
            arbitration_id=401,
            data=can_data,
            is_extended_id=False
        )

        trigger_on = can.Message(
            arbitration_id=257,
            data=[7],
            is_extended_id=False
        )

        # Send the message
        bus.send(can_msg)
        #bus.send(trigger_on)
        bus.shutdown()

def main(args=None):
    rclpy.init(args=args)
    can_subscriber = CanSubscriber()
    rclpy.spin(can_subscriber)
    can_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()