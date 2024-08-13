#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import can

class CanAutoModeTrigger(Node):

    def __init__(self):
        super().__init__('can_trigger')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', 
                            self.listener_callback,QoSProfile(depth=10))
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.message_received = False
        #self.get_logger().info('Topic Checker Node has started')

    def listener_callback(self, msg):
        self.message_received = True
        #self.get_logger().info(f'Heard message')

    def timer_callback(self):
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        if self.message_received:
            trigger_on = can.Message(
                arbitration_id=257,
                data=[7],
                is_extended_id=False
            )
            self.message_received = False
            bus.send(trigger_on)            
        else:
            trigger_on = can.Message(
                arbitration_id=257,
                data=[0],
                is_extended_id=False
            )
            bus.send(trigger_on)
        bus.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = CanAutoModeTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()