#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from innotrac_msgs.msg import LifterStatus, NSCmd1, NSCmd2
import can
from nsstatus3_handler import NSStatus3Handler
from nscmd1_handler import send_nscmd1
from nscmd2_handler import send_nscmd2
from scmd1_handler import send_scmd1

class AutonomousControlUnit(Node):
    def __init__(self):
        super().__init__('acu_node')

        # Subscriptions
        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.subscription_lifter_status = self.create_subscription(LifterStatus, '/power_lifter', self.lifter_status_callback, 10)
        
        # CAN interface setup
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # Handlers
        self.nsstatus3_handler = NSStatus3Handler()

        # State variables
        self.current_cmd_vel = Twist()
        self.lifter_status = LifterStatus()
        self.autonomous_mode_activated = False  # New flag to track activation of autonomous mode
        self.target_lifter_height = None
        self.lifter_active_request = False
        self.pto_activation = None
        self.height_check_active = False

        # Lifter calibration ranges
        self.lifter_limits = {
            'front': {'min': 125, 'max': 542},
            'middle': {'min': 120, 'max': 570},
            'rear': {'min': 250, 'max': 699},
        }

        # Timer for sending messages at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg
        self.autonomous_mode_activated = True  # Activate autonomous mode when a cmd_vel message is received

    def lifter_status_callback(self, msg):
        lifter_name = msg.lifter_name.lower()
        target_height = self.clamp_lifter_height(lifter_name, msg.height)
        self.lifter_status = msg
        self.target_lifter_height = target_height
        self.autonomous_mode_activated = True  # Activate autonomous mode when a power_lifter message is received
        self.height_check_active = True  # Start height checking

        # Determine PTO activation
        if lifter_name in ['middle', 'rear']:
            self.pto_activation = 'rear'
        elif lifter_name == 'front':
            self.pto_activation = 'front'
        else:
            self.pto_activation = None

    def clamp_lifter_height(self, lifter_name, height):
        """
        Clamps the lifter height to within the calibrated range.
        """
        limits = self.lifter_limits[lifter_name]
        return max(limits['min'], min(limits['max'], height))

    def timer_callback(self):
        # Send SCmd1[7] to maintain auto mode if activated
        if self.autonomous_mode_activated:
            send_scmd1(self.bus, 7)
        else:
            send_scmd1(self.bus, 0)

        # Handle NSCmd1 (movement) if cmd_vel was received
        if self.autonomous_mode_activated:
            self.send_nscmd1()

        # Handle NSCmd2 (lifter control) if power_lifter was received
        if self.height_check_active and self.target_lifter_height is not None:
            self.check_lifter_height()
            self.send_nscmd2()

    def send_nscmd1(self):
        nscmd1_msg = NSCmd1()
        nscmd1_msg.steer_ang_magn = int(self.current_cmd_vel.angular.z * 100)  # Convert to centidegrees
        nscmd1_msg.drv_spd_magn = int(self.current_cmd_vel.linear.x * 100)  # Convert to cm/s
        nscmd1_msg.eng_spd = 0x03E8  # Example fixed engine speed (1000 RPM)
        nscmd1_msg.steer_dir_le = self.current_cmd_vel.angular.z > 0
        nscmd1_msg.steer_dir_ri = self.current_cmd_vel.angular.z < 0
        nscmd1_msg.drv_dir_fwd = self.current_cmd_vel.linear.x > 0
        nscmd1_msg.drv_dir_bwd = self.current_cmd_vel.linear.x < 0

        # Handle PTO based on the lifter status
        nscmd1_msg.front_pto = self.pto_activation == 'front'
        nscmd1_msg.rear_pto = self.pto_activation == 'rear'

        send_nscmd1(self.bus, nscmd1_msg)

    def send_nscmd2(self):
        nscmd2_msg = NSCmd2()
        lifter_name = self.lifter_status.lifter_name.lower()

        if lifter_name == 'front':
            nscmd2_msg.front_pow_lift_hgt = self.target_lifter_height
            nscmd2_msg.front_pow_lift_flt = self.lifter_active_request
        elif lifter_name == 'middle':
            nscmd2_msg.mid_pow_lift_hgt = self.target_lifter_height
            nscmd2_msg.mid_pow_lift_flt = self.lifter_active_request
        elif lifter_name == 'rear':
            nscmd2_msg.rear_pow_lift_hgt = self.target_lifter_height
            nscmd2_msg.rear_pow_lift_flt = self.lifter_active_request

        send_nscmd2(self.bus, nscmd2_msg)

    def check_lifter_height(self):
        lifter_name = self.lifter_status.lifter_name.lower()
        current_height = self.nsstatus3_handler.get_lifter_heights()[lifter_name]
        
        # Log the current and target height for debugging
        self.get_logger().info(f"Checking height for {lifter_name}: current={current_height}, target={self.target_lifter_height}")
        
        # Check if the current height is within the 10mm tolerance range
        if abs(current_height - self.target_lifter_height) <= 10:
            self.lifter_active_request = True
            self.height_check_active = False  # Stop checking height once within range
            self.get_logger().info(f"{lifter_name.capitalize()} lifter reached target height: {current_height}mm")
        else:
            self.get_logger().info(f"{lifter_name.capitalize()} lifter has not yet reached target height: {current_height}mm")
            self.lifter_active_request = False


def main(args=None):
    rclpy.init(args=args)
    acu_node = AutonomousControlUnit()
    rclpy.spin(acu_node)
    acu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
