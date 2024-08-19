#!/usr/bin/env python3
import can
import os
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

# Global variables to store GNSS and orientation data
gnss_data = {"x": None, "y": None}
orientation = {"orientations": [0.0]}

# Node class to handle ROS 2 subscriptions and callbacks
class CanLogger(Node):
    def __init__(self):
        super().__init__('can_logger')

        # Subscribe to GNSS and Odometry topics
        self.create_subscription(NavSatFix, "/fixposition/navsatfix", self.navsatfix_callback, 10)
        self.create_subscription(Odometry, "/fixposition/odometry_enu", self.odometry_callback, 10)

        # Set up CAN interface
        self.can_interface = 'can0'
        self.bus = can.interface.Bus(self.can_interface, bustype='socketcan')

        # Initialize data structure
        self.aggregated_data = {
            "x": None,
            "y": None,
            "orientations": [0.0],
            "sensors": {
                "MachineData": {
                    "tankLevel": 0,
                    "engineLoad": 0,
                    "velocity": 0,
                    "engineSpeed": 0,
                    "frontPTO": 0,
                    "rearMidPTO": 0,
                    "mode": "auto"
                },
                "GNSSData": {
                    "geoFencingStatus": "ok"
                },
                "HydraulicData": {
                    "forwardHydraulicPressureA": 0,
                    "forwardHydraulicPressureB": 0,
                    "reverseHydraulicPressureA": 0,
                    "reverseHydraulicPressureB": 0,
                    "workHydraulicPressure": 0,
                    "pilotStatus": "ok",
                    "oilTemperatureStatus": "ok",
                    "oilLevelStatus": "ok"
                },
                "ImplementsData": {
                    "nameImplementRear": "Implement A",
                    "heightImplementRear": 0,
                    "nameImplementFront": "Implement B",
                    "heightImplementFront": 0,
                    "nameImplementMiddle": "Implement C",
                    "heightImplementMiddle": 0,
                    "speedImplementMiddle": 0
                }
            }
        }

        # Set data logging frequency (100ms)
        self.log_interval = 0.1
        self.last_log_time = time.time()

        # Main loop to process CAN messages
        self.create_timer(0.01, self.process_can_messages)

    def navsatfix_callback(self, data):
        global gnss_data
        gnss_data["x"] = data.longitude
        gnss_data["y"] = data.latitude

    def odometry_callback(self, data):
        global orientation
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        rotation = R.from_quat(quaternion)
        heading = rotation.as_euler('zyx', degrees=True)[0]  # Extract yaw (heading) in degrees
        heading = (heading + 360) % 360  # Normalize to 0-360
        orientation["orientations"][0] = heading

    def parse_can_message(self, message):
        data = {}

        if message.arbitration_id == 0x1a5:  # NSStatus5
            data = {
                "HydraulicData": {
                    "forwardHydraulicPressureA": message.data[0],
                    "forwardHydraulicPressureB": message.data[2],
                    "reverseHydraulicPressureA": message.data[4],
                    "reverseHydraulicPressureB": message.data[6],
                }
            }
        elif message.arbitration_id == 0x11e:  # SError1
            s_error_code = message.data[3]
            data = {"MachineData": {"SErrorCode1": s_error_code}}
        elif message.arbitration_id == 0x111:  # SStatus1
            auto_mode_status = message.data[0] & 0x03
            man_mode_status = (message.data[0] >> 2) & 0x03
            mode = "off"
            if auto_mode_status == 1:
                mode = "auto"
            elif man_mode_status == 1:
                mode = "manual"
            elif auto_mode_status == 2 or man_mode_status == 2:
                mode = "error"
            data = {"MachineData": {"mode": mode}}
        elif message.arbitration_id == 0x1a4:  # NSStatus4
            hydr_oil_lvl_too_low = message.data[7] & 0x01
            hydr_oil_over_temp = (message.data[7] >> 1) & 0x01
            data = {
                "HydraulicData": {
                    "workHydraulicPressure": message.data[3],
                    "oilTemperatureStatus": "ok" if hydr_oil_over_temp == 0 else "error",
                    "oilLevelStatus": "ok" if hydr_oil_lvl_too_low == 0 else "error"
                }
            }

        # Add more conditions for other message IDs as needed

        return data

    def merge_data(self, existing_data, new_data):
        for key, value in new_data.items():
            if isinstance(value, dict):
                self.merge_data(existing_data[key], value)
            elif value is not None:
                existing_data[key] = value

    def process_can_messages(self):
        message = self.bus.recv()

        if message is not None:
            new_data = self.parse_can_message(message)
            self.merge_data(self.aggregated_data["sensors"], new_data)

        # Update GNSS and orientation data
        self.aggregated_data["x"] = gnss_data["x"]
        self.aggregated_data["y"] = gnss_data["y"]
        self.aggregated_data["orientations"] = orientation["orientations"]

        # Update JSON file every 100ms
        if time.time() - self.last_log_time >= self.log_interval:
            self.last_log_time = time.time()
            file_path = '/tmp/position_sensor.json'
            with open(file_path, 'w') as json_file:
                json.dump(self.aggregated_data, json_file, indent=4)


def main(args=None):
    rclpy.init(args=args)
    can_logger = CanLogger()
    rclpy.spin(can_logger)
    can_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
