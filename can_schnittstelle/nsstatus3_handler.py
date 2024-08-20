#!/usr/bin/env python3

import can

class NSStatus3Handler:
    def __init__(self):
        self.front_lifter_height = 0
        self.middle_lifter_height = 0
        self.rear_lifter_height = 0
        self.engine_speed = 0

    def parse_nsstatus3(self, msg):
        """
        Parses the NSStatus3 CAN message and updates the lifter heights and engine speed.

        Parameters:
        - msg: The CAN message (expected to be an NSStatus3 message).
        """
        if msg.arbitration_id != 419: # 0x1A3 in decimal
            raise ValueError(f"Unexpected CAN message ID: {msg.arbitration_id}")

        # Extract the front lifter height (2 bytes)
        self.front_lifter_height = (msg.data[1] << 8) | msg.data[0]

        # Extract the middle lifter height (2 bytes)
        self.middle_lifter_height = (msg.data[3] << 8) | msg.data[2]

        # Extract the rear lifter height (2 bytes)
        self.rear_lifter_height = (msg.data[5] << 8) | msg.data[4]

        # Extract the engine speed (2 bytes)
        self.engine_speed = (msg.data[7] << 8) | msg.data[6]

        # Print or log the extracted values for debugging
        print(f"Front Lifter Height: {self.front_lifter_height} mm")
        print(f"Middle Lifter Height: {self.middle_lifter_height} mm")
        print(f"Rear Lifter Height: {self.rear_lifter_height} mm")
        print(f"Engine Speed: {self.engine_speed} RPM")

    def get_lifter_heights(self):
        """
        Returns the current heights of the front, middle, and rear lifters.

        Returns:
        - A dictionary with the current lifter heights.
        """
        return {
            "front": self.front_lifter_height,
            "middle": self.middle_lifter_height,
            "rear": self.rear_lifter_height,
        }

    def get_engine_speed(self):
        """
        Returns the current engine speed.

        Returns:
        - The engine speed in RPM.
        """
        return self.engine_speed
