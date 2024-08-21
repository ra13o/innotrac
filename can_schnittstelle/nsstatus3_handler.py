#!/usr/bin/env python3

import can
import threading
from typing import Dict, Optional
import time

class NSStatus3Handler:
    def __init__(self, channel: str = 'can0', bustype: str = 'socketcan', bitrate: int = 500000):
        """
        Initializes the NSStatus3Handler and starts listening on the CAN bus for NSStatus3 messages.

        :param channel: The CAN interface channel (e.g., 'can0').
        :param bustype: The type of CAN interface being used (default is 'socketcan').
        :param bitrate: The bitrate of the CAN bus (default is 500000).
        """
        self.front_lifter_height: int = 0
        self.middle_lifter_height: int = 0
        self.rear_lifter_height: int = 0
        self.engine_speed: int = 0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Set up the CAN bus
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)

        # Start the listener thread
        self._listener_thread = threading.Thread(target=self._listen_to_can_bus, daemon=True)
        self._listener_thread.start()

        # Wait for a short period to allow processing of initial messages
        time.sleep(0.1)  # Adjust the sleep time if needed

    def _listen_to_can_bus(self):
        """
        Listens to the CAN bus for messages with arbitration ID 419 (0x1A3) and parses them.
        """
        while not self._stop_event.is_set():
            msg = self.bus.recv(timeout=1)  # Wait for a message
            if msg and msg.arbitration_id == 419:  # 0x1A3
                self.parse_nsstatus3(msg)

    def parse_nsstatus3(self, msg: can.Message):
        """
        Parses the NSStatus3 CAN message and updates the lifter heights and engine speed.

        :param msg: The CAN message (expected to be an NSStatus3 message with ID 419).
        """
        with self._lock:
            self.front_lifter_height = (msg.data[1] << 8) | msg.data[0]
            self.middle_lifter_height = (msg.data[3] << 8) | msg.data[2]
            self.rear_lifter_height = (msg.data[5] << 8) | msg.data[4]
            self.engine_speed = (msg.data[7] << 8) | msg.data[6]

    def get_lifter_heights(self) -> Dict[str, int]:
        """
        Returns the current heights of the front, middle, and rear lifters.

        :return: A dictionary with the current lifter heights.
        """
        with self._lock:
            return {
                "front": self.front_lifter_height,
                "middle": self.middle_lifter_height,
                "rear": self.rear_lifter_height,
            }

    def get_engine_speed(self) -> int:
        """
        Returns the current engine speed.

        :return: The engine speed in RPM.
        """
        with self._lock:
            return self.engine_speed

    def stop(self):
        """
        Stops the CAN bus listener and cleans up resources.
        """
        self._stop_event.set()
        self._listener_thread.join()
        self.bus.shutdown()