#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import asyncio
import time
import numpy as np
from typing import Optional, Tuple, List, Dict, Any, Union
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base_optional import OptionalDeviceBase
from .motor_base import MitMotorCommand, MotorBase, MotorError, MotorCommand, CommandType, Timestamp
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from copy import deepcopy
from .generated.public_api_types_pb2 import (ImuData)
import threading


class Gamepad(OptionalDeviceBase):
    """
    Gamepad class - Optional device for processing gamepad_read

    Inherits from OptionalDeviceBase, mainly implements reading of Gamepad
    This class processes the optional gamepad_read field from APIUp messages.

    Supported gamepad types:
    - Gamepad: Gamepad gamepad type
    """

    SUPPORTED_DEVICE_TYPE = [
        public_api_types_pb2.SecondaryDeviceType.SdtGamepad,
    ]

    DEVICE_ID_TO_DEVICE_TYPE = {
        1: public_api_types_pb2.SecondaryDeviceType.SdtGamepad,
    }

    def __init__(
        self,
        device_id,
        device_type,
        send_message_callback,
        name: str = "Gamepad",
        control_hz: int = 250,
        read_only: bool = True,
    ):
        """
        Initialize Gamepad device
        
        Args:
            device_id: Device ID (SecondaryDeviceType enum)
            device_type: Device type (e.g., 'SdtUnknown', 'SdtGamepad')
            send_message_callback: Callback function for sending messages, used to send downstream messages
            name: Device name
            control_hz: Control frequency
            read_only: Whether this device is read-only (read only will not create periodic task)
        """
        OptionalDeviceBase.__init__(self, read_only, name, device_id,
                                    device_type, send_message_callback)

        self.name = name or "Gamepad"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._device_id = device_id
        self._device_type = device_type

        # gamepad read
        self.gamepad_read = None

    @classmethod
    def _supports_device_id(cls, device_id):
        """
        Check if the specified device_id is supported
        
        Args:
            device_id: Device ID
            
        Returns:
            bool: Whether it is supported
        """
        return device_id in cls.SUPPORTED_DEVICE_TYPE

    async def _init(self) -> bool:
        """
        Initialize robotic arm
        
        Returns:
            bool: Whether initialization was successful
        """
        return True

    def _update_optional_data(
            self, device_type,
            device_status: public_api_types_pb2.SecondaryDeviceStatus,
            timestamp: Timestamp) -> bool:
        """
        Update gamepad device with optional message data
        
        Args:
            device_type: Should be equal to self._device_type
            device_status: The SecondaryDeviceStatus from APIUp
            timestamp: Timestamp
        Returns:
            bool: Whether update was successful
        """
        if device_type != self._device_type:
            log_warn(
                f"Warning: Imu device type mismatch, expected {self._device_type}, actual {device_type}"
            )
            return False

        try:
            # Update gamepad read data
            self.gamepad_read = device_status.gamepad_read

            return True
        except Exception as e:
            log_err(f"Gamepad data update failed: {e}")
            return False

    async def _periodic(self):
        """
        Main control loop for gamepad device
        This method implements the original periodic control logic
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("Gamepad init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()


    # Gamepad specific methods
    def get_gamepad_read(self) -> public_api_types_pb2.GamepadRead:
        """Get gamepad read data"""
        return deepcopy(self.gamepad_read)

    def get_gamepad_summary(self) -> dict:
        """
        Get gamepad device summary including gamepad read data
        
        Returns:
            dict: Gamepad device summary
        """
        summary = self.get_device_summary()
        gamepad_read = self.get_gamepad_read()

        summary.update({
            'gamepad_type':
            self._device_type,
            'gamepad_read': gamepad_read
        })
        return summary
