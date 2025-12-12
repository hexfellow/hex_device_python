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


class Imu(OptionalDeviceBase):
    """
    Imu class - Optional device for processing imu_data

    Inherits from OptionalDeviceBase, mainly implements reading of Imu
    This class processes the optional imu_data field from APIUp messages.

    Supported imu types:
    - ImuY200: ImuY200 imu type
    """

    SUPPORTED_DEVICE_TYPE = [
        public_api_types_pb2.SecondaryDeviceType.SdtImuY200,
    ]

    DEVICE_ID_TO_DEVICE_TYPE = {
        1: public_api_types_pb2.SecondaryDeviceType.SdtImuY200,
    }

    def __init__(
        self,
        device_id,
        device_type,
        send_message_callback,
        name: str = "Imu",
        control_hz: int = 250,
        read_only: bool = True,
    ):
        """
        Initialize Imu device
        
        Args:
            device_id: Device ID (SecondaryDeviceType enum)
            device_type: Device type (e.g., 'SdtUnknown', 'SdtImuY200')
            send_message_callback: Callback function for sending messages, used to send downstream messages
            name: Device name
            control_hz: Control frequency
            read_only: Whether this device is read-only (read only will not create periodic task)
        """
        OptionalDeviceBase.__init__(self, read_only, name, device_id,
                                    device_type, send_message_callback)

        self.name = name or "Imu"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._device_id = device_id
        self._device_type = device_type

        # hand status
        self.acceleration = None
        self.angular_velocity = None
        self.quaternion = None

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
        Update hands device with optional message data
        
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
            # Update imu data
            self.acceleration = device_status.imu_data.acceleration
            self.angular_velocity = device_status.imu_data.angular_velocity
            self.quaternion = device_status.imu_data.quaternion

            return True
        except Exception as e:
            log_err(f"Hands data update failed: {e}")
            return False

    async def _periodic(self):
        """
        Main control loop for imu device
        This method implements the original periodic control logic
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("Imu init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()


    # Imu specific methods
    def get_imu_data(self) -> ImuData:
        """Get imu data"""
        return {
            'acceleration': deepcopy(self.acceleration),
            'angular_velocity': deepcopy(self.angular_velocity),
            'quaternion': deepcopy(self.quaternion)
        }

    def get_imu_summary(self) -> dict:
        """
        Get imu device summary
        
        Returns:
            dict: Imu device summary
        """
        imu_data = self.get_imu_data()
        return {
            'imu_type': self._device_type,
            'imu_data': imu_data
        }
