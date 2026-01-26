#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import asyncio
from collections import deque
from signal import raise_signal
import time
import numpy as np
from typing import Optional, Tuple, List, Dict, Any, Union
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base_optional import OptionalDeviceBase
from .motor_base import MitMotorCommand, MotorBase, MotorError, MotorCommand, CommandType, Timestamp
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from copy import deepcopy
import threading


class SdtHello(OptionalDeviceBase):
    """
    SdtHello class - Optional device for processing hello_data

    Inherits from OptionalDeviceBase, mainly implements reading of SdtHello
    This class processes the optional hello_data field from APIUp messages.

    Supported hello types:
    - Hello1J1T4B: Hello1J1T4B hello type
    """

    SUPPORTED_DEVICE_TYPE = [
        public_api_types_pb2.SecondaryDeviceType.SdtHello1J1T4BV1,
    ]

    DEVICE_ID_TO_DEVICE_TYPE = {
        5: public_api_types_pb2.SecondaryDeviceType.SdtHello1J1T4BV1,
    }

    def __init__(
        self,
        device_id,
        device_type,
        send_message_callback,
        name: str = "SdtHello",
        control_hz: int = 500,
        read_only: bool = False,
    ):
        """
        Initialize SdtHello device
        
        Args:
            device_id: Device ID (SecondaryDeviceType enum)
            device_type: Device type (e.g., 'SdtUnknown', 'SdtHello1J1T4BV1')
            send_message_callback: Callback function for sending messages, used to send downstream messages
            name: Device name
            control_hz: Control frequency
            read_only: Whether this device is read-only (read only will not create periodic task)
        """
        OptionalDeviceBase.__init__(self, read_only, name, device_id,
                                    device_type, send_message_callback)

        self.name = name or "SdtHello"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._device_id = device_id
        self._device_type = device_type

        if self._device_type == public_api_types_pb2.SecondaryDeviceType.SdtHello1J1T4BV1:
            self._motor_count = 7

        # hello status
        self._status_lock = threading.Lock()
        self._timestamp = None
        self._status_data = deque(maxlen=10)

        # control command
        self._command_lock = threading.Lock()
        self._command = None

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
        # hello no need to init
        return True

    def _update_optional_data(
            self, device_type,
            device_status: public_api_types_pb2.SecondaryDeviceStatus,
            timestamp: Timestamp) -> bool:
        """
        Update hello device with optional message data
        
        Args:
            device_type: Should be equal to self._device_type
            device_status: The SecondaryDeviceStatus from APIUp
            timestamp: Timestamp
        Returns:
            bool: Whether update was successful
        """
        if device_type != self._device_type:
            log_warn(
                f"Warning: SdtHello device type mismatch, expected {self._device_type}, actual {device_type}"
            )
            return False

        try:
            # Update hello data
            with self._status_lock:
                self._timestamp = timestamp
                self._status_data.append((device_status, timestamp))

            return True
        except Exception as e:
            log_err(f"SdtHello data update failed: {e}")
            return False

    async def _periodic(self):
        """
        Main control loop for hello device
        This method implements the original periodic control logic
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("SdtHello init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                # process command
                with self._command_lock:
                    c = self._command
                    self._command = None

                if c is not None:
                    cmd = self._construct_rgb_stripe_command(c[0], c[1], c[2])
                    await self._send_message(cmd)

            except Exception as e:
                log_err(f"SdtHello periodic task exception: {e}")
                await asyncio.sleep(0.5)
                continue


    # SdtHello specific methods
    def has_new_data(self) -> bool:
        """Check if there is new data"""
        return len(self._status_data) > 0

    def get_simple_motor_status(self, pop: bool = True) -> Optional[Dict[str, Any]]:
        """Get simple hello status
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Dictionary with simple hello status or None if queue is empty
        """
        with self._status_lock:
            if self._timestamp is None:
                return None
            else:
                timestamp = self._timestamp
            if pop:
                if len(self._status_data) != 0:
                    status = self._status_data.popleft()
                else:
                    return None
            else:
                status = self._status_data[-1]

        joystick_x = status[0].hello1j1t4b_status.joystick_x
        joystick_y = status[0].hello1j1t4b_status.joystick_y
        trigger = status[0].hello1j1t4b_status.trigger
        btn_z = 1.0 if status[0].hello1j1t4b_status.btn_z else -1.0
        btn_w = 1.0 if status[0].hello1j1t4b_status.btn_w else -1.0
        btn_x = 1.0 if status[0].hello1j1t4b_status.btn_x else -1.0
        btn_y = 1.0 if status[0].hello1j1t4b_status.btn_y else -1.0

        return {
            'pos': [trigger, joystick_x, joystick_y, btn_z, btn_w, btn_x, btn_y],
            'vel': [0.0] * self._motor_count,
            'eff': [0.0] * self._motor_count,
            'ts': timestamp.to_dict()
        }

    def set_rgb_stripe_command(self, r: list[int], g: list[int], b: list[int]):
        """
        Set RGB stripe command

        Args:
            r: List of red values (0-255)
            g: List of green values (0-255)
            b: List of blue values (0-255)
        """
        if len(r) != len(g) != len(b):
            raise ValueError("RGB list must have the same length")
        with self._command_lock:
            self._command = (r, g, b)

    def _construct_rgb_stripe_command(self, r: list[int], g: list[int], b: list[int]) -> public_api_down_pb2.APIDown:
        """Construct RGB stripe command"""
        if len(r) != len(g) != len(b):
            raise ValueError("RGB list must have the same length")
        cmd = public_api_types_pb2.RGBStripeCommand()
        for i in range(len(r)):
            cmd.rgbs.append(r[i] | g[i] << 8 | b[i] << 16)
        
        msg = public_api_down_pb2.APIDown()
        secondary_device_command = public_api_types_pb2.SecondaryDeviceCommand()
        secondary_device_command.device_id = self._device_id
        secondary_device_command.hello1j1t4b_controller_command.rgb_stripe_command.CopyFrom(cmd)
        msg.secondary_device_command.CopyFrom(secondary_device_command)
        return msg

    # Configuration related methods
    def get_joint_limits(self) -> List[float]:
        """Get hello joint limits"""
        return deepcopy([[-1.0, 1.0, 0.0, 0.0, 0.0, 0.0]] * self._motor_count)

    def get_hello_summary(self) -> dict:
        """
        Get hello device summary
        
        Returns:
            dict: SdtHello device summary
        """
        summary = self.get_device_summary()
        summary.update({
            'hello_type': self._device_type,
            'control_hz': self._control_hz,
        })
        return summary

    def __len__(self) -> int:
        """Return motor count"""
        return self._motor_count