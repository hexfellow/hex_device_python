#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any
import threading
import time
from collections import deque


class DeviceBase(ABC):
    """
    Device base class
    Defines common interfaces and basic functionality for all devices
    """

    def __init__(self, name: str = "", send_message_callback=None):
        """
        Initialize device base class
        Args:
            name: Device name
        """
        self.name = name or "Device"

        self._send_message_callback = send_message_callback

        self._last_update_time = deque(maxlen=10)

        self._data_lock = threading.Lock()  # Use for motor data read and write
        self._status_lock = threading.Lock() # Use for status read and write

        self._has_new_data = threading.Event()  # Use Event to avoid lock contention

        # Api_control_initialized
        self._send_init: Optional[bool] = None

    def _set_send_message_callback(self, callback):
        """
        Set callback function for sending messages
        
        Args:
            callback: Asynchronous callback function that accepts one parameter (message object)
        """
        self._send_message_callback = callback

    async def _send_message(self, msg):
        """
        Generic method for sending messages
        
        Args:
            msg: Message object to send
        """
        if msg is not None:
            if self._send_message_callback:
                await self._send_message_callback(msg)
            else:
                raise AttributeError(
                    "send_message: send_message_callback is not set")

    def start(self):
        """
        Start to control chassis
        
        """
        with self._status_lock:
            self._send_init = True
    
    def stop(self):
        """
        Stop to control chassis
        
        """
        with self._status_lock:
            self._send_init = False

    def _clear_send_init(self):
        """
        Clear send init
        """
        with self._status_lock:
            self._send_init = None

    def set_has_new_data(self):
        """Set new data flag"""
        self._has_new_data.set()

    def has_new_data(self) -> bool:
        """Check if there is new data"""
        return self._has_new_data.is_set()

    def clear_new_data_flag(self):
        """
        Clear new data flag
        
        Thread operations can incur significant performance overhead. 
        If you do not rely on has_new_data() to obtain the information update status, it is recommended not to call this method.
        """
        self._has_new_data.clear()

    def get_device_summary(self) -> Dict[str, Any]:
        """Get device status summary"""
        return {
            'name': self.name,
            'has_new_data': self.has_new_data(),
            'last_update_time': self._last_update_time,
        }

    # Abstract methods - subclasses must implement
    @abstractmethod
    async def _init(self) -> bool:
        """
        Initialize device
        
        Returns:
            bool: Whether initialization was successful
        """
        pass

    @abstractmethod
    def _update(self, api_up_data) -> bool:
        """
        Update device data
        
        Args:
            api_up_data: Upstream data received from API (APIUp)
            
        Returns:
            bool: Whether update was successful
        """
        pass

    @abstractmethod
    async def _periodic(self):
        """
        Periodic execution function
        
        Subclasses must implement this method to execute periodic tasks for the device,
        such as status checking, data updates, control calculations, etc.
        
        Returns:
            bool: Whether execution was successful
        """
        pass

    @abstractmethod
    def _set_robot_type(self, robot_type):
        """
        Set robot type
        
        Subclasses must implement this method to set the robot types supported by the device.
        Each subclass should define its own SUPPORTED_ROBOT_TYPES list.
        
        Args:
            robot_type: Robot type
        """
        pass

    @classmethod
    @abstractmethod
    def _supports_robot_type(cls, robot_type):
        """
        Check if the specified robot type is supported
        
        Subclasses must implement this method to check if the specified robot type is supported.
        Each subclass should define its own SUPPORTED_ROBOT_TYPES list.
        
        Args:
            robot_type: Robot type
            
        Returns:
            bool: Whether it is supported
        """
        pass

    def _update_timestamp(self):
        """Update timestamp (adds to queue)"""
        with self._data_lock:
            # Ensure _last_update_time is a deque (may be overridden by child class)
            if not isinstance(self._last_update_time, deque):
                self._last_update_time = deque(maxlen=10)
            self._last_update_time.append(time.time_ns())
        self._has_new_data.set()

    def __str__(self) -> str:
        """String representation"""
        return f"{self.name}, {self.has_new_data()}, {self._last_update_time}"

    def __repr__(self) -> str:
        """Detailed string representation"""
        return f"DeviceBase(name='{self.name}', has_new_data={self.has_new_data()}, last_update_time={self._last_update_time})"
