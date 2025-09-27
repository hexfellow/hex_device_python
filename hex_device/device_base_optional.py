#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any, Type
import threading
import time


class OptionalDeviceBase(ABC):
    """
    Optional Device base class
    Defines common interfaces and basic functionality for processing optional fields in APIUp messages.
    These devices are matched by message type rather than robot_type.
    """

    def __init__(self, read_only: bool, name: str = "", send_message_callback=None):
        """
        Initialize optional device base class
        Args:
            name: Device name
            send_message_callback: Callback function for sending messages
        """
        self.name = name or "OptionalDevice"

        self._send_message_callback = send_message_callback

        self._last_update_time = None

        self._data_lock = threading.Lock()

        self._has_new_data = False

        self._read_only = read_only

        # Store supported message types for this device
        self._supported_message_types = self._get_supported_message_types()

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
        if self._send_message_callback:
            await self._send_message_callback(msg)
        else:
            raise AttributeError(
                "send_message: send_message_callback is not set")

    def set_has_new_data(self):
        with self._data_lock:
            self._has_new_data = True

    def has_new_data(self) -> bool:
        """Check if there is new data"""
        with self._data_lock:
            return self._has_new_data

    def clear_new_data_flag(self):
        """Clear new data flag"""
        with self._data_lock:
            self._has_new_data = False

    def get_device_summary(self) -> Dict[str, Any]:
        """Get device status summary"""
        return {
            'name': self.name,
            'has_new_data': self.has_new_data(),
            'last_update_time': self._last_update_time,
        }

    @abstractmethod
    def _get_supported_message_types(self) -> List[str]:
        """
        Get supported message types for this optional device
        
        Subclasses must implement this method to define which optional message types
        they can process. Message types correspond to the optional fields in APIUp.
        
        Examples: ['imu_data', 'gamepad_read', 'hand_status']
        
        Returns:
            List[str]: List of supported message type names
        """
        pass

    def supports_message_type(self, message_type: str) -> bool:
        """
        Check if this device supports the specified message type
        
        Args:
            message_type: Message type name (e.g., 'imu_data', 'gamepad_read')
            
        Returns:
            bool: Whether this message type is supported
        """
        return message_type in self._supported_message_types

    @classmethod
    @abstractmethod
    def get_supported_message_types_static(cls) -> List[str]:
        """
        Static method to get supported message types without instantiation
        
        Returns:
            List[str]: List of supported message type names
        """
        pass

    # Abstract methods - subclasses must implement
    @abstractmethod
    async def _init(self) -> bool:
        """
        Initialize optional device
        
        Returns:
            bool: Whether initialization was successful
        """
        pass

    @abstractmethod
    def _update_optional_data(self, message_type: str, message_data) -> bool:
        """
        Update device with optional message data
        
        Args:
            message_type: Type of the optional message (e.g., 'imu_data', 'gamepad_read')
            message_data: The actual message data from APIUp
            
        Returns:
            bool: Whether update was successful
        """
        pass

    async def _periodic(self):
        """
        Periodic execution function
        
        Default implementation that does nothing but returns success.
        Subclasses can override this method to execute periodic tasks for the device,
        such as status checking, data updates, control calculations, etc.
        Subclasses can also call super()._periodic() to use this default behavior.
        If read_only is False, this method will be called periodically.
        
        Returns:
            bool: Whether execution was successful
        """
        return True

    def _update_timestamp(self):
        """Update timestamp"""
        with self._data_lock:
            self._last_update_time = time.time_ns()
            self._has_new_data = True

    def __str__(self) -> str:
        """String representation"""
        return f"{self.name}, {self.has_new_data()}, {self._last_update_time}"

    def __repr__(self) -> str:
        """Detailed string representation"""
        return f"OptionalDeviceBase(name='{self.name}', has_new_data={self.has_new_data()}, last_update_time={self._last_update_time}, supported_types={self._supported_message_types})"

