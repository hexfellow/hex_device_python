#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any, Type
import logging
import threading
from collections import deque

_fallback_logger = logging.getLogger('hex_device')


class OptionalDeviceBase(ABC):
    """
    Optional Device base class
    Defines common interfaces and basic functionality for processing SecondaryDeviceStatus in APIUp messages.
    These devices are matched by device_id from SecondaryDeviceStatus.
    """

    def __init__(self, read_only: bool, name: str, device_id, device_type,
                 send_message_callback=None, logger=None):
        """
        Initialize optional device base class
        Args:
            read_only: Whether this device is read-only
            name: Device name
            send_message_callback: Callback function for sending messages
            device_id: Device ID from SecondaryDeviceStatus
            logger: Per-instance logger or LoggerAdapter; falls back to the
                    shared 'hex_device' logger when not provided.
        """
        self.name = name or "OptionalDevice"
        self.device_id = device_id
        self.device_type = device_type
        self._logger = logger if logger is not None else _fallback_logger

        self._send_message_callback = send_message_callback

        self._data_lock = threading.Lock()

        self._read_only = read_only


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

    def get_device_summary(self) -> Dict[str, Any]:
        """Get device status summary"""
        return {
            'name': self.name,
            'device_id': self.device_id,
        }


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
    def _update_optional_data(self, device_type, device_status) -> bool:
        """
        Update device with optional message data
        
        Args:
            device_type: The device type of the optional message
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

    def _log_info(self, msg: str) -> None:
        self._logger.info(msg)

    def _log_warn(self, msg: str) -> None:
        self._logger.warning(msg)

    def _log_err(self, msg: str) -> None:
        self._logger.error(msg)

    def _log_debug(self, msg: str) -> None:
        self._logger.debug(msg)

    def __str__(self) -> str:
        """String representation"""
        return f"{self.name}"

    def __repr__(self) -> str:
        """Detailed string representation"""
        return f"OptionalDeviceBase(name='{self.name}', device_id={self.device_id})"

