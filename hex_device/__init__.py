#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

"""
HexDevice Python Library

A Python library for controlling HexDevice robots and devices.
"""

# 核心类
from .device_base import DeviceBase
from .motor_base import MotorBase, MotorError, MotorCommand, CommandType
from .chassis_maver import ChassisMaver

# Import error types
from .error_type import WsError, ProtocolError

# Import web utilities
from .hex_device_api import HexDeviceApi

# Define what gets imported with "from hex_device import *"
__all__ = [
    # 核心类
    'DeviceBase',
    'MotorBase', 
    'MotorError',
    'MotorCommand',
    'CommandType',
    'VehicleDevice',
    'ChassisMaver',
    
    # 工具类
    'HexDeviceApi',
    
    # 版本信息
    '__version__',
    '__author__',
    '__email__'
]

# Version information
__version__ = "1.0.0"
__author__ = "Jecjune"
__email__ = "zejun.chen@hexfellow.com"
