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
from .motor_base import MotorBase, MotorError, MotorCommand, CommandType, MitMotorCommand
from .chassis_maver import ChassisMaver

# 机械臂配置系统
from .arm_config import (ArmConfig, ArmConfigManager, DofType, JointParam,
                         JointParams, load_default_arm_config, get_arm_config,
                         add_arm_config, arm_config_manager,
                         set_arm_initial_positions, set_arm_initial_velocities,
                         clear_arm_position_history,
                         clear_arm_velocity_history, clear_arm_motion_history,
                         get_arm_last_positions, get_arm_last_velocities)

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
    'MitMotorCommand',
    'VehicleDevice',
    'ChassisMaver',

    # 机械臂配置系统
    'ArmConfig',
    'ArmConfigManager',
    'DofType',
    'JointParam',
    'JointParams',
    'load_default_arm_config',
    'get_arm_config',
    'add_arm_config',
    'arm_config_manager',
    'set_arm_initial_positions',
    'set_arm_initial_velocities',
    'clear_arm_position_history',
    'clear_arm_velocity_history',
    'clear_arm_motion_history',
    'get_arm_last_positions',
    'get_arm_last_velocities',

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
