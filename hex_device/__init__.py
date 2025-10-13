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

import logging

# Configure default logging for the hex_device package
# Users can override this configuration if needed
def _setup_default_logging():
    """Setup default logging configuration for hex_device package"""
    logger = logging.getLogger('hex_device')
    
    # Only add handler if no handlers exist (avoid duplicate handlers)
    if not logger.handlers:
        # Create handler
        handler = logging.StreamHandler()
        
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        
        # Add handler to logger
        logger.addHandler(handler)
        
        # Set default level to WARNING (so INFO and DEBUG are not shown by default)
        # Users can change this by calling logging.getLogger('hex_device').setLevel(logging.INFO)
        logger.setLevel(logging.WARNING)
        
        # Prevent propagation to root logger to avoid duplicate messages
        logger.propagate = False

# Setup default logging
_setup_default_logging()

def set_log_level(level):
    """
    Set the logging level for hex_device package
    
    Args:
        level: Logging level (logging.DEBUG, logging.INFO, logging.WARNING, logging.ERROR)
               or string ('DEBUG', 'INFO', 'WARNING', 'ERROR')
    
    Example:
        import hex_device
        import logging
        
        # Enable INFO level logging
        hex_device.set_log_level(logging.INFO)
        # or
        hex_device.set_log_level('INFO')
    """
    logger = logging.getLogger('hex_device')
    
    if isinstance(level, str):
        level = getattr(logging, level.upper())
    
    logger.setLevel(level)

def get_logger():
    """
    Get the hex_device logger
    
    Returns:
        logging.Logger: The hex_device package logger
        
    Example:
        import hex_device
        logger = hex_device.get_logger()
        logger.info("Custom log message")
    """
    return logging.getLogger('hex_device')

# Core classes
from .device_base import DeviceBase
from .device_base_optional import OptionalDeviceBase
from .device_factory import DeviceFactory
from .motor_base import (
    MotorBase, 
    MotorError, 
    MotorCommand, 
    CommandType, 
    MitMotorCommand
)

# Device implementations
from .chassis import Chassis

# Optional device implementations
from .hands import Hands

# Arm configuration system
from .arm_config import (
    ArmConfig,
    ArmConfigManager, 
    DofType,
    JointParam,
    JointParams,
    load_default_arm_config,
    get_arm_config,
    add_arm_config,
    arm_config_manager,
    set_arm_initial_positions,
    set_arm_initial_velocities,
    clear_arm_position_history,
    clear_arm_velocity_history,
    clear_arm_motion_history,
    get_arm_last_positions,
    get_arm_last_velocities
)

# Error types
from .error_type import WsError, ProtocolError

# API utilities  
from .hex_device_api import HexDeviceApi

# Define what gets imported with "from hex_device import *"
__all__ = [
    # Core classes
    'DeviceBase',
    'OptionalDeviceBase',
    'DeviceFactory',
    'MotorBase',
    'MotorError',
    'MotorCommand',
    'CommandType',
    'MitMotorCommand',
    
    # Device implementations
    'Chassis',
    
    # Optional device implementations
    'Hands',

    # Arm configuration system
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

    # Error types
    'WsError',
    'ProtocolError',

    # API utilities
    'HexDeviceApi',
    
    # Logging functionality
    'set_log_level',
    'get_logger',

    # Version information
    '__version__',
    '__author__',
    '__email__'
]

# Version information
__version__ = "1.0.0"
__author__ = "Jecjune"
__email__ = "zejun.chen@hexfellow.com"
