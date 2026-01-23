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
import sys
import os
from pathlib import Path
from typing import Optional

# Custom filter for separating error logs from other logs
class NonErrorFilter(logging.Filter):
    """Filter that only allows non-error level logs (DEBUG, INFO, WARNING)"""
    def filter(self, record):
        return record.levelno < logging.ERROR

# Configure default logging for the hex_device package
# Users can override this configuration if needed
def _setup_default_logging():
    """Setup default logging configuration for hex_device package"""
    logger = logging.getLogger('hex_device')
    
    # Only add handler if no handlers exist (avoid duplicate handlers)
    if not logger.handlers:
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # Create handler for non-error logs (DEBUG, INFO, WARNING) -> stdout
        stdout_handler = logging.StreamHandler(stream=sys.stdout)
        stdout_handler.setFormatter(formatter)
        stdout_handler.setLevel(logging.DEBUG)  # Capture all levels at handler level
        # Filter: only allow DEBUG, INFO, WARNING through to stdout
        stdout_handler.addFilter(NonErrorFilter())
        
        # Create handler for error logs (ERROR, CRITICAL) -> stderr
        stderr_handler = logging.StreamHandler(stream=sys.stderr)
        stderr_handler.setFormatter(formatter)
        stderr_handler.setLevel(logging.ERROR)  # Only ERROR and above
        
        # Add handlers to logger
        logger.addHandler(stdout_handler)
        logger.addHandler(stderr_handler)
        
        # Set default level to INFO (so DEBUG are not shown by default)
        # Users can change this by calling logging.getLogger('hex_device').setLevel(logging.INFO)
        logger.setLevel(logging.INFO)
        
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

def _get_version_from_pyproject() -> Optional[str]:
    """
    Get version from pyproject.toml file.
    
    Returns:
        Version string or None if not found
    """
    try:
        # Try to get version from installed package metadata (works for installed packages)
        try:
            from importlib.metadata import version
            return version("hex_device")
        except ImportError:
            # Python < 3.8, try pkg_resources
            try:
                import pkg_resources
                return pkg_resources.get_distribution("hex_device").version
            except (ImportError, pkg_resources.DistributionNotFound):
                pass
    except Exception:
        pass
    
    # Fallback: read directly from pyproject.toml (works in development)
    try:
        # Get the directory of this file
        current_file = Path(__file__).resolve()
        # Go up to hex_device_python directory
        project_root = current_file.parent.parent
        pyproject_path = project_root / "pyproject.toml"
        
        if pyproject_path.exists():
            # Try to parse TOML
            try:
                # Python 3.11+ has tomllib built-in
                import tomllib
                with open(pyproject_path, 'rb') as f:
                    data = tomllib.load(f)
                    return data.get('project', {}).get('version')
            except ImportError:
                # Python < 3.11, try tomli
                try:
                    import tomli
                    with open(pyproject_path, 'rb') as f:
                        data = tomli.load(f)
                        return data.get('project', {}).get('version')
                except ImportError:
                    # Fallback: simple regex parsing (not ideal but works)
                    import re
                    with open(pyproject_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                        match = re.search(r'version\s*=\s*["\']([^"\']+)["\']', content)
                        if match:
                            return match.group(1)
    except Exception:
        pass
    
    return None

# Version information (defined early to avoid circular import)
_version = _get_version_from_pyproject()
__version__ = _version if _version else "1.0.0"  # Fallback version
__author__ = "Jecjune"
__email__ = "zejun.chen@hexfellow.com"

# Proto types
from .generated import public_api_types_pb2
from .generated.version import CURRENT_PROTOCOL_MAJOR_VERSION, CURRENT_PROTOCOL_MINOR_VERSION

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
from .arm import Arm
from .chassis import Chassis
from .linear_lift import LinearLift
from .zeta_lift import ZetaLift

# Optional device implementations
from .hands import Hands
from .imu import Imu
from .gamepad import Gamepad
from .sdt_hello import SdtHello

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
from .error_type import WsError, ProtocolError, InvalidWSURLException

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
    'Arm',
    'Chassis',
    'LinearLift',
    'ZetaLift',
    # Optional device implementations
    'Hands',
    'Imu',
    'Gamepad',
    'SdtHello',

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
    'InvalidWSURLException',


    # API utilities
    'HexDeviceApi',
    
    # Logging functionality
    'set_log_level',
    'get_logger',

    # Version information
    '__version__',
    '__author__',
    '__email__',

    # Protocol version information
    'CURRENT_PROTOCOL_MAJOR_VERSION',
    'CURRENT_PROTOCOL_MINOR_VERSION',
]
