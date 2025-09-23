#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
from typing import Optional, Tuple, List, Type, Dict, Any
from .device_base import DeviceBase
from .common_utils import log_err

class DeviceFactory:
    """
    Device factory class, responsible for creating and managing device instances based on robot_type
    """

    def __init__(self):
        self._device_classes: List[Type[DeviceBase]] = []

    def register_device_class(self, device_class):
        """
        Register device class
        
        Args:
            device_class: Device class, must support _supports_robot_type class method
        """
        if hasattr(device_class, '_supports_robot_type'):
            self._device_classes.append(device_class)
        else:
            raise ValueError(
                f"Device class {device_class.__name__} must support _supports_robot_type class method")

    def create_device_for_robot_type(
        self,
        robot_type,
        send_message_callback=None,
        api_up=None,
    ):
        """
        Create device instance based on robot_type
        
        Args:
            robot_type: Robot type
            send_message_callback: Send message callback function
            api_up: API upstream data, used to extract device constructor parameters
            **kwargs: Other parameters
            
        Returns:
            Device instance or None
        """
        for device_class in self._device_classes:
            if device_class._supports_robot_type(robot_type):
                # Extract constructor parameters from api_up
                constructor_params = self._extract_constructor_params(
                    device_class, robot_type, api_up)

                all_params = {
                    'send_message_callback': send_message_callback,
                    **constructor_params,
                }

                device = device_class(**all_params)
                device._set_robot_type(robot_type)
                return device

        return None

    def _extract_constructor_params(self, device_class, robot_type, api_up):
        """
        Extract device constructor parameters from api_up
        
        Args:
            device_class: Device class
            robot_type: Robot type
            api_up: API upstream data
            
        Returns:
            dict: Constructor parameters dictionary
        """
        params = {}

        if api_up is None:
            return params

        # Extract different parameters based on device class name
        class_name = device_class.__name__

        if class_name == 'ArmArcher':
            params['robot_type'] = robot_type
            # Get motor_count from api_up
            motor_count = self._get_motor_count_from_api_up(api_up)
            if motor_count is not None:
                params['motor_count'] = motor_count

        elif class_name == 'ChassisMaver':
            # Get motor_count from api_up
            motor_count = self._get_motor_count_from_api_up(api_up)
            if motor_count is not None:
                params['motor_count'] = motor_count

        elif class_name == 'ChassisMark2':
            # Get motor_count from api_up
            motor_count = self._get_motor_count_from_api_up(api_up)
            if motor_count is not None:
                params['motor_count'] = motor_count

        ## TODO: For adding different devices in the future, need to add new additional parameter extraction methods based on the parameters required by new classes.
        ## Error capture using try has been used earlier, if there are problems with parameter capture here, just raise directly.

        return params

    def _get_motor_count_from_api_up(self, api_up):
        """
        Get motor count from api_up
        
        Args:
            api_up: API upstream data
            
        Returns:
            int: Motor count or None
        """
        if api_up is None:
            return None

        # Use WhichOneof to check which status field is actually set in the oneof group
        status_field = api_up.WhichOneof('status')
        if status_field == 'arm_status':
            if hasattr(api_up.arm_status, 'motor_status'):
                motor_count = len(api_up.arm_status.motor_status)
                return motor_count
        elif status_field == 'base_status':
            if hasattr(api_up.base_status, 'motor_status'):
                motor_count = len(api_up.base_status.motor_status)
                return motor_count
        elif status_field == 'linear_lift_status':
            if hasattr(api_up.linear_lift_status, 'motor_status'):
                motor_count = len(api_up.linear_lift_status.motor_status)
                return motor_count
        elif status_field == 'rotate_lift_status':
            if hasattr(api_up.rotate_lift_status, 'motor_status'):
                motor_count = len(api_up.rotate_lift_status.motor_status)
                return motor_count
        else:
            log_err(f"No recognized status field is set (got: {status_field})")

        return None

    def get_supported_robot_types(self):
        """
        Get all supported robot types
        
        Returns:
            List: List of supported robot types
        """
        supported_types = []
        for device_class in self._device_classes:
            if hasattr(device_class, 'SUPPORTED_ROBOT_TYPES'):
                supported_types.extend(device_class.SUPPORTED_ROBOT_TYPES)
        return supported_types
