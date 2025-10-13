#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
from typing import Optional, Tuple, List, Type, Dict, Any
from .device_base import DeviceBase
from .device_base_optional import OptionalDeviceBase
from .common_utils import log_err

class DeviceFactory:
    """
    Unified device factory class, responsible for creating and managing device instances
    Supports both robot_type-based devices (DeviceBase) and message_type-based devices (OptionalDeviceBase)
    """

    def __init__(self):
        # Traditional robot_type-based devices
        self._device_classes: List[Type[DeviceBase]] = []
        
        # Optional message_type-based devices
        self._optional_device_classes: Dict[str, Type[OptionalDeviceBase]] = {}

    def register_device_class(self, device_class):
        """
        Register device class (supports both DeviceBase and OptionalDeviceBase)
        
        Args:
            device_class: Device class
        """
        # Check if it's an OptionalDeviceBase
        if issubclass(device_class, OptionalDeviceBase):
            self.register_optional_device_class(device_class)
        # Check if it's a traditional DeviceBase
        elif hasattr(device_class, '_supports_robot_type'):
            self._device_classes.append(device_class)
        else:
            raise ValueError(
                f"Device class {device_class.__name__} must be either OptionalDeviceBase or support _supports_robot_type class method")

    def register_optional_device_class(self, device_class: Type[OptionalDeviceBase]):
        """
        Register an optional device class
        
        Args:
            device_class: The optional device class to register
        """
        if not issubclass(device_class, OptionalDeviceBase):
            raise ValueError(f"Device class {device_class.__name__} must inherit from OptionalDeviceBase")
            
        supported_types = device_class.get_supported_message_types_static()
        for message_type in supported_types:
            if message_type in self._optional_device_classes:
                raise ValueError(f"Message type '{message_type}' already registered with device: {self._optional_device_classes[message_type]}")
            self._optional_device_classes[message_type] = device_class

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

    def create_optional_device(self, message_type: str, send_message_callback=None, api_up=None) -> Optional[OptionalDeviceBase]:
        """
        Create an optional device instance for the specified message type
        
        Args:
            message_type: The message type to create device for
            send_message_callback: Callback function for sending messages
            api_up: API upstream data, used to extract device constructor parameters
            
        Returns:
            OptionalDeviceBase: Device instance or None if not supported
        """
        if message_type not in self._optional_device_classes:
            return None
            
        device_class = self._optional_device_classes[message_type]
        
        # Extract constructor parameters based on device class and api_up
        constructor_params = self._extract_optional_constructor_params_from_api_up(device_class, message_type, api_up)

        all_params = {
            'send_message_callback': send_message_callback,
            **constructor_params,
        }
        
        device_instance = device_class(**all_params)
        return device_instance

    def get_supported_message_types(self) -> List[str]:
        """
        Get all supported message types for optional devices
        
        Returns:
            List[str]: List of all registered message types
        """
        return list(self._optional_device_classes.keys())

    def _extract_optional_constructor_params_from_api_up(self, device_class: Type[OptionalDeviceBase], message_type: str, api_up) -> Dict[str, Any]:
        """
        Extract optional device constructor parameters from api_up
        
        Args:
            device_class: Optional device class
            message_type: Message type
            api_up: API upstream data
            
        Returns:
            dict: Constructor parameters dictionary
        """
        params = {}
        class_name = device_class.__name__
        
        if class_name == 'Hands':
            # Extract motor count from hand_status if available
            if api_up and hasattr(api_up, 'hand_status') and api_up.hand_status:
                if hasattr(api_up.hand_status, 'motor_status'):
                    motor_count = len(api_up.hand_status.motor_status)
                else:
                    raise ValueError(f"Hands device motor_status is not set")
                if hasattr(api_up.hand_status, 'hand_type'):
                    hand_type = api_up.hand_status.hand_type
                else:
                    raise ValueError(f"Hands device hand_type is not set")
            else:
                raise ValueError(f"Hands device hand_status is not set")
            
            params.update({
                'hand_type': hand_type,
                'motor_count': motor_count,
                'name': f"Hands_{message_type}",
            })

        #TODO: Add more optional device parameter extraction logic here as needed
        return params

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
            params['name'] = f"ArmArcher_{robot_type}"
            # Get motor_count from api_up
            motor_count = self._get_motor_count_from_api_up(api_up)
            if motor_count is not None:
                params['motor_count'] = motor_count

        elif class_name == 'Chassis':
            params['name'] = f"Chassis_{robot_type}"
            params['robot_type'] = robot_type
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
