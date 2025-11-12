#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import time
import numpy as np
from typing import Optional, Tuple, List, Dict, Any, Union
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base_optional import OptionalDeviceBase
from .motor_base import MitMotorCommand, MotorBase, MotorError, MotorCommand, CommandType
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from copy import deepcopy
from .generated.public_api_types_pb2 import (HandStatus)
import threading


class Hands(OptionalDeviceBase, MotorBase):
    """
    Hands class - Optional device for processing hand_status

    Inherits from OptionalDeviceBase and MotorBase, mainly implements control of Hands
    This class processes the optional hand_status field from APIUp messages.

    Supported hand types:
    - HtGp100: GP100 hand type
    """

    SUPPORTED_DEVICE_TYPE = [
        public_api_types_pb2.SecondaryDeviceType.SdtHandGp100,
    ]

    DEVICE_ID_TO_DEVICE_TYPE = {
        1: public_api_types_pb2.SecondaryDeviceType.SdtHandGp100,
    }

    def __init__(self,
                 device_id,
                 device_type,
                 motor_count,
                 send_message_callback,
                 name: str = "Hands",
                 control_hz: int = 250,
                 read_only: bool = False,
                 ):
        """
        Initialize Hands device
        
        Args:
            device_id: Device ID (SecondaryDeviceType enum)
            device_type: Device type (e.g., 'SdtUnknown', 'SdtHandGp100', 'SdtGamepad', 'SdtImuY200')
            motor_count: Number of motors
            name: Device name
            control_hz: Control frequency
            read_only: Whether this device is read-only (read only will not create periodic task)
            send_message_callback: Callback function for sending messages, used to send downstream messages
        """
        OptionalDeviceBase.__init__(self, read_only, name, send_message_callback, device_id)
        MotorBase.__init__(self, motor_count, name)

        self.name = name or "Hands"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._device_id = device_id
        self._device_type = device_type

        # hand status
        self._api_control_initialized = False
        self._calibrated = False

        # Control related
        self._command_timeout_check = True
        self._last_command_time = None
        self._command_timeout = 0.3  # 300ms
        self.__last_warning_time = time.perf_counter()  # last log warning time
        # the last command sent to the device
        self._last_command_send = None

        ## limit and step
        self._config_lock = threading.Lock()
        if self._device_type == public_api_types_pb2.SecondaryDeviceType.SdtHandGp100:
            self._hands_limit = [0.0, 1.335, -np.inf, np.inf, -np.inf, np.inf]
            self._max_torque = 3.0
            self._positon_step = 0.02

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
        try:
            self.motor_command(CommandType.POSITION, [0.0] * self.motor_count)
            return True
        except Exception as e:
            log_err(f"Hands initialization failed: {e}")
            return False

    def _update_optional_data(self, device_type, device_status: public_api_types_pb2.SecondaryDeviceStatus) -> bool:
        """
        Update hands device with optional message data
        
        Args:
            device_type: Should be equal to self._device_type
            device_status: The SecondaryDeviceStatus from APIUp
            
        Returns:
            bool: Whether update was successful
        """
        if device_type != self._device_type:
            log_warn(f"Warning: Hands device type mismatch, expected {self._device_type}, actual {device_type}")
            return False
            
        try:
            # Update motor data
            self._update_motor_data_from_hands_status(device_status.hand_status)
            self._update_timestamp()
            return True
        except Exception as e:
            log_err(f"Hands data update failed: {e}")
            return False

    def _update_motor_data_from_hands_status(self, hand_status: HandStatus):
        motor_status_list = hand_status.motor_status

        if len(motor_status_list) != self.motor_count:
            log_warn(
                f"Warning: Motor count mismatch, expected {self.motor_count}, actual {len(motor_status_list)}")
            return

        # Parse motor data
        positions = []  # encoder position
        velocities = []  # rad/s
        torques = []  # Nm
        driver_temperature = []
        motor_temperature = []
        pulse_per_rotation = []
        wheel_radius = []
        voltage = []
        error_codes = []
        current_targets = []

        for motor_status in motor_status_list:
            positions.append(motor_status.position)
            velocities.append(motor_status.speed)
            torques.append(motor_status.torque)
            pulse_per_rotation.append(motor_status.pulse_per_rotation)
            wheel_radius.append(motor_status.wheel_radius)
            current_targets.append(motor_status.current_target)

            driver_temp = motor_status.driver_temperature if motor_status.HasField(
                'driver_temperature') else 0.0
            motor_temp = motor_status.motor_temperature if motor_status.HasField(
                'motor_temperature') else 0.0
            volt = motor_status.voltage if motor_status.HasField(
                'voltage') else 0.0
            driver_temperature.append(driver_temp)
            motor_temperature.append(motor_temp)
            voltage.append(volt)

            error_code = None
            if motor_status.error:
                error_code = motor_status.error[0]
            error_codes.append(error_code)

        self.update_motor_data(positions=positions,
                               velocities=velocities,
                               torques=torques,
                               driver_temperature=driver_temperature,
                               motor_temperature=motor_temperature,
                               voltage=voltage,
                               pulse_per_rotation=pulse_per_rotation,
                               wheel_radius=wheel_radius,
                               error_codes=error_codes,
                               current_targets=current_targets)

    async def _periodic(self):
        """
        Main control loop for hands device
        This method implements the original periodic control logic
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("Hands init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                # check motor error
                if start_time - self.__last_warning_time > 1.0:
                    for i in range(self.motor_count):
                        if self.get_motor_state(i) == "error":
                            log_err(f"Error: Motor {i} error occurred")
                            self.__last_warning_time = start_time

                # prepare sending message
                # command timeout
                if self._command_timeout_check and (start_time -
                        self._last_command_time) > self._command_timeout:
                    try:
                        motor_msg = self._construct_custom_motor_msg(
                            CommandType.BRAKE, [True] * self.motor_count)
                        msg = self._construct_custom_joint_command_msg(motor_msg)
                        await self._send_message(msg)
                    except Exception as e:
                        log_err(f"Hands failed to construct custom joint command message: {e}")
                        continue
                # normal command
                else:
                    try:
                        msg = self._construct_joint_command_msg()
                        await self._send_message(msg)
                    except Exception as e:
                        log_err(f"Hands failed to construct joint command message: {e}")
                        continue

            except Exception as e:
                log_err(f"Hands periodic task exception: {e}")
                continue
        
    # Robotic arm specific methods
    def command_timeout_check(self, check_or_not: bool = True):
        """
        Set whether to check command timeout
        """
        self._command_timeout_check = check_or_not

    def construct_mit_command(self, 
            pos: Union[np.ndarray, List[float]], 
            speed: Union[np.ndarray, List[float]], 
            torque: Union[np.ndarray, List[float]], 
            kp: Union[np.ndarray, List[float]], 
            kd: Union[np.ndarray, List[float]]
        ) -> List[MitMotorCommand]:
        """
        Construct MIT command
        """
        mit_commands = []
        for i in range(self.motor_count):
            mit_commands.append(MitMotorCommand(position=pos[i], speed=speed[i], torque=torque[i], kp=kp[i], kd=kd[i]))
        return deepcopy(mit_commands)

    def motor_command(self, command_type: CommandType, values: Union[List[bool], List[float], List[MitMotorCommand], np.ndarray]):
        """
        Set motor command
        Note:
            1. Only when CommandType is POSITION or SPEED, will validate the values.
            2. When CommandType is BRAKE, the values can be any, but the length must be the same as the motor count.
        Args:
            command_type: Command type
            values: List of command values or numpy array
        """
        # Convert numpy array to list if needed
        if isinstance(values, np.ndarray):
            values = values.tolist()

        # limit position
        if command_type == CommandType.POSITION:
            values = [max(min(value, self._hands_limit[1]), self._hands_limit[0]) for value in values]

        super().motor_command(command_type, values)
        self._last_command_time = time.perf_counter()

    def set_positon_step(self, step: float):
        """
        Set position step
        """
        with self._config_lock:
            self._positon_step = deepcopy(step)

    def set_pos_torque(self, max_torque: float):
        """
        Set max torque
        """
        with self._config_lock:
            self._max_torque = deepcopy(max_torque)

    def _construct_joint_command_msg(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        hand_command = public_api_types_pb2.HandCommand()
        secondary_device_command = public_api_types_pb2.SecondaryDeviceCommand()
        # limit the torque of position command
        command = deepcopy(self._target_command)

        if command.command_type == CommandType.POSITION:
            # check the torque if valid
            torques = self.get_motor_torques()
            now_pos = self.get_motor_positions()
            with self._config_lock:
                positon_step = self._positon_step
                max_torque = self._max_torque

            if self._last_command_send is not None:
                last_command = self._last_command_send
            else:
                last_command = MotorCommand.create_position_command(now_pos)

            for i in range(self.motor_count):
                err = np.clip(command.position_command[i] - last_command.position_command[i], -positon_step, positon_step)
                if err > 0.0 and torques[i] < max_torque:
                    command.position_command[i] = last_command.position_command[i] + err
                elif err < 0.0 and torques[i] > -max_torque:
                    command.position_command[i] = last_command.position_command[i] + err
                else:
                    # max torque or reach the target position
                    command.position_command[i] = last_command.position_command[i]
            self._last_command_send = deepcopy(command)

        motor_targets = self._construct_target_motor_msg(self._pulse_per_rotation, command)
        hand_command.motor_targets.CopyFrom(motor_targets)
        secondary_device_command.device_id = self._device_id
        secondary_device_command.hand_command.CopyFrom(hand_command)
        msg.secondary_device_command.CopyFrom(secondary_device_command)
        return msg

    def _construct_custom_joint_command_msg(self, motor_msg: public_api_types_pb2.MotorTargets) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a custom joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        hand_command = public_api_types_pb2.HandCommand()
        secondary_device_command = public_api_types_pb2.SecondaryDeviceCommand()
        hand_command.motor_targets.CopyFrom(motor_msg)
        secondary_device_command.device_id = self._device_id
        secondary_device_command.hand_command.CopyFrom(hand_command)
        msg.secondary_device_command.CopyFrom(secondary_device_command)
        return msg

    # msg constructor
    def _construct_target_motor_msg(
            self,
            pulse_per_rotation,
            command: MotorCommand = None) -> public_api_types_pb2.MotorTargets:
        """Construct downstream message"""
        # if no new command, use the last command 
        if command is None:
            with self._command_lock:
                if self._target_command is None:
                    raise ValueError(
                        "Construct down msg failed, No target command")
                command = self._target_command

        motor_targets = super()._construct_target_motor_msg(pulse_per_rotation, command)
        
        return motor_targets

    # Configuration related methods
    def get_hand_type(self) -> int:
        """Get hand type"""
        return deepcopy(self._hand_type)

    def get_joint_limits(self) -> List[float]:
        """Get hands joint limits"""
        return deepcopy(self._hands_limit)

    def get_hands_summary(self) -> dict:
        """
        Get hands device summary including motor data
        
        Returns:
            dict: Hands device summary
        """
        summary = self.get_device_summary()
        summary.update({
            'hand_type': self._hand_type,
            'motor_count': self.motor_count,
            'control_hz': self._control_hz,
            'command_timeout_check': self._command_timeout_check,
            'calibrated': self._calibrated,
            'api_control_initialized': self._api_control_initialized,
            'motor_positions': self.get_motor_positions(),
            'motor_velocities': self.get_motor_velocities(),
            'motor_torques': self.get_motor_torques()
        })
        return summary
