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
from .device_base import DeviceBase
from .motor_base import MitMotorCommand, MotorBase, MotorError, MotorCommand, CommandType
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .generated.public_api_types_pb2 import (ArmStatus)
from .arm_config import get_arm_config, ArmConfig, arm_config_manager
from copy import deepcopy


class ArmArcher(DeviceBase, MotorBase):
    """
    ArmArcher class

    Inherits from DeviceBase and MotorBase, mainly implements control of ArmArcher

    Supported robot types:
    - RtArmSaberD6X: Custom PCW vehicle
    - RtArmSaberD7X: PCW vehicle
    """

    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtArmArcherD6Y,
        public_api_types_pb2.RobotType.RtArmSaberD6X,
        public_api_types_pb2.RobotType.RtArmSaberD7X,
    ]

    ARM_SERIES_TO_ROBOT_TYPE = {
        9: public_api_types_pb2.RobotType.RtArmSaber750d3Lr3DmDriver,
        10: public_api_types_pb2.RobotType.RtArmSaber750d4Lr3DmDriver,
        11: public_api_types_pb2.RobotType.RtArmSaber750h3Lr3DmDriver,
        12: public_api_types_pb2.RobotType.RtArmSaber750h4Lr3DmDriver,
        14: public_api_types_pb2.RobotType.RtArmSaberD6X,
        15: public_api_types_pb2.RobotType.RtArmSaberD7X,
        16: public_api_types_pb2.RobotType.RtArmArcherD6Y,
    }

    def __init__(self,
                 robot_type,
                 motor_count,
                 name: str = "ArmArcher",
                 control_hz: int = 500,
                 send_message_callback=None):
        """
        Initialize chassis Maver
        
        Args:
            motor_count: Number of motors
            robot_type: Robotic arm series
            name: Device name
            control_hz: Control frequency
            send_message_callback: Callback function for sending messages, used to send downstream messages
        """
        DeviceBase.__init__(self, name, send_message_callback)
        MotorBase.__init__(self, motor_count, name)

        self.name = name or "ArmArcher"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._arm_series = robot_type

        # arm status
        self._arm_state = public_api_types_pb2.ArmState.AsParked
        self._api_control_initialized = False
        self._calibrated = False
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

        # Control related
        self._command_timeout_check = True
        self._last_command_time = None
        self._command_timeout = 0.3  # 300ms
        self.__last_warning_time = time.perf_counter()  # last log warning time

    def _set_robot_type(self, robot_type):
        """
        Set robot type
        
        Args:
            robot_type: Robot type
        """
        if robot_type in self.SUPPORTED_ROBOT_TYPES:
            self.robot_type = robot_type
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")

    @classmethod
    def _supports_robot_type(cls, robot_type):
        """
        Check if the specified robot type is supported
        
        Args:
            robot_type: Robot type
            
        Returns:
            bool: Whether it is supported
        """
        return robot_type in cls.SUPPORTED_ROBOT_TYPES

    async def _init(self) -> bool:
        """
        Initialize robotic arm
        
        Returns:
            bool: Whether initialization was successful
        """
        try:
            msg = self._construct_init_message()
            await self._send_message(msg)
            msg = self._construct_calibrate_message()
            await self._send_message(msg)
            return True
        except Exception as e:
            log_err(f"ArmArcher initialization failed: {e}")
            return False

    def _update(self, api_up_data) -> bool:
        """
        Update robotic arm data
        
        Args:
            api_up_data: Upstream data received from API (APIUp)
            
        Returns:
            bool: Whether update was successful
        """
        try:
            if not api_up_data.HasField('arm_status'):
                return False

            arm_status = api_up_data.arm_status

            # Update robotic arm status
            self._arm_state = arm_status.state
            self._api_control_initialized = arm_status.api_control_initialized
            self._calibrated = arm_status.calibrated

            if arm_status.HasField('parking_stop_detail'):
                self._parking_stop_detail = arm_status.parking_stop_detail
            else:
                self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

            # Update motor data
            self._update_motor_data_from_arm_status(arm_status)
            self.set_has_new_data()
            return True
        except Exception as e:
            log_err(f"ArmArcher data update failed: {e}")
            return False

    def _update_motor_data_from_arm_status(self, arm_status: ArmStatus):
        motor_status_list = arm_status.motor_status

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
        Periodic execution function
        
        Execute periodic tasks for the robotic arm, including:
        - Status check
        - Command timeout check
        - Safety monitoring
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("ArmArcher init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                # check arm error
                error = self.get_parking_stop_detail()
                if error != public_api_types_pb2.ParkingStopDetail():
                    if start_time - self.__last_warning_time > 1.0:
                        log_err(f"emergency stop: {error}")
                        self.__last_warning_time = start_time

                    # auto clear api communication timeout
                    if error.category == public_api_types_pb2.ParkingStopCategory.PscAPICommunicationTimeout:
                        msg = self._construct_clear_parking_stop_message()
                        await self._send_message(msg)

                # check motor error
                for i in range(self.motor_count):
                    if self.get_motor_state(i) == "error":
                        log_err(f"Warning: Motor {i} error occurred")

                # prepare sending message
                if self._api_control_initialized == False:
                    msg = self._construct_init_message()
                    await self._send_message(msg)
                elif self._calibrated == False:
                    # If there is anything that requires special action, modify this calibrate sending logic.
                    msg = self._construct_calibrate_message()
                    await self._send_message(msg)
                else:
                    # no command
                    if self._last_command_time is None:
                        msg = self._construct_init_message()
                        await self._send_message(msg)
                    # command timeout
                    elif self._command_timeout_check and (start_time -
                          self._last_command_time) > self._command_timeout:
                        try:
                            motor_msg = self._construct_custom_motor_msg(
                                CommandType.BRAKE, [True] * self.motor_count)
                            msg = self._construct_custom_joint_command_msg(motor_msg)
                            await self._send_message(msg)
                        except Exception as e:
                            log_err(f"ArmArcher failed to construct custom joint command message: {e}")
                            continue
                    # normal command
                    else:
                        try:
                            msg = self._construct_joint_command_msg()
                            await self._send_message(msg)
                        except Exception as e:
                            log_err(f"ArmArcher failed to construct joint command message: {e}")
                            continue

            except Exception as e:
                log_err(f"ArmArcher periodic task exception: {e}")
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
        
        super().motor_command(command_type, values)
        self._last_command_time = time.perf_counter()

    def _construct_joint_command_msg(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        motor_targets = self._construct_target_motor_msg(self._pulse_per_rotation, self._period)
        arm_command.motor_targets.CopyFrom(motor_targets)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_custom_joint_command_msg(self, motor_msg: public_api_types_pb2.MotorTargets) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a custom joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_command.motor_targets.CopyFrom(motor_msg)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def get_parking_stop_detail(
            self) -> public_api_types_pb2.ParkingStopDetail:
        """Get parking stop details"""
        return deepcopy(self._parking_stop_detail)

    # msg constructor
    def _construct_init_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a init message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_command.api_control_initialize = True
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_calibrate_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a calibrate message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_command.calibrate = True
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_clear_parking_stop_message(self):
        """
        @brief: For constructing a clear_parking_stop message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_command.clear_parking_stop = True
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_target_motor_msg(
            self,
            pulse_per_rotation,
            dt,
            command: MotorCommand = None) -> public_api_types_pb2.MotorTargets:
        """Construct downstream message"""
        # if no new command, use the last command 
        if command is None:
            with self._command_lock:
                if self._target_command is None:
                    raise ValueError(
                        "Construct down msg failed, No target command")
                command = self._target_command

        # validate joint positions and velocities
        validated_command = deepcopy(command)

        if validated_command.command_type == CommandType.POSITION:
            validated_positions = self.validate_joint_positions(command.position_command, dt)
            validated_command.position_command = validated_positions
        elif validated_command.command_type == CommandType.SPEED:
            validated_velocities = self.validate_joint_velocities(command.speed_command, dt)
            validated_command.speed_command = validated_velocities

        motor_targets = super()._construct_target_motor_msg(pulse_per_rotation, validated_command)
        
        return motor_targets

    # Configuration related methods
    def get_arm_config(self) -> Optional[ArmConfig]:
        """Get current robotic arm configuration"""
        return deepcopy(get_arm_config(self._arm_series))

    def get_joint_limits(self) -> Optional[List[List[float]]]:
        """Get joint limits"""
        return deepcopy(arm_config_manager.get_joint_limits(self._arm_series))

    def validate_joint_positions(self,
                                 positions: List[float],
                                 dt: float = 0.002) -> List[float]:
        """
        Validate whether joint positions are within limit range and return corrected position list
        
        Args:
            positions: Target position list (rad)
            dt: Time step (s), used for velocity limit calculation
            
        Returns:
            List[float]: Corrected position list
        """
        last_positions = arm_config_manager.get_last_positions(self._arm_series)
        
        if last_positions is None:
            current_positions = self.get_motor_positions()
            if len(current_positions) == len(positions):
                arm_config_manager.set_last_positions(self._arm_series, current_positions)
                log_common(f"ArmArcher: Initialize current motor positions: {current_positions}")
            else:
                log_warn(f"ArmArcher: Current motor positions count({len(current_positions)}) does not match the target positions count({len(positions)})")
        
        return arm_config_manager.validate_joint_positions(
            self._arm_series, positions, dt)

    def validate_joint_velocities(self,
                                  velocities: List[float],
                                  dt: float = 0.002) -> List[float]:
        """
        Validate whether joint velocities are within limit range and return corrected velocity list
        """
        return arm_config_manager.validate_joint_velocities(
            self._arm_series, velocities, dt)

    def get_joint_names(self) -> Optional[List[str]]:
        """Get joint names"""
        return deepcopy(arm_config_manager.get_joint_names(self._arm_series))

    def get_expected_motor_count(self) -> Optional[int]:
        """Get expected motor count"""
        return deepcopy(arm_config_manager.get_motor_count(self._arm_series))

    def check_motor_count_match(self) -> bool:
        """Check if motor count matches configuration"""
        expected_count = self.get_expected_motor_count()
        if expected_count is None:
            return False
        return self.motor_count == expected_count

    def get_arm_series(self) -> int:
        """Get robotic arm series"""
        return deepcopy(self._arm_series)

    def get_arm_name(self) -> Optional[str]:
        """Get robotic arm name"""
        config = self.get_arm_config()
        return deepcopy(config.name) if config else None

    def reload_arm_config_from_dict(self, config_data: dict) -> bool:
        """
        Reload current robotic arm configuration parameters from dictionary data. Parameters are used to provide limiting indicators for velocity and position commands.
        
        Args:
            config_data: Configuration data dictionary
            
        Returns:
            bool: Whether reload was successful
        """
        try:
            success = arm_config_manager.reload_from_dict(
                self._arm_series, config_data)
            if success:
                log_common(f"ArmArcher: reload arm config success: {config_data.get('name', 'unknown')}")
            else:
                log_err(f"ArmArcher: reload arm config from dict failed: {config_data.get('name', 'unknown')}")
            return success
        except Exception as e:
            log_err(f"ArmArcher: reload arm config from dict exception: {e}")
            return False

    def set_initial_positions(self, positions: List[float]):
        """
        Set initial positions of robotic arm, used for velocity limit calculation
        
        Args:
            positions: Initial position list (rad)
        """
        arm_config_manager.set_initial_positions(self._arm_series, positions)

    def set_initial_velocities(self, velocities: List[float]):
        """
        Set initial velocities of robotic arm, used for acceleration limit calculation
        
        Args:
            velocities: Initial velocity list (rad/s)
        """
        arm_config_manager.set_initial_velocities(self._arm_series, velocities)

    def get_last_positions(self) -> Optional[List[float]]:
        """
        Get last position record
        
        Returns:
            List[float]: Last position list, returns None if no record exists
        """
        return arm_config_manager.get_last_positions(self._arm_series)

    def get_last_velocities(self) -> Optional[List[float]]:
        """
        Get last velocity record
        
        Returns:
            List[float]: Last velocity list, returns None if no record exists
        """
        return arm_config_manager.get_last_velocities(self._arm_series)

    def clear_position_history(self):
        """Clear position history records"""
        arm_config_manager.clear_position_history(self._arm_series)

    def clear_velocity_history(self):
        """Clear velocity history records"""
        arm_config_manager.clear_velocity_history(self._arm_series)

    def clear_motion_history(self):
        """Clear all motion history records (position and velocity)"""
        arm_config_manager.clear_motion_history(self._arm_series)
