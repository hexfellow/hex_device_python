#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import threading
import time
import numpy as np
from typing import Optional, Tuple, List, Dict, Any, Union
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base import DeviceBase
from .motor_base import MitMotorCommand, MotorBase, MotorError, MotorCommand, CommandType
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .generated.public_api_types_pb2 import (ArmStatus)
from .arm_config import get_arm_config, ArmConfig, arm_config_manager
import copy


class Arm(DeviceBase, MotorBase):
    """
    Arm class

    Inherits from DeviceBase and MotorBase, mainly implements control of Arm

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
                 name: str = "Arm",
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

        self.name = name or "Arm"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._arm_series = robot_type

        # arm status
        self._status_lock = threading.Lock()
        self._arm_mode = public_api_types_pb2.ArmMode.AmBrake
        self._api_control_initialized = False
        self._calibrated = False
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        self._session_holder = 0
        self._previous_session_holder = None

        # Control related
        self._command_timeout_check = True
        self._last_command_time = None
        self._command_timeout = 0.3  # 300ms
        self.__last_warning_time = time.perf_counter()  # last log warning time
        self._my_session_id = 0   # my session id, was assigned by server
        self.__send_init: Optional[bool] = None

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
            # self.start()
            return True
        except Exception as e:
            log_err(f"Arm initialization failed: {e}")
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
            if api_up_data.HasField('log'):
                log_warn(f"Arm: Get log from server: {api_up_data.log}")

            if not api_up_data.HasField('arm_status'):
                return False
            arm_status = api_up_data.arm_status

            with self._status_lock:
                # update my session id
                self._my_session_id = api_up_data.session_id
                # Update robotic arm status
                self._arm_mode = arm_status.current_mode
                self._api_control_initialized = arm_status.api_control_initialized
                self._calibrated = arm_status.calibrated
                self._session_holder = arm_status.session_holder

                if self._session_holder != self._previous_session_holder:
                    if self._session_holder == self._my_session_id:
                        log_warn(f"Arm: You can control the arm now! Your session ID: {self._session_holder}")
                    else:
                        log_warn(f"Arm: Can not control the arm, now holder is ID: {self._session_holder}, waiting...")
                self._previous_session_holder = self._session_holder

                if arm_status.HasField('parking_stop_detail'):
                    self._parking_stop_detail = arm_status.parking_stop_detail
                else:
                    self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

            # Update motor data
            self._update_motor_data_from_arm_status(arm_status)
            self.set_has_new_data()
            return True
        except Exception as e:
            log_err(f"Arm data update failed: {e}")
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
        log_info("Arm init success")
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
                        log_warn(f"You have disconnected from arm, trying to connect again.")
                        msg = self._construct_clear_parking_stop_message()
                        await self._send_message(msg)
                        # when timeout, the session holder will be release, should re-api-control-initialize again
                        self.start()

                # check motor error
                for i in range(self.motor_count):
                    if self.get_motor_state(i) == "error":
                        log_err(f"Warning: Motor {i} error occurred")

                # prepare sending message
                with self._status_lock:
                    s = self.__send_init
                    a = self._api_control_initialized
                    c = self._calibrated
                    sh = self._session_holder
                    mi = self._my_session_id

                # print(f"api_control_initialized: {a}, calibrated: {c}, session holder: {sh}, my session id: {mi}")
                
                ## send init message
                if s is None:
                    pass
                elif s:
                    msg = self._construct_init_message()
                    await self._send_message(msg)
                    self.__send_init = None
                elif not s:
                    msg = self._construct_init_message(False)
                    await self._send_message(msg)
                    self.__send_init = None

                ## check if is holder:
                if sh != mi:
                    if start_time - self.__last_warning_time > 3.0:
                        log_warn(f"Arm: Waiting to get the control of the arm...")
                        self.__last_warning_time = start_time
                    continue
                
                if a == True and c == True:
                    ### no command
                    if self._last_command_time is None:
                        msg = self._construct_init_message()
                        await self._send_message(msg)
                    ### command timeout
                    elif self._command_timeout_check and (start_time -
                          self._last_command_time) > self._command_timeout:
                        try:
                            motor_msg = self._construct_custom_motor_msg(
                                CommandType.BRAKE, [True] * self.motor_count)
                            msg = self._construct_custom_joint_command_msg(motor_msg)
                            await self._send_message(msg)
                        except Exception as e:
                            log_err(f"Arm failed to construct custom joint command message: {e}")
                            continue
                    ### normal command
                    else:
                        try:
                            msg = self._construct_joint_command_msg()
                            await self._send_message(msg)
                        except Exception as e:
                            log_err(f"Arm failed to construct joint command message: {e}")
                            continue
                elif c == False:
                    # If there is anything that requires special action, modify this calibrate sending logic.
                    msg = self._construct_calibrate_message()
                    await self._send_message(msg)

            except Exception as e:
                log_err(f"Arm periodic task exception: {e}")
                continue

    def start(self):
        """
        Set init message to True to start the arm
        """
        with self._status_lock:
            self.__send_init = True

    def stop(self):
        """
        Set init message to False to stop the arm
        """
        with self._status_lock:
            self.__send_init = False

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
        return copy.deepcopy(mit_commands)

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
        
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        motor_targets = self._construct_target_motor_msg(self._pulse_per_rotation, self._period)
        arm_exclusive_command.motor_targets.CopyFrom(motor_targets)
        arm_exclusive_command.target_mode = public_api_types_pb2.ArmMode.AmApiControl

        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_custom_joint_command_msg(self, motor_msg: public_api_types_pb2.MotorTargets) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a custom joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()

        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_exclusive_command.motor_targets.CopyFrom(motor_msg)
        arm_exclusive_command.target_mode = public_api_types_pb2.ArmMode.AmApiControl

        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def get_parking_stop_detail(
            self) -> public_api_types_pb2.ParkingStopDetail:
        """Get parking stop details"""
        return copy.deepcopy(self._parking_stop_detail)

    # msg constructor
    def _construct_init_message(self, api_control_initialize: bool = True) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a init message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_exclusive_command.api_control_initialize = api_control_initialize
        arm_exclusive_command.target_mode = public_api_types_pb2.ArmMode.AmApiControl
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_calibrate_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a calibrate message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_exclusive_command.calibrate = True
        arm_exclusive_command.target_mode = public_api_types_pb2.ArmMode.AmApiControl
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_clear_parking_stop_message(self):
        """
        @brief: For constructing a clear_parking_stop message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_exclusive_command.clear_parking_stop = True
        arm_exclusive_command.target_mode = public_api_types_pb2.ArmMode.AmApiControl
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
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
        validated_command = copy.deepcopy(command)

        if validated_command.command_type == CommandType.POSITION:
            validated_positions = self.validate_joint_positions(command.position_command, dt)
            validated_command.position_command = validated_positions
        elif validated_command.command_type == CommandType.SPEED:
            validated_velocities = self.validate_joint_velocities(command.speed_command, dt)
            validated_command.speed_command = validated_velocities

        motor_targets = super()._construct_target_motor_msg(pulse_per_rotation, validated_command)
        
        return motor_targets

    # Configuration related methods
    def get_session_holder(self) -> int:
        """Get session holder"""
        with self._status_lock:
            return self._session_holder

    def get_my_session_id(self) -> int:
        """Get my session id"""
        with self._status_lock:
            return self._my_session_id
            
    def get_arm_config(self) -> Optional[ArmConfig]:
        """Get current robotic arm configuration"""
        return copy.deepcopy(get_arm_config(self._arm_series))

    def get_joint_limits(self) -> Optional[List[List[float]]]:
        """Get joint limits"""
        return copy.deepcopy(arm_config_manager.get_joint_limits(self._arm_series))

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
                log_common(f"Arm: Initialize current motor positions: {current_positions}")
            else:
                log_warn(f"Arm: Current motor positions count({len(current_positions)}) does not match the target positions count({len(positions)})")
        
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
        return copy.deepcopy(arm_config_manager.get_joint_names(self._arm_series))

    def get_expected_motor_count(self) -> Optional[int]:
        """Get expected motor count"""
        return copy.deepcopy(arm_config_manager.get_motor_count(self._arm_series))

    def check_motor_count_match(self) -> bool:
        """Check if motor count matches configuration"""
        expected_count = self.get_expected_motor_count()
        if expected_count is None:
            return False
        return self.motor_count == expected_count

    def get_arm_series(self) -> int:
        """Get robotic arm series"""
        return copy.deepcopy(self._arm_series)

    def get_arm_name(self) -> Optional[str]:
        """Get robotic arm name"""
        config = self.get_arm_config()
        return copy.deepcopy(config.name) if config else None

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
                log_common(f"Arm: reload arm config success: {config_data.get('name', 'unknown')}")
            else:
                log_err(f"Arm: reload arm config from dict failed: {config_data.get('name', 'unknown')}")
            return success
        except Exception as e:
            log_err(f"Arm: reload arm config from dict exception: {e}")
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
