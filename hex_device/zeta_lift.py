#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from typing import Optional, List, Dict, Any, Tuple
import numpy as np
from collections import deque
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base import DeviceBase
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .motor_base import MotorBase, MotorError, MotorCommand, CommandType, Timestamp
from .generated.public_api_types_pb2 import BaseState
import time
import threading
import copy

class ZetaLift(DeviceBase, MotorBase):
    """
    ZetaLift class
    
    Inherits from DeviceBase and MotorBase, mainly implements mapping to BaseStatus
    This class corresponds to BaseStatus in proto, managing ZetaLift status and motor control
    
    """

    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtZetaVc2,
    ]

    def __init__(self,
                 motor_count: int,
                 robot_type: int,
                 name: str = "ZetaLift",
                 control_hz: int = 500,
                 send_message_callback=None,
                 ):
        """
        Initialize ZetaLift
        
        Args:
            motor_count: Number of motors
            name: Device name
            control_hz: Control frequency
            send_message_callback: Callback function for sending messages, used to send downstream messages
        """
        DeviceBase.__init__(self, name, send_message_callback)
        MotorBase.__init__(self, motor_count, name)
        self.name = name or "ZetaLift"
        self._set_robot_type(robot_type)
        self._control_hz = control_hz

        # ZetaLift status
        self._status_lock = threading.Lock()
        self._my_session_id = 0
        self._calibrated = True # nouse now
        self._state = public_api_types_pb2.LsBrake
        self._max_pos = None
        self._min_pos = None
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        self._last_update_time = None

        # Control related
        self._check_timeout = True
        self._is_timeout = False
        self._target_speed = None
        self._command_timeout = 0.2  # 100ms timeout
        self._last_command_time = None
        self.__command_lock = threading.Lock()
        self._send_calibrate: Optional[bool] = None

        # Robot type - will be set when matched
        self.robot_type = None

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
        Initialize ZetaLift
        
        Returns:
            bool: Whether initialization was successful
        """
        try:
            return True
        except Exception as e:
            log_err(f"ZetaLift initialization failed: {e}")
            return False

    def _update(self, api_up_data, timestamp: Timestamp) -> bool:
        """
        Update ZetaLift data
        
        Args:
            api_up_data: Upstream data received from API (APIUp)
            
        Returns:
            bool: Whether update was successful
        """
        try:
            if not api_up_data.HasField('rotate_lift_status'):
                return False
            lift_status = api_up_data.rotate_lift_status

            # Update motor data
            self._push_motor_data(lift_status.motor_status, timestamp)

            with self._status_lock:
                # update my session id
                self._my_session_id = api_up_data.session_id
                # Update lift status
                self._calibrated = lift_status.calibrated
                self._base_state = lift_status.state
                self.max_pos = lift_status.max_pos
                self.min_pos = lift_status.min_pos

                # Update optional fields
                if lift_status.HasField('parking_stop_detail'):
                    self._parking_stop_detail = lift_status.parking_stop_detail
                else:
                    self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

            return True
        except Exception as e:
            log_err(f"ZetaLift data update failed: {e}")
            return False

    async def _periodic(self):
        """
        Periodic execution function
        
        Execute periodic tasks for the lift, including:
        - Status check
        - Command timeout check
        - Safety monitoring
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("ZetaLift init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                # check error
                if self.get_parking_stop_detail(
                ) != public_api_types_pb2.ParkingStopDetail():
                    if start_time - self.__last_warning_time > 1.0:
                        log_err(
                            f"emergency stop: {self.get_parking_stop_detail()}"
                        )
                        self.__last_warning_time = start_time

                # Check motor status
                if start_time - self.__last_warning_time > 1.0:
                    for i in range(self.motor_count):
                        if self.get_motor_state(i) == "error":
                            log_err(f"Error: Motor {i} error occurred")
                            self.__last_warning_time = start_time

                # prepare sending message
                with self._status_lock:
                    sc = self._send_calibrate
                    c = self._calibrated
                    self._send_calibrate = None
                    ts = self._target_speed
                    self._target_speed = None
                    lc = copy.deepcopy(self._last_command_time)

                if sc:
                    msg = self._construct_calibrate_message()
                    await self._send_message(msg)

                if ts is not None:
                    msg = self._construct_runtime_config_message(self._make_runtime_config(ts))
                    await self._send_message(msg)

                # send control message
                if c:
                    if lc is not None:
                        if self._check_timeout:
                            if start_time - lc > self._command_timeout:
                                self._is_timeout = True
                                msg = self._construct_motor_targets_message(MotorCommand.create_speed_command([0.0] * self.motor_count))
                            else:
                                self._is_timeout = False
                                msg = self._construct_motor_targets_message()
                        else:
                            msg = self._construct_motor_targets_message()

                        await self._send_message(msg)

            except Exception as e:
                log_err(f"lift periodic failed: {e}")

    ## control functions
    def calibrate(self):
        """
        Calibrate
        """
        with self._status_lock:
            self._send_calibrate = True

    def set_move_speed(self, speed: List[float]):
        """
        Set move speed
        """
        with self._status_lock:
            self._target_speed = np.abs(speed)

    ## construct message
    def _construct_calibrate_message(self) -> public_api_down_pb2.APIDown:
        """
        Construct calibrate message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.RotateLiftCommand()
        lift_command.calibrate = True
        msg.rotate_lift_command.CopyFrom(lift_command)
        return msg

    def _construct_motor_targets_message(self, custom_command: MotorCommand = None) -> public_api_down_pb2.APIDown:
        """
        Construct motor targets message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.RotateLiftCommand()
        pulse_per_rotation = self.get_motor_pulse_per_rotations()
        with self.__command_lock:
            motor_targets = super()._construct_target_motor_msg(pulse_per_rotation, custom_command)
        lift_command.motor_targets.CopyFrom(motor_targets)
        msg.rotate_lift_command.CopyFrom(lift_command)
        return msg

    def _construct_runtime_config_message(self, runtime_config: public_api_types_pb2.RotateLiftRuntimeConfig) -> public_api_down_pb2.APIDown:
        """
        Construct runtime config message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.RotateLiftCommand()
        lift_command.runtime_config.CopyFrom(runtime_config)
        msg.rotate_lift_command.CopyFrom(lift_command)
        return msg

    def _make_runtime_config(self, pos_mode_max_speed: List[float]) -> public_api_types_pb2.RotateLiftRuntimeConfig:
        """
        Make runtime config
        """
        valid_speed = []
        for speed in pos_mode_max_speed:
            valid_speed.append(max(0, speed))
        runtime_config = public_api_types_pb2.RotateLiftRuntimeConfig()
        runtime_config.pos_mode_max_speed.extend(valid_speed)
        return runtime_config

    # state getters
    def is_calibrated(self) -> bool:
        """Check if calibrated"""
        with self._status_lock:
            return self._calibrated

    def get_joint_limits(self) -> Optional[List[List[float]]]:
        """
        Get joint limits
        Returns:
            Optional[List[List[float]]]: Joint limits
            The list is in the format of [[min_pos, max_pos], [min_vel, max_vel], [min_acc, max_acc]]
        """
        # 524288 pulses per rotation. Acc: 80000. Vel: 60000.
        joint_limits = []
        with self._status_lock:
            for i in range(len(self._max_pos)):
                joint_limits.append([self._min_pos[i], self._max_pos[i], -0.7186889648, 0.7186889648, -0.9582519531, 0.9582519531])
        return copy.deepcopy(joint_limits)

    def get_state(self) -> str:
        """Get lift state"""
        lift_state_descriptor = public_api_types_pb2.LiftState.DESCRIPTOR
        with self._status_lock:
            return lift_state_descriptor.values_by_number[self._state].name

    def get_my_session_id(self) -> int:
        """Get my session id"""
        with self._status_lock:
            return self._my_session_id

    def get_parking_stop_detail(self) -> public_api_types_pb2.ParkingStopDetail:
        """Get parking stop details"""
        with self._status_lock:
            return copy.deepcopy(self._parking_stop_detail)

    def motor_command(self, command_type: CommandType, values: List[float]):
        """
        Set lift command
        
        Args:
            command_type: Command type
            values: List of command values
        """
        if command_type == CommandType.TORQUE or command_type == CommandType.MIT:
            raise ValueError("ZetaLift does not support torque or MIT command")
        super().motor_command(command_type, values)
        with self._status_lock:
            self._last_command_time = time.perf_counter()

    def get_status_summary(self) -> Dict[str, Any]:
        """Get lift status summary"""
        summary = super().get_device_summary()

        # Add lift-specific information
        lift_summary = {
            'calibrated':
            self._calibrated,
            'state':
            public_api_types_pb2.LiftState.Name(self._state),
            'max_pos':
            self._max_pos,
            'min_pos':
            self._min_pos,
            'parking_stop_detail':
            self._parking_stop_detail,
        }

        summary.update(lift_summary)
        return summary

    def __str__(self) -> str:
        """String representation"""
        state_name = public_api_types_pb2.LiftState.Name(self._state)
        return f"{self.name}(State:{state_name}, Motors:{self.motor_count}, Calibrated={self._calibrated})"

    def __repr__(self) -> str:
        """Detailed string representation"""
        state_name = public_api_types_pb2.LiftState.Name(self._state)
        return f"ZetaLift(motor_count={self.motor_count}, name='{self.name}', state={state_name}, calibrated={self._calibrated})"
