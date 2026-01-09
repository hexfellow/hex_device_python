#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import copy
import time
import numpy as np
import asyncio
from typing import List, Optional, Union, Tuple

from .common_utils import delay, log_err, log_info, log_warn
from .device_base import DeviceBase
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .motor_base import MitMotorCommand, CommandType, Timestamp

class LinearLift(DeviceBase):
    """
    Linear Lift class

    Inherits from DeviceBase, mainly implements control of Linear Lift
    """

    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtIotaP1,
    ]

    def __init__(self, motor_count: int, robot_type: int, name: str = "Lift", control_hz: int = 500, send_message_callback=None):
        """
        Initialize Linear Lift
        """
        DeviceBase.__init__(self, name, send_message_callback)
        
        self.name = name or "Linear Lift"
        self._control_hz = control_hz
        self.motor_count = motor_count
        self._period = 1.0 / control_hz
        self._set_robot_type(robot_type)

        # lift status
        self._calibrated = False
        self._state = public_api_types_pb2.LsBrake
        self._max_pos = None  # The max position you can set using software. encoder position
        self._current_pos = None  # encoder position
        self._pulse_per_rotation = None  # pulse / m
        self._max_speed = None # pulse / s, max target speed can be set
        self._move_speed = None # pulse / s
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        self._custom_button_pressed = False
        self._last_update_time = None  # update with data lock, only check has now positions

        # Control related
        self._last_warning_time = time.perf_counter()
        self._send_calibrate: Optional[bool] = None
        self._send_brake: Optional[bool] = None
        self._target_pos: Optional[int] = None  # encoder position
        self._target_speed: Optional[int] = None  # pulse / s
        
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
        Initialize lift
        
        Returns:
            bool: Whether initialization was successful
        """
        try:
            # self.start()
            return True
        except Exception as e:
            log_err(f"Lift initialization failed: {e}")
            return False

    def _update(self, api_up_data, timestamp: Timestamp) -> bool:
        """
        Update lift data
        
        Args:
            api_up_data: Upstream data received from API (APIUp)
            
        Returns:
            bool: Whether update was successful
        """
        try:
            if not api_up_data.HasField('linear_lift_status'):
                return False
            lift_status = api_up_data.linear_lift_status

            with self._data_lock:
                self._last_update_time = timestamp
                self._current_pos = lift_status.current_pos
                self._move_speed = lift_status.speed
                if lift_status.HasField('custom_button_pressed'):
                    self._custom_button_pressed = lift_status.custom_button_pressed
                else:
                    self._custom_button_pressed = False

            with self._status_lock:
                self._calibrated = lift_status.calibrated
                self._state = lift_status.state
                self._pulse_per_rotation = lift_status.pulse_per_rotation  # pulse / m
                self._max_pos = lift_status.max_pos
                self._max_speed = lift_status.max_speed

                if lift_status.HasField('parking_stop_detail'):
                    self._parking_stop_detail = lift_status.parking_stop_detail
                else:
                    self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

            return True
        except Exception as e:
            log_err(f"Lift data update failed: {e}")
            return False

    def has_new_data(self) -> bool:
        """
        Check if there is new data
        """
        with self._data_lock:
            return self._last_update_time is not None

    def convert_positions_to_rad(self, positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray:
        """
        Convert positions to radians

        Args:
            positions: Positions
            pulse_per_rotation: Pulse per rotation
        """
        return positions / pulse_per_rotation

    def convert_rad_to_positions(self, positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray:
        """
        Convert radians to positions
        
        Args:
            positions: Positions
            pulse_per_rotation: Pulse per rotation
        """
        return positions * pulse_per_rotation

    async def _periodic(self):
        """
        Periodic execution function
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time
        
        await self._init()
        log_info("Lift init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                # check lift error
                error = self.get_parking_stop_detail()
                if error != public_api_types_pb2.ParkingStopDetail():
                    if start_time - self.__last_warning_time > 1.0:
                        log_err(f"emergency stop: {error}")
                        self.__last_warning_time = start_time

                # prepare sending message
                with self._status_lock:
                    c = self._calibrated
                    sc = self._send_calibrate
                    self._send_calibrate = None
                    b = self._send_brake
                    self._send_brake = None
                    tp = self._target_pos
                    self._target_pos = None
                    ts = self._target_speed
                    self._target_speed = None

                ## send calibrate message
                if sc:
                    msg = self._construct_calibrate_message()
                    await self._send_message(msg)

                ## send brake message
                if b:
                    msg = self._construct_brake_msg(b)
                    await self._send_message(msg)

                ## send move command
                if c == True:
                    if ts is not None:
                        msg = self._construct_set_speed_msg(ts)
                        await self._send_message(msg)
                        await asyncio.sleep(0.5)
                    if tp is not None:
                        msg = self._construct_target_pos_msg(tp)
                        await self._send_message(msg)
                
            except Exception as e:
                log_err(f"Lift periodic task exception: {e}")
                continue
    
    def get_parking_stop_detail(self) -> public_api_types_pb2.ParkingStopDetail:
        """
        Get parking stop detail
        """
        if self._parking_stop_detail == public_api_types_pb2.ParkingStopDetail():
            return None
        return copy.deepcopy(self._parking_stop_detail)
    
    def get_state(self) -> str:
        """Get lift state"""
        lift_state_descriptor = public_api_types_pb2.LiftState.DESCRIPTOR
        with self._status_lock:
            return lift_state_descriptor.values_by_number[self._state].name

    def get_pos_range(self) -> Tuple[int, int]:
        """
        Get position range, in meters
        """
        with self._status_lock:
            if self._max_pos is None:
                return (0, 0)
            return (0, float(np.abs(self._max_pos)/self._pulse_per_rotation))

    def get_motor_positions(self) -> List[float]:
        """Get all motor positions (rad)"""
        with self._data_lock:
            self._last_update_time = None
            return np.abs(self._current_pos / self._pulse_per_rotation)

    def get_move_speed(self) -> float:
        """Get motor velocity (pulse/s)"""
        with self._data_lock:
            return self._move_speed
        
    def get_max_move_speed(self) -> float:
        """Get max move speed (pulse/s)"""
        with self._data_lock:
            return self._max_speed
        
    def get_pulse_per_meter(self) -> float:
        """Get pulse per meter"""
        with self._data_lock:
            return self._pulse_per_rotation

    # support command type: POSITION, BRAKE
    # position unit: rad
    def motor_command(self, command_type: CommandType, values: Union[bool, float, np.ndarray]):
        """
        Set motor command
        """
        if isinstance(values, float):
            values = np.array([values] * self.motor_count)
        if command_type == CommandType.POSITION:
            with self._status_lock:
                values = self.convert_rad_to_positions(values, self._pulse_per_rotation)
                if values < 0:
                    raise ValueError("Position command value cannot be negative")
                np.clip(values, 0, np.abs(self._max_pos), out=values)
                self._target_pos = values
        elif command_type == CommandType.BRAKE:
            with self._status_lock:
                self._send_brake = True
        else:
            raise ValueError(f"Lift only supports POSITION command type, got {command_type}")

    def set_move_speed(self, speed: Union[float, np.ndarray]):
        """
        Set move speed
        """
        tar_speed = np.clip(speed, 0, self._max_speed)
        with self._status_lock:
            self._target_speed = tar_speed

    def calibrate(self):
        """
        Calibrate
        The lift must be calibrated before moving when powered on
        It is strictly forbidden to send the calibrate command continuously!!!
        """
        with self._status_lock:
            self._send_calibrate = True

    # message constructor
    def _construct_calibrate_message(self) -> public_api_down_pb2.APIDown:
        """
        Construct calibrate message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.LinearLiftCommand()
        lift_command.calibrate = True
        msg.linear_lift_command.CopyFrom(lift_command)
        return msg

    def _construct_target_pos_msg(self, target_pos: np.ndarray) -> public_api_down_pb2.APIDown:
        """
        Construct target motor message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.LinearLiftCommand()
        with self._status_lock:
            target_pos = np.sign(self._max_pos) * target_pos
        lift_command.target_pos = int(target_pos[0])
        msg.linear_lift_command.CopyFrom(lift_command)
        return msg

    def _construct_brake_msg(self, brake: bool) -> public_api_down_pb2.APIDown:
        """
        Construct brake message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.LinearLiftCommand()
        lift_command.brake = brake
        msg.linear_lift_command.CopyFrom(lift_command)
        return msg

    def _construct_set_speed_msg(self, speed: int) -> public_api_down_pb2.APIDown:
        """
        Construct set speed message
        """
        msg = public_api_down_pb2.APIDown()
        lift_command = public_api_types_pb2.LinearLiftCommand()
        lift_command.set_speed = speed
        msg.linear_lift_command.CopyFrom(lift_command)
        return msg

    def __str__(self) -> str:
        """String representation"""
        return f"{self.name}(Count:{self.motor_count})"

    def __repr__(self) -> str:
        """Detailed string representation"""
        return f"Lift(motor_count={self.motor_count}, name='{self.name}')"

    def __len__(self) -> int:
        """Return motor count"""
        return self.motor_count