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
import copy

ROBOT_TYPE_TRIPLE_OMNI_WHEEL_LR_DRIVER = 1
ROBOT_TYPE_PCW_VEHICLE = 2
ROBOT_TYPE_CUSTOM_PCW_VEHICLE = 4
ROBOT_TYPE_LR_MARK2 = 5

class Chassis(DeviceBase, MotorBase):
    """
    Chassis class
    
    Inherits from DeviceBase and MotorBase, mainly implements mapping to BaseStatus
    This class corresponds to BaseStatus in proto, managing chassis status and motor control
    
    """

    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtTriggerA3Lr1,
        public_api_types_pb2.RobotType.RtMaverX4D,
        public_api_types_pb2.RobotType.RtMaverL4D,
        public_api_types_pb2.RobotType.RtArk2Lr1,
    ]

    def __init__(self,
                 motor_count: int,
                 robot_type: int,
                 name: str = "Chassis",
                 control_hz: int = 500,
                 send_message_callback=None,
                 ):
        """
        Initialize Chassis
        
        Args:
            motor_count: Number of motors
            name: Device name
            control_hz: Control frequency
            send_message_callback: Callback function for sending messages, used to send downstream messages
        """
        DeviceBase.__init__(self, name, send_message_callback)
        MotorBase.__init__(self, motor_count, name)
        self.name = name or "Chassis"
        self._set_robot_type(robot_type)
        self._control_hz = control_hz
        self._target_zero_resistance = False
        self._target_velocity = (0.0, 0.0, 0.0)  # Initialize target velocity

        # Chassis status
        self._status_lock = self._status_lock
        self._base_state = BaseState.BsParked
        self._api_control_initialized = False
        self._simple_control_mode = None
        self._session_holder = 0
        self._previous_session_holder = None

        # Battery information
        self._battery_voltage = 0.0
        self._battery_thousandth = 0

        # Optional fields
        self._battery_charging = None
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        self._warning = None

        # Odometry information
        self.__vehicle_origin_position = np.eye(3)  # Used to clear odometry bias
        self._vehicle_speed = deque(maxlen=10)  # (x, y, z) m/s, m/s, rad/s - FIFO queue
        self._vehicle_position = deque(maxlen=10)  # (x, y, yaw) m, m, rad - FIFO queue

        # Control related
        self._send_init = self._send_init
        self._send_clear_parking_stop: Optional[bool] = None

        self._last_command_time = None
        self._command_timeout = 0.1  # 100ms timeout
        self.__last_warning_time = time.perf_counter()  # Add warning time attribute
        self._my_session_id = 0   # my session id, was assigned by server
        self._is_timeout = False

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
        Initialize chassis
        
        Returns:
            bool: Whether initialization was successful
        """
        try:
            return True
        except Exception as e:
            log_err(f"Chassis initialization failed: {e}")
            return False

    def _update(self, api_up_data, timestamp: Timestamp) -> bool:
        """
        Update chassis data
        
        Args:
            api_up_data: Upstream data received from API (APIUp)
            
        Returns:
            bool: Whether update was successful
        """
        try:
            # Check if it contains BaseStatus
            if not api_up_data.HasField('base_status'):
                return False

            base_status = api_up_data.base_status
            # Update motor data
            self._push_motor_data(base_status.motor_status, timestamp)

            with self._status_lock:
                # update my session id
                self._my_session_id = api_up_data.session_id
                # Update chassis status
                self._base_state = base_status.state
                self._api_control_initialized = base_status.api_control_initialized
                self._battery_voltage = base_status.battery_voltage
                self._battery_thousandth = base_status.battery_thousandth
                self._session_holder = base_status.session_holder

                if self._session_holder != self._previous_session_holder:
                    if self._session_holder == self._my_session_id:
                        log_info(f"Chassis: You can control the chassis now! Your session ID: {self._session_holder}")
                    else:
                        log_warn(f"Chassis: Can not control the chassis, now holder is ID: {self._session_holder}, waiting...")
                self._previous_session_holder = self._session_holder

                # Update optional fields
                if base_status.HasField('battery_charging'):
                    self._battery_charging = base_status.battery_charging
                if base_status.HasField('parking_stop_detail'):
                    self._parking_stop_detail = base_status.parking_stop_detail
                else:
                    self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
                if base_status.HasField('warning'):
                    self._warning = base_status.warning
                if base_status.HasField('estimated_odometry'):
                    self._vehicle_speed.append((base_status.estimated_odometry.speed_x,
                                                base_status.estimated_odometry.speed_y,
                                                base_status.estimated_odometry.speed_z))
                    self._vehicle_position.append((base_status.estimated_odometry.pos_x,
                                                  base_status.estimated_odometry.pos_y,
                                                  base_status.estimated_odometry.pos_z))

            
            return True
        except Exception as e:
            log_err(f"Chassis data update failed: {e}")
            return False

    async def _periodic(self):
        """
        Periodic execution function
        
        Execute periodic tasks for the chassis, including:
        - Status check
        - Command timeout check
        - Safety monitoring
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("Chassis init success")
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
                    s = self._send_init
                    a = self._api_control_initialized
                    sh = self._session_holder
                    mi = self._my_session_id
                    sps = self._send_clear_parking_stop
                    self._send_clear_parking_stop = None

                # check if send clear parking stop message
                if sps is not None:
                    if sps:
                        msg = self._construct_clear_parking_stop_message()
                        await self._send_message(msg)

                # Check if send init message
                if s is None:
                    pass
                elif s:
                    msg = self._construct_init_message(True)
                    await self._send_message(msg)
                    self._clear_send_init()
                elif not s:
                    msg = self._construct_init_message(False)
                    await self._send_message(msg)
                    self._simple_control_mode = None
                    self._clear_send_init()

                # check if is holder:
                if sh != mi:
                    if start_time - self.__last_warning_time > 3.0:
                        log_warn(f"Chassis: You are not the session holder, please use start() method to get the control of the chassis...")
                        self.__last_warning_time = start_time
                    continue

                if a == False:
                    # if not simple control mode and target zero resistance, it means the vehicle is in zero resistance state
                    if self._simple_control_mode == False and self._target_zero_resistance == True:
                        pass
                    else:
                        if start_time - self.__last_warning_time > 1.0:
                            log_warn(
                                f"Chassis is not started."
                            )
                            self.__last_warning_time = start_time
                else:
                # send control message
                    if self._simple_control_mode == True:
                        if self._target_zero_resistance:
                            msg = self._construct_zero_resistance_message(
                                True, True)
                            await self._send_message(msg)
                        else:
                            msg = self._construct_zero_resistance_message(
                                False, True)
                            await self._send_message(msg)

                            if start_time - self._last_command_time > self._command_timeout:
                                self._is_timeout = True
                                msg = self._construct_simple_control_message(
                                    (0.0, 0.0, 0.0))
                            else:
                                self._is_timeout = False
                                msg = self._construct_simple_control_message(
                                    self._target_velocity)
                            await self._send_message(msg)

                    elif self._simple_control_mode == False:
                        if self._target_zero_resistance:
                            msg = self._construct_zero_resistance_message(
                                True, False)
                            await self._send_message(msg)
                        else:
                            msg = self._construct_zero_resistance_message(
                                False, False)
                            await self._send_message(msg)

                            if start_time - self._last_command_time > self._command_timeout:
                                self._is_timeout = True
                                self.motor_command(
                                    CommandType.BRAKE,
                                    [0.0 * self.motor_count])
                                msg = self._construct_wheel_control_message()
                            else:
                                self._is_timeout = False
                                msg = self._construct_wheel_control_message()
                            await self._send_message(msg)

            except Exception as e:
                log_err(f"Chassis periodic failed: {e}")

    def clear_odom_bias(self):
        """ reset odometry position """
        with self._status_lock:
            if len(self._vehicle_position) == 0:
                raise ValueError("Cannot clear odom bias: vehicle position data not available (queue is empty)")
            # Get latest position without popping
            x, y, yaw = self._vehicle_position[-1]
            log_common(f"clear odom bias: {x}, {y}, {yaw}")
            # Convert (x, y, yaw) to 2D transformation matrix
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            self.__vehicle_origin_position = np.array([[cos_yaw, -sin_yaw, x],
                                                       [sin_yaw, cos_yaw, y],
                                                       [0.0, 0.0, 1.0]])

    # Chassis-specific methods
    def get_base_state(self) -> str:
        """Get chassis status"""
        base_state_descriptor = public_api_types_pb2.BaseState.DESCRIPTOR
        with self._status_lock:
            return base_state_descriptor.values_by_number[self._base_state].name

    def is_api_control_initialized(self) -> bool:
        """Check if API control is initialized"""
        return self._api_control_initialized

    def get_battery_info(self) -> Dict[str, Any]:
        """Get battery information"""
        return {
            'voltage': self._battery_voltage,
            'thousandth': self._battery_thousandth,
            'charging': self._battery_charging
        }

    def get_vehicle_speed(self, pop: bool = True) -> Optional[Tuple[float, float, float]]:
        """Get vehicle speed (m/s, m/s, rad/s)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Tuple of (speed_x, speed_y, speed_z) or None if queue is empty
        """
        with self._status_lock:
            if len(self._vehicle_speed) > 0:
                if pop:
                    return self._vehicle_speed.popleft()
                else:
                    return self._vehicle_speed[-1]
            return None

    def get_vehicle_position(self, pop: bool = True) -> Optional[Tuple[float, float, float]]:
        """ get vehicle position
        Odometry position, unit: m
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Tuple of (relative_x, relative_y, relative_yaw) or None if queue is empty
        """
        with self._status_lock:
            if len(self._vehicle_position) == 0:
                return None
            
            self.__has_new = False

            # Get position from queue
            if pop:
                x, y, yaw = self._vehicle_position.popleft()
                # get once in order to sync deque length.
                self._get_motor_motion_data()
            else:
                x, y, yaw = self._vehicle_position[-1]
            
            # Convert current position to transformation matrix
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            current_matrix = np.array([[cos_yaw, -sin_yaw, x],
                                       [sin_yaw, cos_yaw, y], [0.0, 0.0, 1.0]])

            # Calculate relative transformation: current * inverse(origin)
            origin_inv = np.linalg.inv(self.__vehicle_origin_position)
            relative_matrix = origin_inv @ current_matrix

            # Extract position and orientation from relative matrix
            relative_x = float(relative_matrix[0, 2])
            relative_y = float(relative_matrix[1, 2])
            relative_yaw = float(np.arctan2(relative_matrix[1, 0],
                                            relative_matrix[0, 0]))

            return (relative_x, relative_y, relative_yaw)

    def get_parking_stop_detail(self):
        """Get parking stop details"""
        return self._parking_stop_detail

    def get_warning(self) -> Optional[int]:
        """Get warning information"""
        return self._warning

    def get_session_holder(self) -> int:
        """Get session holder"""
        with self._status_lock:
            return self._session_holder

    def get_my_session_id(self) -> int:
        """Get my session id"""
        with self._status_lock:
            return self._my_session_id

    def clear_parking_stop(self):
        """
        Clear parking stop
        Note: If you have clear parking stop, you must call start() again.
        """
        with self._status_lock:
            self._send_clear_parking_stop = True

    def enable(self):
        '''
        enable chassis
        '''
        with self._command_lock:
            self._target_zero_resistance = False

    def disable(self):
        '''
        set zero resistance
        '''
        with self._command_lock:
            self._target_zero_resistance = True

    def motor_command(self, command_type: CommandType, values: List[float]):
        """
        Set chassis command
        
        Args:
            command_type: Command type
            values: List of command values
        """
        if self._simple_control_mode == True:
            raise NotImplementedError(
                "motor_command not implemented for _simple_control_mode: True")
        elif self._simple_control_mode == None:
            self._simple_control_mode = False

        super().motor_command(command_type, values)
        self._last_command_time = time.perf_counter()

    def set_vehicle_speed(self, speed_x: float, speed_y: float,
                          speed_z: float):
        """
        Set XYZ velocity
        
        Args:
            speed_x: X-direction velocity (m/s)
            speed_y: Y-direction velocity (m/s)
            speed_z: Z-direction angular velocity (rad/s)
        """
        if self._simple_control_mode == False:
            raise NotImplementedError(
                "set_vehicle_speed not implemented for _simple_control_mode: False"
            )
        elif self._simple_control_mode == None:
            self._simple_control_mode = True

        # filter speed_y for Mark2
        if self.robot_type == ROBOT_TYPE_LR_MARK2:
            speed_y = 0.0

        with self._command_lock:
            self._target_velocity = (speed_x, speed_y, speed_z)
            self._last_command_time = time.perf_counter()

    def is_timeout(self) -> bool:
        """
        Check if the command is timeout
        """
        return copy.copy(self._is_timeout)

    # msg constructer
    # construct control message
    def _construct_wheel_control_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a control message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        
        pulse_per_rotation_arr = self.get_motor_pulse_per_rotations()
        if pulse_per_rotation_arr is not None:
            motor_targets = self._construct_target_motor_msg(pulse_per_rotation_arr)
            base_command.motor_targets.CopyFrom(motor_targets)
            msg.base_command.CopyFrom(base_command)
            return msg
        else:
            raise ValueError(f"Cannot construct wheel control message: pulse_per_rotation data not available (not set yet)")

    def _construct_simple_control_message(
            self, data: Tuple[float, float,
                              float]) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a simple control message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        simple_base_move_command = public_api_types_pb2.SimpleBaseMoveCommand()
        xyz_speed = public_api_types_pb2.XyzSpeed()
        xyz_speed.speed_x = data[0]
        xyz_speed.speed_y = data[1]
        xyz_speed.speed_z = data[2]
        simple_base_move_command.xyz_speed.CopyFrom(xyz_speed)
        base_command.simple_move_command.CopyFrom(simple_base_move_command)
        msg.base_command.CopyFrom(base_command)
        return msg

    def _construct_zero_resistance_message(
            self, data: bool,
            is_simple_control_mode: bool) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a zero resistance message.
        """
        msg = None
        if is_simple_control_mode:
            msg = public_api_down_pb2.APIDown()
            base_command = public_api_types_pb2.BaseCommand()
            simple_base_move_command = public_api_types_pb2.SimpleBaseMoveCommand(
            )
            simple_base_move_command.zero_resistance = data
            base_command.simple_move_command.CopyFrom(simple_base_move_command)
            msg.base_command.CopyFrom(base_command)
        else:
            msg = public_api_down_pb2.APIDown()
            base_command = public_api_types_pb2.BaseCommand()
            base_command.api_control_initialize = not data
            msg.base_command.CopyFrom(base_command)
        return msg

    def _construct_init_message(self, api_control_initialize: bool) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a init message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        base_command.api_control_initialize = api_control_initialize
        msg.base_command.CopyFrom(base_command)
        return msg

    def _construct_clear_parking_stop_message(self):
        """
        @brief: For constructing a clear_parking_stop message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        base_command.clear_parking_stop = True
        msg.base_command.CopyFrom(base_command)
        return msg

    def _construct_set_parking_stop_message(
            self, reason: str, category: int,
            is_remotely_clearable: bool) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a set_parking_stop message.
        @params:
            reason: what caused the parking stop
            category: parking stop category, values can be :
                0: EmergencyStopButton,
                1: MotorHasError,
                2: BatteryFail
                3: GamepadTriggered,
                4: UnknownParkingStopCategory,
                5: APICommunicationTimeout
            is_remotely_clearable: whether the parking stop can be cleared remotely
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        parking_stop_category = category

        parking_stop_detail.reason = reason
        parking_stop_detail.category = parking_stop_category
        parking_stop_detail.is_remotely_clearable = is_remotely_clearable

        base_command.trigger_parking_stop.CopyFrom(parking_stop_detail)
        msg.base_command.CopyFrom(base_command)
        return msg

    def get_status_summary(self) -> Dict[str, Any]:
        """Get chassis status summary"""
        summary = super().get_device_summary()

        # Add chassis-specific information
        chassis_summary = {
            'base_state':
            public_api_types_pb2.BaseState.Name(self._base_state),
            'api_control_initialized':
            self._api_control_initialized,
            'battery_info':
            self.get_battery_info(),
            'vehicle_speed':
            self._vehicle_speed[-1] if len(self._vehicle_speed) > 0 else None,
            'vehicle_position':
            self._vehicle_position[-1] if len(self._vehicle_position) > 0 else None,
            'parking_stop_detail':
            self._parking_stop_detail,
            'warning':
            public_api_types_pb2.WarningCategory.Name(self._warning)
            if self._warning else None,
        }

        summary.update(chassis_summary)
        return summary

    def __str__(self) -> str:
        """String representation"""
        state_name = public_api_types_pb2.BaseState.Name(self._base_state)
        return f"{self.name}(State:{state_name}, Motors:{self.motor_count}, API:{self._api_control_initialized})"

    def __repr__(self) -> str:
        """Detailed string representation"""
        state_name = public_api_types_pb2.BaseState.Name(self._base_state)
        return f"Chassis(motor_count={self.motor_count}, name='{self.name}', base_state={state_name})"
