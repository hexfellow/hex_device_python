#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any, Union, Callable
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from enum import Enum
import threading
import time
import numpy as np
from copy import deepcopy
from collections import deque

class CommandType(Enum):
    """Command type enumeration"""
    BRAKE = "brake"
    SPEED = "speed"
    POSITION = "position"
    TORQUE = "torque"
    MIT = "mit"


@dataclass
class MitMotorCommand:
    """MIT motor command structure
    
    Contains all parameters required for MIT motor control:
    - torque: Torque (Nm)
    - speed: Speed (rad/s) 
    - position: Position (rad)
    - kp: Proportional gain
    - kd: Derivative gain
    """
    torque: float
    speed: float
    position: float
    kp: float
    kd: float

@dataclass
class MotorCommand:
    """Motor command structure

    Can choose from five types of commands:
    1. brake command - bool type
    2. speed command - float array type
    3. position command - float array type
    4. torque command - float array type
    5. MIT command - MitMotorCommand list type
    """
    command_type: CommandType
    brake_command: Optional[List[bool]] = None
    speed_command: Optional[List[float]] = None
    position_command: Optional[List[float]] = None
    torque_command: Optional[List[float]] = None
    mit_command: Optional[List[MitMotorCommand]] = None

    def __post_init__(self):
        """Validate command data validity"""
        if self.command_type == CommandType.BRAKE:
            if self.brake_command is None:
                raise ValueError("brake command type requires brake_command parameter")
            if self.speed_command is not None or self.position_command is not None or self.torque_command is not None or self.mit_command is not None:
                raise ValueError(
                    "brake command type should not contain speed_command, position_command, torque_command or mit_command"
                )

        elif self.command_type == CommandType.SPEED:
            if self.speed_command is None:
                raise ValueError("speed command type requires speed_command parameter")
            if self.brake_command is not None or self.position_command is not None or self.torque_command is not None or self.mit_command is not None:
                raise ValueError(
                    "speed command type should not contain brake_command, position_command, torque_command or mit_command"
                )
            if not isinstance(self.speed_command, list) or not all(
                    isinstance(x, (int, float)) for x in self.speed_command):
                raise ValueError("speed_command must be a float array")

        elif self.command_type == CommandType.POSITION:
            if self.position_command is None:
                raise ValueError("position command type requires position_command parameter")
            if self.brake_command is not None or self.speed_command is not None or self.torque_command is not None or self.mit_command is not None:
                raise ValueError(
                    "position command type should not contain brake_command, speed_command, torque_command or mit_command"
                )
            if not isinstance(self.position_command, list) or not all(
                    isinstance(x, (int, float))
                    for x in self.position_command):
                raise ValueError("position_command must be a float array")

        elif self.command_type == CommandType.TORQUE:
            if self.torque_command is None:
                raise ValueError("torque command type requires torque_command parameter")
            if self.brake_command is not None or self.speed_command is not None or self.position_command is not None or self.mit_command is not None:
                raise ValueError(
                    "torque command type should not contain brake_command, speed_command, position_command or mit_command"
                )
            if not isinstance(self.torque_command, list) or not all(
                    isinstance(x, (int, float)) for x in self.torque_command):
                raise ValueError("torque_command must be a float array")

        elif self.command_type == CommandType.MIT:
            if self.mit_command is None:
                raise ValueError("mit command type requires mit_command parameter")
            if self.brake_command is not None or self.speed_command is not None or self.position_command is not None or self.torque_command is not None:
                raise ValueError(
                    "mit command type should not contain brake_command, speed_command, position_command or torque_command"
                )
            if not isinstance(self.mit_command, list) or not all(
                    isinstance(x, MitMotorCommand) for x in self.mit_command):
                raise ValueError("mit_command must be a list of MitMotorCommand objects")

    @classmethod
    def create_brake_command(cls, brake: List[bool]) -> 'MotorCommand':
        """Create brake command
        Args:
            brake: Brake command, whether True or False, both indicate braking
        """
        return cls(command_type=CommandType.BRAKE, brake_command=deepcopy(brake))

    @classmethod
    def create_speed_command(cls, speeds: List[float]) -> 'MotorCommand':
        """Create speed command
        Args:
            speeds: Speed value list (rad/s)
        """
        return cls(command_type=CommandType.SPEED, speed_command=deepcopy(speeds))

    @classmethod
    def create_position_command(
            cls,
            positions: List[float]) -> 'MotorCommand':
        """
        Create position command
        Args:
            positions: Position value list (rad)
            pulse_per_rotation: Pulses per rotation
        """
        return cls(command_type=CommandType.POSITION,
                    position_command=deepcopy(positions))

    @classmethod
    def create_torque_command(cls, torques: List[float]) -> 'MotorCommand':
        """Create torque command"""
        return cls(command_type=CommandType.TORQUE, torque_command=deepcopy(torques))

    @classmethod
    def create_mit_command(cls, mit_commands: List[MitMotorCommand]) -> 'MotorCommand':
        """Create MIT command"""
        return cls(command_type=CommandType.MIT, mit_command=deepcopy(mit_commands))

class MotorError(Enum):
    """Motor error enumeration, used to implement mapping from MotorError in proto to python class"""
    ME_COMMUNICATION_ERROR = 0
    ME_OVER_CURRENT = 1
    ME_OVER_VOLTAGE = 2
    ME_UNDER_VOLTAGE = 3
    ME_MOTOR_OVER_TEMPERATURE = 4
    ME_DRIVER_OVER_TEMPERATURE = 5
    ME_GENERAL_ERROR = 6


class MotorBase(ABC):
    """
    Motor base class
    Manages multiple motors in array form, defines basic interfaces and common functionality for motors
    This class corresponds to MotorStatus in proto
    """

    def __init__(self, motor_count: int, name: str = "",
                 convert_positions_to_rad_func: Optional[Callable[[np.ndarray, np.ndarray], np.ndarray]] = None,
                 convert_rad_to_positions_func: Optional[Callable[[np.ndarray, np.ndarray], np.ndarray]] = None):
        """
        Initialize motor base class
        Args:
            motor_count: Number of motors
            name: Motor group name
            convert_positions_to_rad_func: Optional custom function to convert encoder positions to radians.
                If None, uses default implementation.
                Function signature: (positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray
            convert_rad_to_positions_func: Optional custom function to convert radians to encoder positions.
                If None, uses default implementation.
                Function signature: (positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray
        """
        self.motor_count = motor_count
        self.name = name or f"MotorGroup"

        # Initialize queues for each motor (FIFO, max length 10)
        self._states = [deque(maxlen=10) for _ in range(motor_count)]  # "normal", "error"
        self._error_codes = [deque(maxlen=10) for _ in range(motor_count)]  # Use None to indicate no error

        # Motor motion data
        self._positions = [deque(maxlen=10) for _ in range(motor_count)]  # Position (rad)
        self._velocities = [deque(maxlen=10) for _ in range(motor_count)]  # Velocity (rad/s)
        self._torques = [deque(maxlen=10) for _ in range(motor_count)]  # Torque (Nm)
        self._encoder_positions = [deque(maxlen=10) for _ in range(motor_count)]  # Encoder position
        self._pulse_per_rotation: Optional[np.ndarray] = None  # Pulses per rotation (set once, not updated)
        self._wheel_radius: Optional[np.ndarray] = None  # Wheel radius (set once, not updated)
        # cache data, use to control the motor command
        self.__cache_positions = None
        self.__cache_velocities = None
        self.__cache_torques = None

        # Motor status parameters (optional)
        self._driver_temperature = [deque(maxlen=10) for _ in range(motor_count)]  # Driver temperature (°C)
        self._motor_temperature = [deque(maxlen=10) for _ in range(motor_count)]  # Motor temperature (°C)
        self._voltage = [deque(maxlen=10) for _ in range(motor_count)]  # Voltage (V)

        # Target commands
        self._current_targets = [deque(maxlen=10) for _ in range(motor_count)]  # Commands currently running on the device
        self._target_command = None  # The raw command, not converted to the scale in proto comments

        # Timestamp
        self._last_update_time = deque(maxlen=10)

        # Thread locks
        self._data_lock = threading.Lock()
        self._command_lock = threading.Lock()

        # Store custom conversion functions if provided
        self._custom_convert_positions_to_rad = convert_positions_to_rad_func
        self._custom_convert_rad_to_positions = convert_rad_to_positions_func

    @property
    def cache_motion_data(self) -> np.ndarray:
        """Get all motor cache motion data"""
        with self._data_lock:
            return self.__cache_positions, self.__cache_velocities, self.__cache_torques

    @property
    def target_positions(self) -> np.ndarray:
        """Get all motor target positions (rad)"""
        with self._command_lock:
            if self._target_command and self._target_command.command_type == CommandType.POSITION:
                return np.array(self._target_command.position_command)
            return np.zeros(self.motor_count)

    @property
    def target_velocities(self) -> np.ndarray:
        """Get all motor target velocities (rad/s)"""
        with self._command_lock:
            if self._target_command and self._target_command.command_type == CommandType.SPEED:
                return np.array(self._target_command.speed_command)
            return np.zeros(self.motor_count)

    @property
    def target_torques(self) -> np.ndarray:
        """Get all motor target torques (Nm)"""
        with self._command_lock:
            if self._target_command and self._target_command.command_type == CommandType.TORQUE:
                return np.array(self._target_command.torque_command)
            return np.zeros(self.motor_count)

    def get_motor_error_codes(self, pop: bool = True) -> Optional[List[Optional[int]]]:
        """Get all motor error codes
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of error codes or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._error_codes[i]) > 0:
                    if pop:
                        result.append(self._error_codes[i].popleft())
                    else:
                        result.append(self._error_codes[i][-1])
                else:
                    return None
            return result

    def get_motor_states(self, pop: bool = True) -> Optional[List[str]]:
        """Get all motor states
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of motor states or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._states[i]) > 0:
                    if pop:
                        result.append(self._states[i].popleft())
                    else:
                        result.append(self._states[i][-1])
                else:
                    return None
            return result

    def get_motor_state(self, motor_index: int, pop: bool = True) -> Optional[str]:
        """Get specified motor state
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor state or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._states[motor_index]) > 0:
                if pop:
                    return self._states[motor_index].popleft()
                else:
                    return self._states[motor_index][-1]
            return None

    def get_motor_encoder_positions(self, pop: bool = True) -> Optional[np.ndarray]:
        """Get all motor encoder positions
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Array of encoder positions or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._encoder_positions[i]) > 0:
                    if pop:
                        result.append(self._encoder_positions[i].popleft())
                    else:
                        result.append(self._encoder_positions[i][-1])
                else:
                    return None
            return np.array(result, dtype=np.float64)

    def get_motor_position(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor position (rad)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor position or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._positions[motor_index]) > 0:
                if pop:
                    return self._positions[motor_index].popleft()
                else:
                    return self._positions[motor_index][-1]
            return None

    def get_motor_positions(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor positions (rad)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of positions or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._positions[i]) > 0:
                    if pop:
                        result.append(self._positions[i].popleft())
                    else:
                        result.append(self._positions[i][-1])
                else:
                    return None
            return result

    def get_encoders_to_zero(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor encoders to zero (rad)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of encoder positions to zero or None if queue is empty
        """
        with self._data_lock:
            encoder_positions = []
            for i in range(self.motor_count):
                if len(self._encoder_positions[i]) > 0:
                    if pop:
                        encoder_positions.append(self._encoder_positions[i].popleft())
                    else:
                        encoder_positions.append(self._encoder_positions[i][-1])
                else:
                    return None
            tar_arr = 32767 - np.array(encoder_positions, dtype=np.float64)
            return tar_arr.tolist()

    def get_motor_velocity(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor velocity (rad/s)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor velocity or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._velocities[motor_index]) > 0:
                if pop:
                    return self._velocities[motor_index].popleft()
                else:
                    return self._velocities[motor_index][-1]
            return None

    def get_motor_velocities(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor velocities (rad/s)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of velocities or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._velocities[i]) > 0:
                    if pop:
                        result.append(self._velocities[i].popleft())
                    else:
                        result.append(self._velocities[i][-1])
                else:
                    return None
            return result

    def get_motor_torque(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor torque (Nm)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor torque or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._torques[motor_index]) > 0:
                if pop:
                    return self._torques[motor_index].popleft()
                else:
                    return self._torques[motor_index][-1]
            return None

    def get_motor_torques(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor torques (Nm)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of torques or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._torques[i]) > 0:
                    if pop:
                        result.append(self._torques[i].popleft())
                    else:
                        result.append(self._torques[i][-1])
                else:
                    return None
            return result

    def get_motor_driver_temperatures(self, pop: bool = True) -> Optional[np.ndarray]:
        """Get all motor driver temperatures (°C)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Array of driver temperatures or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._driver_temperature[i]) > 0:
                    if pop:
                        result.append(self._driver_temperature[i].popleft())
                    else:
                        result.append(self._driver_temperature[i][-1])
                else:
                    return None
            return np.array(result, dtype=np.float64)

    def get_motor_driver_temperature(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor driver temperature (°C)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Driver temperature or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._driver_temperature[motor_index]) > 0:
                if pop:
                    return self._driver_temperature[motor_index].popleft()
                else:
                    return self._driver_temperature[motor_index][-1]
            return None
        
    def get_motor_temperatures(self, pop: bool = True) -> Optional[np.ndarray]:
        """Get all motor temperatures (°C)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Array of motor temperatures or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._motor_temperature[i]) > 0:
                    if pop:
                        result.append(self._motor_temperature[i].popleft())
                    else:
                        result.append(self._motor_temperature[i][-1])
                else:
                    return None
            return np.array(result, dtype=np.float64)

    def get_motor_temperature(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor temperature (°C)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor temperature or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._motor_temperature[motor_index]) > 0:
                if pop:
                    return self._motor_temperature[motor_index].popleft()
                else:
                    return self._motor_temperature[motor_index][-1]
            return None

    def get_motor_voltages(self, pop: bool = True) -> Optional[np.ndarray]:
        """Get all motor voltages (V)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Array of voltages or None if queue is empty
        """
        with self._data_lock:
            result = []
            for i in range(self.motor_count):
                if len(self._voltage[i]) > 0:
                    if pop:
                        result.append(self._voltage[i].popleft())
                    else:
                        result.append(self._voltage[i][-1])
                else:
                    return None
            return np.array(result, dtype=np.float64)

    def get_motor_voltage(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor voltage (V)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor voltage or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if len(self._voltage[motor_index]) > 0:
                if pop:
                    return self._voltage[motor_index].popleft()
                else:
                    return self._voltage[motor_index][-1]
            return None

    def get_motor_pulse_per_rotations(self) -> Optional[np.ndarray]:
        """Get all motor pulses per rotation
        
        Returns:
            Array of pulse per rotation values or None if not set
        """
        with self._data_lock:
            if self._pulse_per_rotation is None:
                return None
            return self._pulse_per_rotation.copy()

    def get_motor_pulse_per_rotation(self, motor_index: int) -> Optional[float]:
        """Get specified motor pulses per rotation
        
        Args:
            motor_index: Motor index
        
        Returns:
            Pulse per rotation or None if not set
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if self._pulse_per_rotation is None:
                return None
            return float(self._pulse_per_rotation[motor_index])

    def get_motor_wheel_radius(self) -> Optional[np.ndarray]:
        """Get all motor wheel radii (m)
        
        Returns:
            Array of wheel radii or None if not set
        """
        with self._data_lock:
            if self._wheel_radius is None:
                return None
            return self._wheel_radius.copy()

    def get_motor_wheel_radius(self, motor_index: int) -> Optional[float]:
        """Get specified motor wheel radius (m)
        
        Args:
            motor_index: Motor index
        
        Returns:
            Wheel radius or None if not set
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            if self._wheel_radius is None:
                return None
            return float(self._wheel_radius[motor_index])

    def motor_command(self, command_type: CommandType, values: Union[List[bool], List[float], List[MitMotorCommand], np.ndarray]):
        """
        Set motor command
        
        Args:
            command_type: Command type (BRAKE, SPEED, POSITION, TORQUE, MIT)
            values: Command value list
                - BRAKE: values parameter is only used to determine motor count (List[bool])
                - SPEED: Speed value list (rad/s) (List[float])
                - POSITION: Position value list (rad) (List[float])
                - TORQUE: Torque value list (Nm) (List[float])
                - MIT: MIT command list (List[MitMotorCommand])
        """
        # Convert numpy array to list if needed
        if isinstance(values, np.ndarray):
            values = values.tolist()

        if command_type == CommandType.BRAKE:
            if not isinstance(values, list) or not all(isinstance(x, bool) for x in values):
                raise ValueError("BRAKE command type requires boolean list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} brake values, got {len(values)}"
                )
            command = MotorCommand.create_brake_command(values)
        elif command_type == CommandType.SPEED:
            if not isinstance(values, list) or not all(isinstance(x, float) for x in values):
                raise ValueError("SPEED command type requires float list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} speed values, got {len(values)}"
                )
            command = MotorCommand.create_speed_command(values)
        elif command_type == CommandType.POSITION:
            if not isinstance(values, list) or not all(isinstance(x, float) for x in values):
                raise ValueError("POSITION command type requires float list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} position values, got {len(values)}"
                )
            command = MotorCommand.create_position_command(
                values)
        elif command_type == CommandType.TORQUE:
            if not isinstance(values, list) or not all(isinstance(x, float) for x in values):
                raise ValueError("TORQUE command type requires float list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} torque values, got {len(values)}"
                )
            command = MotorCommand.create_torque_command(values)
        elif command_type == CommandType.MIT:
            if not isinstance(values, list) or not all(isinstance(x, MitMotorCommand) for x in values):
                raise ValueError("MIT command type requires MitMotorCommand object list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} MIT commands, got {len(values)}"
                )
            command = MotorCommand.create_mit_command(values)
        else:
            raise ValueError(f"Unknown command type: {command_type}")

        with self._command_lock:
            self._target_command = command

    def mit_motor_command(self, mit_commands: List[MitMotorCommand]):
        """
        Set MIT motor command
        
        Args:
            mit_commands: MIT motor command list, each element contains torque, speed, position, kp, kd
        """
        if len(mit_commands) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} MIT commands, got {len(mit_commands)}"
            )
        
        command = MotorCommand.create_mit_command(mit_commands)
        
        with self._command_lock:
            self._target_command = command

    def convert_positions_to_rad(self, positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray:
        """
        Convert encoder positions to radians
        
        This method provides a default implementation but can be overridden by providing
        a custom function during instance initialization.
        
        Args:
            positions: Encoder positions array
            pulse_per_rotation: Pulses per rotation array
            
        Returns:
            Positions in radians
        """
        if self._custom_convert_positions_to_rad is not None:
            return self._custom_convert_positions_to_rad(positions, pulse_per_rotation)
        # Default implementation
        return (positions - 65535.0 / 2.0) / pulse_per_rotation * 2 * np.pi
    
    def convert_rad_to_positions(self, positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray:
        """
        Convert radians to encoder positions
        
        This method provides a default implementation but can be overridden by providing
        a custom function during instance initialization.
        
        Args:
            positions: Positions in radians array
            pulse_per_rotation: Pulses per rotation array
            
        Returns:
            Encoder positions array
        """
        if self._custom_convert_rad_to_positions is not None:
            return self._custom_convert_rad_to_positions(positions, pulse_per_rotation)
        # Default implementation
        return positions / (2 * np.pi) * pulse_per_rotation + 65535.0 / 2.0
    
    def update_motor_data(self,
                          positions: List[float],
                          velocities: List[float],
                          torques: List[float],
                          driver_temperature: List[float],
                          motor_temperature: List[float],
                          voltage: List[float],
                          pulse_per_rotation: Optional[List[float]] = None,
                          wheel_radius: Optional[List[float]] = None,
                          error_codes: Optional[List[Optional[int]]] = None,
                          current_targets: Optional[List[
                              public_api_types_pb2.SingleMotorTarget]] = None):
        """
        Update all motor data
        
        Args:
            positions: Position list (encoder position)
            velocities: Velocity list (rad/s)
            torques: Torque list (Nm)
            driver_temperature: Driver temperature list (°C)
            motor_temperature: Motor temperature list (°C)
            voltage: Voltage list (V)
            pulse_per_rotation: Pulses per rotation list, None means no update
            wheel_radius: Wheel radius list (m), None means no update
            error_codes: Error code list, None means no error
        """
        if len(positions) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} positions, got {len(positions)}")
        if len(velocities) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} velocities, got {len(velocities)}"
            )
        if len(torques) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} torques, got {len(torques)}")
        if len(driver_temperature) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} driver_temperatures, got {len(driver_temperature)}"
            )
        if len(motor_temperature) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} motor_temperatures, got {len(motor_temperature)}"
            )
        if len(voltage) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} voltages, got {len(voltage)}")

        if pulse_per_rotation is not None and len(
                pulse_per_rotation) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} pulse_per_rotation values, got {len(pulse_per_rotation)}"
            )

        if wheel_radius is not None and len(wheel_radius) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} wheel_radius values, got {len(wheel_radius)}"
            )

        if current_targets is not None and len(
                current_targets) != self.motor_count:
            raise ValueError(
                f"Expected {self.motor_count} current_targets, got {len(current_targets)}"
            )

        # Convert to numpy arrays outside the lock to minimize lock holding time
        # Use asarray instead of array to avoid copying if input is already an array
        velocities_arr = np.asarray(velocities, dtype=np.float64)
        torques_arr = np.asarray(torques, dtype=np.float64)
        driver_temperature_arr = np.asarray(driver_temperature, dtype=np.float64)
        motor_temperature_arr = np.asarray(motor_temperature, dtype=np.float64)
        voltage_arr = np.asarray(voltage, dtype=np.float64)
        positions_arr = np.asarray(positions, dtype=np.float64)
        
        # Handle optional parameters
        pulse_per_rotation_arr = np.asarray(pulse_per_rotation, dtype=np.float64) if pulse_per_rotation is not None else None
        wheel_radius_arr = np.asarray(wheel_radius, dtype=np.float64) if wheel_radius is not None else None

        # Prepare error code updates
        error_codes_copy = error_codes.copy() if error_codes is not None else None
        current_targets_copy = current_targets.copy() if current_targets is not None else None

        # Get pulse_per_rotation for position conversion
        # If provided, use it; otherwise read existing value quickly
        if pulse_per_rotation_arr is not None:
            pulse_for_conversion = pulse_per_rotation_arr
        else:
            with self._data_lock:
                if self._pulse_per_rotation is None:
                    raise ValueError(f"Cannot update motor data: pulse_per_rotation data not available (not set yet)")
                pulse_for_conversion = self._pulse_per_rotation
        
        # Convert encoder positions to radians
        positions_rad_arr = self.convert_positions_to_rad(
            positions_arr, pulse_for_conversion)

        # Now update all data within lock (add to queues)
        with self._data_lock:
            self.__cache_positions = positions_rad_arr.copy()  # ndarry copy is the same as deepcopy
            self.__cache_velocities = velocities_arr.copy()
            self.__cache_torques = torques_arr.copy()

            # Add data to queues (FIFO, max length 10)
            for i in range(self.motor_count):
                self._velocities[i].append(velocities_arr[i])
                self._torques[i].append(torques_arr[i])
                self._driver_temperature[i].append(driver_temperature_arr[i])
                self._motor_temperature[i].append(motor_temperature_arr[i])
                self._voltage[i].append(voltage_arr[i])
                self._encoder_positions[i].append(positions_arr[i])
                self._positions[i].append(positions_rad_arr[i])

            # Update pulse_per_rotation and wheel_radius only once (if not already set)
            if self._pulse_per_rotation is None and pulse_per_rotation_arr is not None:
                self._pulse_per_rotation = pulse_per_rotation_arr

            if self._wheel_radius is None and wheel_radius_arr is not None:
                self._wheel_radius = wheel_radius_arr

            # Update error codes and states
            if error_codes_copy is not None:
                for i, error_code in enumerate(error_codes_copy):
                    self._error_codes[i].append(error_code)
                    # Update state based on error codes
                    if error_code is not None:
                        self._states[i].append("error")
                    else:
                        # Check if previous state was error, if so change to normal
                        if len(self._states[i]) > 0 and self._states[i][-1] == "error":
                            self._states[i].append("normal")
                        elif len(self._states[i]) == 0:
                            self._states[i].append("normal")
            else:
                # If no error codes provided, maintain previous state or set to normal
                for i in range(self.motor_count):
                    if len(self._states[i]) > 0:
                        # Keep previous state
                        self._states[i].append(self._states[i][-1])
                    else:
                        # No previous state, set to normal
                        self._states[i].append("normal")

            if current_targets_copy is not None:
                for i in range(self.motor_count):
                    self._current_targets[i].append(current_targets_copy[i])

            self._last_update_time.append(time.perf_counter_ns())

    def get_motor_summary(self) -> Optional[Dict[str, Any]]:
        """Get status summary
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Dictionary with motor summary or None if queue is empty
        """
        with self._data_lock:
            # Get data from queues
            states = []
            error_codes = []
            positions = []
            velocities = []
            torques = []
            driver_temperature = []
            motor_temperature = []
            voltage = []
            pulse_per_rotation = []
            wheel_radius = []
            
            for i in range(self.motor_count):
                if (len(self._states[i]) > 0 and len(self._error_codes[i]) > 0 and 
                    len(self._positions[i]) > 0 and len(self._velocities[i]) > 0 and
                    len(self._torques[i]) > 0 and len(self._driver_temperature[i]) > 0 and
                    len(self._motor_temperature[i]) > 0 and len(self._voltage[i]) > 0):
                    states.append(self._states[i][-1])
                    error_codes.append(self._error_codes[i][-1])
                    positions.append(self._positions[i][-1])
                    velocities.append(self._velocities[i][-1])
                    torques.append(self._torques[i][-1])
                    driver_temperature.append(self._driver_temperature[i][-1])
                    motor_temperature.append(self._motor_temperature[i][-1])
                    voltage.append(self._voltage[i][-1])
                else:
                    return None
            
            # Get pulse_per_rotation and wheel_radius (not from queues)
            if self._pulse_per_rotation is not None:
                pulse_per_rotation = self._pulse_per_rotation.tolist()
            else:
                pulse_per_rotation = None
            
            if self._wheel_radius is not None:
                wheel_radius = self._wheel_radius.tolist()
            else:
                wheel_radius = None
            
            last_update_time = None
            if len(self._last_update_time) > 0:
                last_update_time = self._last_update_time[-1]
            
            summary = {
                'name': self.name,
                'motor_count': self.motor_count,
                'states': states,
                'error_codes': error_codes,
                'positions': positions,
                'velocities': velocities,
                'torques': torques,
                'driver_temperature': driver_temperature,
                'motor_temperature': motor_temperature,
                'voltage': voltage,
                'pulse_per_rotation': pulse_per_rotation,
                'wheel_radius': wheel_radius,
                'last_update_time': last_update_time,
            }

            # Add target command information
            if self._target_command:
                summary['target_command'] = {
                    'command_type': self._target_command.command_type.value,
                    'brake_command': deepcopy(self._target_command.brake_command),
                    'speed_command': deepcopy(self._target_command.speed_command),
                    'position_command': deepcopy(self._target_command.position_command),
                    'torque_command': deepcopy(self._target_command.torque_command)
                }
            else:
                summary['target_command'] = None

            return summary

    def get_motor_status(self, motor_index: int, pop: bool = True) -> Optional[Dict[str, Any]]:
        """Get specified motor state
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Dictionary with motor status or None if queue is empty
        """
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )

        with self._data_lock:
            # Check if all queues have data
            if (len(self._states[motor_index]) == 0 or len(self._error_codes[motor_index]) == 0 or
                len(self._positions[motor_index]) == 0 or len(self._velocities[motor_index]) == 0 or
                len(self._torques[motor_index]) == 0 or len(self._driver_temperature[motor_index]) == 0 or
                len(self._motor_temperature[motor_index]) == 0 or len(self._voltage[motor_index]) == 0):
                return None
            
            if pop:
                status = {
                    'index': motor_index,
                    'state': self._states[motor_index].popleft(),
                    'error_code': self._error_codes[motor_index].popleft(),
                    'position': self._positions[motor_index].popleft(),
                    'velocity': self._velocities[motor_index].popleft(),
                    'torque': self._torques[motor_index].popleft(),
                    'driver_temperature': self._driver_temperature[motor_index].popleft(),
                    'motor_temperature': self._motor_temperature[motor_index].popleft(),
                    'voltage': self._voltage[motor_index].popleft(),
                    'pulse_per_rotation': float(self._pulse_per_rotation[motor_index]) if self._pulse_per_rotation is not None else None,
                    'wheel_radius': float(self._wheel_radius[motor_index]) if self._wheel_radius is not None else None
                }
            else:
                status = {
                    'index': motor_index,
                    'state': self._states[motor_index][-1],
                    'error_code': self._error_codes[motor_index][-1],
                    'position': self._positions[motor_index][-1],
                    'velocity': self._velocities[motor_index][-1],
                    'torque': self._torques[motor_index][-1],
                    'driver_temperature': self._driver_temperature[motor_index][-1],
                    'motor_temperature': self._motor_temperature[motor_index][-1],
                    'voltage': self._voltage[motor_index][-1],
                    'pulse_per_rotation': float(self._pulse_per_rotation[motor_index]) if self._pulse_per_rotation is not None else None,
                    'wheel_radius': float(self._wheel_radius[motor_index]) if self._wheel_radius is not None else None
                }

            # Add target command information
            if self._target_command:
                if self._target_command.command_type == CommandType.BRAKE:
                    status['target_brake'] = self._target_command.brake_command
                elif self._target_command.command_type == CommandType.SPEED:
                    status[
                        'target_velocity'] = self._target_command.speed_command[
                            motor_index]
                elif self._target_command.command_type == CommandType.POSITION:
                    status[
                        'target_position'] = self._target_command.position_command[
                            motor_index]
                elif self._target_command.command_type == CommandType.TORQUE:
                    status[
                        'target_torque'] = self._target_command.torque_command[
                            motor_index]
            else:
                status['target_brake'] = None
                status['target_velocity'] = 0.0
                status['target_position'] = 0.0
                status['target_torque'] = 0.0

            return status
    
    def flush_data(self):
        """
        Clear all queues in MotorBase
        
        This method removes all data from all queues including:
        - Motor states, error codes, positions, velocities, torques
        - Temperature and voltage data
        - Encoder positions, pulse per rotation, wheel radius
        - Current targets and last update time
        """
        with self._data_lock:
            # Clear all motor-specific queues
            for i in range(self.motor_count):
                self._states[i].clear()
                self._error_codes[i].clear()
                self._torques[i].clear()
                self._velocities[i].clear()
                self._positions[i].clear()
                self._encoder_positions[i].clear()
                self._driver_temperature[i].clear()
                self._motor_temperature[i].clear()
                self._voltage[i].clear()
                self._current_targets[i].clear()
            
            # Clear timestamp queue
            self._last_update_time.clear()
            
            # Note: _pulse_per_rotation and _wheel_radius are not cleared as they are set once


    def get_simple_motor_status(self, pop: bool = True) -> Optional[Dict[str, Any]]:
        """Get simple motor status
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Dictionary with simple motor status or None if queue is empty
        """
        with self._data_lock:
            positions = []
            velocities = []
            torques = []
            
            for i in range(self.motor_count):
                if (len(self._positions[i]) > 0 and len(self._velocities[i]) > 0 and
                    len(self._torques[i]) > 0):
                    if pop:
                        positions.append(self._positions[i].popleft())
                        velocities.append(self._velocities[i].popleft())
                        torques.append(self._torques[i].popleft())
                    else:
                        positions.append(self._positions[i][-1])
                        velocities.append(self._velocities[i][-1])
                        torques.append(self._torques[i][-1])
                else:
                    return None
            
            last_update_time = None
            if len(self._last_update_time) > 0:
                if pop:
                    last_update_time = self._last_update_time.popleft()
                else:
                    last_update_time = self._last_update_time[-1]
            
            if last_update_time is None:
                return None
            
            return {
                'pos': positions,
                'vel': velocities,
                'eff': torques,
                'ts': {
                        "s": last_update_time // 1_000_000_000,
                        "ns": last_update_time % 1_000_000_000,
                    }
            }

    def _construct_target_motor_msg(
            self,
            pulse_per_rotation,
            command: MotorCommand = None) -> public_api_types_pb2.MotorTargets:
        """Construct downstream message"""
        if command is None:
            with self._command_lock:
                if self._target_command is None:
                    raise ValueError(
                        "Construct down msg failed, No target command")
                command = self._target_command

        motor_targets = public_api_types_pb2.MotorTargets()
        single_motor_target = public_api_types_pb2.SingleMotorTarget()

        if command.command_type == CommandType.BRAKE:
            for target in command.brake_command:
                single_motor_target.brake = target
                motor_targets.targets.append(deepcopy(single_motor_target))
        elif command.command_type == CommandType.SPEED:
            for target in command.speed_command:
                single_motor_target.speed = target
                motor_targets.targets.append(deepcopy(single_motor_target))
        elif command.command_type == CommandType.POSITION:
            # Convert to encoder position
            trans_positions = self.convert_rad_to_positions(
                np.array(command.position_command), pulse_per_rotation)

            for target in trans_positions:
                single_motor_target.position = int(target)
                motor_targets.targets.append(deepcopy(single_motor_target))
        elif command.command_type == CommandType.TORQUE:
            for target in command.torque_command:
                single_motor_target.torque = target
                motor_targets.targets.append(deepcopy(single_motor_target))
        elif command.command_type == CommandType.MIT:
            # Convert to encoder position
            raw_positions = np.array([cmd.position for cmd in command.mit_command])
            trans_positions = self.convert_rad_to_positions(
                raw_positions, pulse_per_rotation)

            for i, mit_cmd in enumerate(command.mit_command):
                mit_target = public_api_types_pb2.MitMotorTarget()
                mit_target.torque = mit_cmd.torque
                mit_target.speed = mit_cmd.speed
                mit_target.position = trans_positions[i]
                mit_target.kp = mit_cmd.kp
                mit_target.kd = mit_cmd.kd
                
                single_motor_target.mit_target.CopyFrom(mit_target)
                motor_targets.targets.append(deepcopy(single_motor_target))
        else:
            raise ValueError("construct_down_message: command_type error")
        return motor_targets

    def _construct_custom_motor_msg(
            self, command_type: CommandType,
            values) -> public_api_types_pb2.MotorTargets:
        """
        Set motor command
        
        Args:
            command_type: Command type (BRAKE, SPEED, POSITION, TORQUE)
            values: Command value list
                - BRAKE: Ignore values parameter
                - SPEED: Speed value list (rad/s)
                - POSITION: Position value list (encoder position)
                - TORQUE: Torque value list (Nm)
        """
        if command_type == CommandType.BRAKE:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} brake values, got {len(values)}"
                )
            command = MotorCommand.create_brake_command(values)
        elif command_type == CommandType.SPEED:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} speed values, got {len(values)}"
                )
            command = MotorCommand.create_speed_command(values)
        elif command_type == CommandType.POSITION:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} position values, got {len(values)}"
                )
            command = MotorCommand.create_position_command(
                values)
        elif command_type == CommandType.TORQUE:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} torque values, got {len(values)}"
                )
            command = MotorCommand.create_torque_command(values)
        elif command_type == CommandType.MIT:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} mit values, got {len(values)}"
                )
            command = MotorCommand.create_mit_command(values)
        else:
            raise ValueError(f"Unknown command type: {command_type}")

        # Get pulse_per_rotation values (not from queue, set once)
        with self._data_lock:
            if self._pulse_per_rotation is None:
                raise ValueError(f"Cannot construct custom motor message: pulse_per_rotation data not available (not set yet)")
            pulse_per_rotation_arr = self._pulse_per_rotation.copy()
        
        return MotorBase._construct_target_motor_msg(self, pulse_per_rotation_arr, command)

    def __str__(self) -> str:
        """String representation"""
        states = self.states(pop=False)  # Use pop=False to read latest without removing
        if states is None:
            return f"{self.name}(Count:{self.motor_count}, No data available)"
        normal_count = sum(1 for state in states if state == "normal")
        error_count = sum(1 for state in states if state == "error")
        return f"{self.name}(Count:{self.motor_count}, Normal:{normal_count}, Errors:{error_count})"

    def __repr__(self) -> str:
        """Detailed string representation"""
        return f"MotorBase(motor_count={self.motor_count}, name='{self.name}')"

    def __len__(self) -> int:
        """Return motor count"""
        return self.motor_count

    def __getitem__(self, motor_index: int) -> Dict[str, Any]:
        """Get motor status by index"""
        return self.get_motor_status(motor_index)
