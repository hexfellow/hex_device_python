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

from hex_device.common_utils import log_warn
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from enum import Enum
import threading
import time
import numpy as np
from copy import deepcopy
from collections import deque


@dataclass
class Timestamp:
    """Timestamp structure with seconds and nanoseconds
    
    Attributes:
        s: Seconds part of the timestamp
        ns: Nanoseconds part of the timestamp (0-999999999)
    """
    s: int
    ns: int
    
    @classmethod
    def from_ns(cls, timestamp_ns: int) -> 'Timestamp':
        """Create Timestamp from nanoseconds timestamp
        
        Args:
            timestamp_ns: Timestamp in nanoseconds (e.g., from time.perf_counter_ns())
        
        Returns:
            Timestamp object with s and ns components
        """
        return cls(
            s=timestamp_ns // 1_000_000_000,
            ns=timestamp_ns % 1_000_000_000
        )

    @classmethod
    def from_s_ns(cls, seconds: int, nanoseconds: int) -> 'Timestamp':
        """Create Timestamp from seconds and nanoseconds
        """
        return cls(
            s=seconds,
            ns=nanoseconds
        )
    
    def to_ns(self) -> int:
        """Convert Timestamp to nanoseconds
        
        Returns:
            Timestamp in nanoseconds
        """
        return self.s * 1_000_000_000 + self.ns
    
    def to_dict(self) -> Dict[str, int]:
        """Convert Timestamp to dictionary
        
        Returns:
            Dictionary with 's' and 'ns' keys
        """
        return {"s": self.s, "ns": self.ns}

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

        # Motor status data: stores tuples of (List[MotorStatus], timestamp)
        # Each entry contains all motors' status at a given timestamp
        self.__motor_data = deque(maxlen=10)

        # Motor motion data
        # self._positions = [deque(maxlen=10) for _ in range(motor_count)]  # Position (rad)
        # self._velocities = [deque(maxlen=10) for _ in range(motor_count)]  # Velocity (rad/s)
        # self._torques = [deque(maxlen=10) for _ in range(motor_count)]  # Torque (Nm)
        # self._encoder_positions = [deque(maxlen=10) for _ in range(motor_count)]  # Encoder position

        self._pulse_per_rotation: Optional[np.ndarray] = None  # Pulses per rotation (set once, not updated)
        self._wheel_radius: Optional[np.ndarray] = None  # Wheel radius (set once, not updated)
        # cache data, use to control the motor command
        self.__cache_positions = None
        self.__cache_velocities = None
        self.__cache_torques = None

        # Motor status parameters (optional)
        self._states = None  # "normal", "error"
        self._error_codes = None  # Use None to indicate no error
        self._driver_temperature = np.array(np.nan, dtype=np.float64) * motor_count  # Driver temperature (°C)
        self._motor_temperature = np.array(np.nan, dtype=np.float64) * motor_count  # Motor temperature (°C)
        self._voltage = np.array(np.nan, dtype=np.float64) * motor_count  # Voltage (V)
        self._last_update_time = None  # only update when status updated

        # Target commands
        self._current_targets = [deque(maxlen=10) for _ in range(motor_count)]  # Commands currently running on the device
        self._target_command = None  # The raw command, not converted to the scale in proto comments

        # Thread locks
        self._data_lock = threading.Lock()
        self._command_lock = threading.Lock()

        # Store custom conversion functions if provided
        self._custom_convert_positions_to_rad = convert_positions_to_rad_func
        self._custom_convert_rad_to_positions = convert_rad_to_positions_func

    @property
    def cache_motion_data(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """Get all motor cache motion data (positions radians, velocities rad/s, torques Nm)"""
        with self._data_lock:
            return self.__cache_positions.copy(), self.__cache_velocities.copy(), self.__cache_torques.copy()

    @property
    def cache_positions(self) -> Optional[np.ndarray]:
        """Get all motor cache positions (rad)"""
        with self._data_lock:
            return self.__cache_positions.copy()
    
    @property
    def cache_velocities(self) -> Optional[np.ndarray]:
        """Get all motor cache velocities (rad/s)"""
        with self._data_lock:
            return self.__cache_velocities.copy()
    
    @property
    def cache_torques(self) -> Optional[np.ndarray]:
        """Get all motor cache torques (Nm)"""
        with self._data_lock:
            return self.__cache_torques.copy()

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

    def get_motor_error_codes(self) -> Optional[List[Optional[int]]]:
        """Get all motor error codes
        
        Returns:
            List of error codes or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._error_codes)

    def get_motor_state(self, motor_index: int) -> Optional[str]:
        """Get specified motor state
        
        Args:
            motor_index: Motor index
        
        Returns:
            Motor state or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._states[motor_index])

    def get_motor_states(self) -> Optional[List[str]]:
        """Get all motor states
        
        Returns:
            List of motor states or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._states)

    def get_motor_encoder_positions(self, pop: bool = True) -> Optional[np.ndarray]:
        """Get all motor encoder positions
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Array of encoder positions (encoder count) or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[0]

    def get_motor_position(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor position (rad)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor position (rad) or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[1][motor_index]

    def get_motor_positions(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor positions (rad)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of positions (rad) or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[1]

    def get_encoders_to_zero(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor encoders to zero (rad)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of encoder positions to zero or None if queue is empty
        """
        with self._data_lock:
            encoder_positions = self.get_motor_encoder_positions(pop=pop)
            if encoder_positions is None:
                return None
            tar_arr = 32767 - np.array(encoder_positions, dtype=np.float64)
            return tar_arr.tolist()

    def get_motor_velocity(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor velocity (rad/s)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor velocity (rad/s) or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[2][motor_index]

    def get_motor_velocities(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor velocities (rad/s)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of velocities or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[2]

    def get_motor_torque(self, motor_index: int, pop: bool = True) -> Optional[float]:
        """Get specified motor torque (Nm)
        
        Args:
            motor_index: Motor index
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Motor torque (Nm) or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[3][motor_index]

    def get_motor_torques(self, pop: bool = True) -> Optional[List[float]]:
        """Get all motor torques (Nm)
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            List of torques (Nm) or None if queue is empty
        """
        return self._get_motor_motion_data(pop=pop)[3]

    def get_motor_driver_temperatures(self) -> Optional[np.ndarray]:
        """Get all motor driver temperatures (°C)
        
        Returns:
            Array of driver temperatures (°C) or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._driver_temperature)

    def get_motor_driver_temperature(self, motor_index: int) -> Optional[float]:
        """Get specified motor driver temperature (°C)
        
        Args:
            motor_index: Motor index
        
        Returns:
            Driver temperature or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._driver_temperature[motor_index])
        
    def get_motor_temperatures(self) -> Optional[np.ndarray]:
        """Get all motor temperatures (°C)
        
        Returns:
            Array of motor temperatures or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._motor_temperature)

    def get_motor_temperature(self, motor_index: int) -> Optional[float]:
        """Get specified motor temperature (°C)
        
        Args:
            motor_index: Motor index
        
        Returns:
            Motor temperature or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._motor_temperature[motor_index])

    def get_motor_voltages(self) -> Optional[np.ndarray]:
        """Get all motor voltages (V)
        
        Returns:
            Array of voltages or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._voltage)

    def get_motor_voltage(self, motor_index: int) -> Optional[float]:
        """Get specified motor voltage (V)
        
        Args:
            motor_index: Motor index
        
        Returns:
            Motor voltage or None if queue is empty
        """
        with self._data_lock:
            return deepcopy(self._voltage[motor_index])

    def get_motor_pulse_per_rotations(self) -> Optional[np.ndarray]:
        """Get all motor pulses per rotation
        
        Returns:
            Array of pulse per rotation values or None if not set
        """
        with self._data_lock:
            if self._pulse_per_rotation is None:
                return None
            return deepcopy(self._pulse_per_rotation)

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
            return deepcopy(float(self._pulse_per_rotation[motor_index]))

    def get_motor_wheel_radii(self) -> Optional[np.ndarray]:
        """Get all motor wheel radii (m)
        
        Returns:
            Array of wheel radii or None if not set
        """
        with self._data_lock:
            if self._wheel_radius is None:
                return None
            return deepcopy(self._wheel_radius)

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
            return deepcopy(float(self._wheel_radius[motor_index]))

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
            if not isinstance(values, list) or not all(isinstance(x, (int, float)) for x in values):
                raise ValueError("SPEED command type requires float list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} speed values, got {len(values)}"
                )
            command = MotorCommand.create_speed_command(values)
        elif command_type == CommandType.POSITION:
            if not isinstance(values, list) or not all(isinstance(x, (int, float)) for x in values):
                raise ValueError("POSITION command type requires float list")
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} position values, got {len(values)}"
                )
            command = MotorCommand.create_position_command(
                values)
        elif command_type == CommandType.TORQUE:
            if not isinstance(values, list) or not all(isinstance(x, (int, float)) for x in values):
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
        return positions / pulse_per_rotation * 2 * np.pi
    
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
        return positions / (2 * np.pi) * pulse_per_rotation

    def _push_motor_data(self, motor_status_list: List[public_api_types_pb2.MotorStatus], timestamp: Timestamp):
        """
        Push motor data for all motors
        
        Args:
            motor_status_list: List of motor status data for each motor
            timestamp: Timestamp object with s and ns components
        """
        if len(motor_status_list) != self.motor_count:
            log_warn(
                f"Warning: Motor count mismatch in _push_motor_data, expected {self.motor_count}, got {len(motor_status_list)}")
            return
        
        # Store tuple (List[MotorStatus], timestamp) as a single entry
        self.__motor_data.append((motor_status_list, timestamp))
        self._update_motor_status_data()

    def has_new_data(self) -> bool:
        """
        Check if there is new motor data
        
        Returns:
            True if there is new motor data, False otherwise
        """
        return len(self.__motor_data) > 0

    def _get_motor_data(self, pop: bool = True) -> Optional[Tuple[List[public_api_types_pb2.MotorStatus], Timestamp]]:
        """
        Get motor status list from deque
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
            
        Returns:
            Tuple of (List of MotorStatus objects, timestamp) or None if deque is empty
        """
        if not self.has_new_data():
            return None, None
        
        if pop:
            # Get and remove oldest data (FIFO)
            motor_status_list, timestamp = self.__motor_data.popleft()
            return motor_status_list, timestamp
        else:
            # Get latest data without removing
            motor_status_list, timestamp = self.__motor_data[-1]
            return motor_status_list, timestamp
    
    def _update_motor_status_data(self):
        """
        Update motor status data
        """
        motor_status_list, timestamp = self._get_motor_data(pop=False)
        if motor_status_list is None:
            # No data available yet
            return
        if len(motor_status_list) != self.motor_count:
            log_warn(
                f"Warning: Motor count mismatch, expected {self.motor_count}, actual {len(motor_status_list)}")
            return
        
        # Parse motor data
        positions = []
        velocities = []
        torques = []
        driver_temperature = []
        motor_temperature = []
        voltage = []
        error_codes = []
        current_targets = []
        states = []
        pulse_per_rotation = []
        wheel_radius = []

        for motor_status in motor_status_list:
            positions.append(motor_status.position)
            velocities.append(motor_status.speed)
            torques.append(motor_status.torque)
            current_targets.append(motor_status.current_target)
            pulse_per_rotation.append(motor_status.pulse_per_rotation)
            wheel_radius.append(motor_status.wheel_radius)

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
            state = "normal"
            if motor_status.error:
                error_code = motor_status.error[0]
                state = "error"
            error_codes.append(error_code)
            states.append(state)

        driver_temperature_arr = np.asarray(driver_temperature, dtype=np.float64)
        motor_temperature_arr = np.asarray(motor_temperature, dtype=np.float64)
        voltage_arr = np.asarray(voltage, dtype=np.float64)
        positions_arr = np.asarray(positions, dtype=np.float64)
        velocities_arr = np.asarray(velocities, dtype=np.float64)
        torques_arr = np.asarray(torques, dtype=np.float64)
        pulse_per_rotation_arr = np.asarray(pulse_per_rotation, dtype=np.float64)

        # Convert encoder positions to radians
        positions_rad_arr = self.convert_positions_to_rad(
            positions_arr, pulse_per_rotation_arr)

        # Now update all data within lock (add to queues)
        with self._data_lock:
            self.__cache_positions = positions_rad_arr.copy()  # ndarry copy is the same as deepcopy
            self.__cache_velocities = velocities_arr.copy()
            self.__cache_torques = torques_arr.copy()

            if self._pulse_per_rotation is None:
                self._pulse_per_rotation = pulse_per_rotation_arr.copy()
            if self._wheel_radius is None:
                wheel_radius_arr = np.asarray(wheel_radius, dtype=np.float64)
                self._wheel_radius = wheel_radius_arr

            self._driver_temperature = driver_temperature_arr
            self._motor_temperature = motor_temperature_arr
            self._voltage = voltage_arr
            self._states = states
            self._error_codes = error_codes

            if current_targets is not None:
                for i in range(self.motor_count):
                    self._current_targets[i].append(current_targets[i])
            self._last_update_time = timestamp
    
    def _get_motor_motion_data(self, pop: bool = True) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Timestamp]]:
        motor_status_list, timestamp = self._get_motor_data(pop=pop)
        if motor_status_list is None:
            # No data available yet
            return None, None, None, None, None
        if len(motor_status_list) != self.motor_count:
            log_warn(
                f"Warning: Motor count mismatch, expected {self.motor_count}, actual {len(motor_status_list)}")
            return None, None, None, None, None
        # Parse motor data
        positions = []  # encoder position
        velocities = []  # rad/s
        torques = []  # Nm
        pulse_per_rotation = []

        for motor_status in motor_status_list:
            positions.append(motor_status.position)
            velocities.append(motor_status.speed)
            torques.append(motor_status.torque)
            pulse_per_rotation.append(motor_status.pulse_per_rotation)

        velocities_arr = np.asarray(velocities, dtype=np.float64)
        torques_arr = np.asarray(torques, dtype=np.float64)
        positions_arr = np.asarray(positions, dtype=np.float64)
        pulse_per_rotation_arr = np.asarray(pulse_per_rotation, dtype=np.float64)

        # Convert encoder positions to radians
        positions_rad_arr = self.convert_positions_to_rad(
            positions_arr, pulse_per_rotation_arr.copy())

        return positions_arr, positions_rad_arr, velocities_arr, torques_arr, timestamp

    def get_motor_summary(self) -> Optional[Dict[str, Any]]:
        """Get status summary
        
        Returns:
            Dictionary with motor summary or None if no data available
        """
        with self._data_lock:
            # Check if data is available
            if (self._states is None or self._error_codes is None or 
                self.__cache_positions is None or self.__cache_velocities is None or 
                self.__cache_torques is None):
                return None
            
            # Get data from cache
            states = self._states.copy()
            error_codes = self._error_codes.copy()
            positions = self.__cache_positions.tolist()
            velocities = self.__cache_velocities.tolist()
            torques = self.__cache_torques.tolist()
            driver_temperature = self._driver_temperature.tolist()
            motor_temperature = self._motor_temperature.tolist()
            voltage = self._voltage.tolist()
            
            # Get pulse_per_rotation and wheel_radius (not from queues)
            if self._pulse_per_rotation is not None:
                pulse_per_rotation = self._pulse_per_rotation.tolist()
            else:
                pulse_per_rotation = None
            
            if self._wheel_radius is not None:
                wheel_radius = self._wheel_radius.tolist()
            else:
                wheel_radius = None
            
            # Convert Timestamp to dict if available
            last_update_time = None
            if self._last_update_time is not None:
                last_update_time = self._last_update_time.to_dict()
            
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
            # Check if data is available
            if (self._states is None or self._error_codes is None or 
                self.__cache_positions is None or self.__cache_velocities is None or 
                self.__cache_torques is None):
                return None
            
            # Get data from cache (pop parameter is ignored since we use cache now)
            status = {
                'index': motor_index,
                'state': self._states[motor_index],
                'error_code': self._error_codes[motor_index],
                'position': float(self.__cache_positions[motor_index]),
                'velocity': float(self.__cache_velocities[motor_index]),
                'torque': float(self.__cache_torques[motor_index]),
                'driver_temperature': float(self._driver_temperature[motor_index]),
                'motor_temperature': float(self._motor_temperature[motor_index]),
                'voltage': float(self._voltage[motor_index]),
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
    
    def flush_motor_data(self):
        """
        Clear all queues in MotorBase
        
        This method removes all data from all queues including:
        """
        with self._data_lock:
            # Clear all motor-specific queues
            self.__motor_data.clear()

    def get_simple_motor_status(self, pop: bool = True) -> Optional[Dict[str, Any]]:
        """Get simple motor status
        
        Args:
            pop: If True, pops from queue (FIFO). If False, reads latest data without popping.
        
        Returns:
            Dictionary with simple motor status or None if queue is empty
        """
        _, positions, velocities, torques, last_update_time = self._get_motor_motion_data(pop=pop)
        if last_update_time is None:
            return None
        return {
            'pos': positions,
            'vel': velocities,
            'eff': torques,
            'ts': last_update_time.to_dict()
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
        states = self.get_motor_states()
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
