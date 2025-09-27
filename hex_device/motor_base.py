#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any, Union
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from enum import Enum
import threading
import time
import numpy as np
from copy import deepcopy


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

    def __init__(self, motor_count: int, name: str = ""):
        """
        Initialize motor base class
        Args:
            motor_count: Number of motors
            name: Motor group name
        """
        self.motor_count = motor_count
        self.name = name or f"MotorGroup"

        self._states = ["normal"] * motor_count  # "normal", "error"
        self._error_codes = [None] * motor_count  # Use None to indicate no error

        # Motor motion data
        self._torques = np.zeros(motor_count)  # Torque (Nm)
        self._velocities = np.zeros(motor_count)  # Velocity (rad/s)
        self._positions = np.zeros(motor_count)  # Position (rad)
        self._pulse_per_rotation = np.zeros(motor_count)  # Pulses per rotation
        self._wheel_radius = np.zeros(motor_count)  # Wheel radius

        # Motor status parameters (optional)
        self._driver_temperature = np.zeros(motor_count)  # Driver temperature (°C)
        self._motor_temperature = np.zeros(motor_count)  # Motor temperature (°C)
        self._voltage = np.zeros(motor_count)  # Voltage (V)

        # Target commands
        self._current_targets = [None] * motor_count  # Commands currently running on the device
        self._target_command = None  # The raw command, not converted to the scale in proto comments

        # Timestamp
        self._last_update_time = None

        # Thread locks
        self._data_lock = threading.Lock()
        self._command_lock = threading.Lock()

        # Data update flag
        self._has_new_data = False

    @property
    def states(self) -> List[str]:
        """Get all motor states"""
        with self._data_lock:
            return self._states.copy()

    @property
    def error_codes(self) -> List[Optional[int]]:
        """Get all motor error codes"""
        with self._data_lock:
            return self._error_codes.copy()

    @property
    def positions(self) -> np.ndarray:
        """Get all motor positions (rad)"""
        with self._data_lock:
            return self._positions.copy()

    @property
    def velocities(self) -> np.ndarray:
        """Get all motor velocities (rad/s)"""
        with self._data_lock:
            return self._velocities.copy()

    @property
    def torques(self) -> np.ndarray:
        """Get all motor torques (Nm)"""
        with self._data_lock:
            return self._torques.copy()

    @property
    def driver_temperature(self) -> np.ndarray:
        """Get all motor driver temperatures (°C)"""
        with self._data_lock:
            return self._driver_temperature.copy()

    @property
    def motor_temperature(self) -> np.ndarray:
        """Get all motor temperatures (°C)"""
        with self._data_lock:
            return self._motor_temperature.copy()

    @property
    def voltage(self) -> np.ndarray:
        """Get all motor voltages (V)"""
        with self._data_lock:
            return self._voltage.copy()

    @property
    def pulse_per_rotation(self) -> np.ndarray:
        """Get all motor pulses per rotation"""
        with self._data_lock:
            return self._pulse_per_rotation.copy()

    @property
    def wheel_radius(self) -> np.ndarray:
        """Get all motor wheel radii (m)"""
        with self._data_lock:
            return self._wheel_radius.copy()

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

    @property
    def has_new_data(self) -> bool:
        """Check if there is new data"""
        with self._data_lock:
            return self._has_new_data

    def get_motor_state(self, motor_index: int) -> str:
        """Get specified motor state"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._states[motor_index]

    def get_motor_position(self, motor_index: int) -> float:
        """Get specified motor position (rad)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._positions[motor_index]

    def get_motor_positions(self) -> List[float]:
        """Get all motor positions (rad)"""
        with self._data_lock:
            self._has_new_data = False
            return self._positions.tolist()

    def get_motor_velocity(self, motor_index: int) -> float:
        """Get specified motor velocity (rad/s)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._velocities[motor_index]

    def get_motor_velocities(self) -> List[float]:
        """Get all motor velocities (rad/s)"""
        with self._data_lock:
            self._has_new_data = False
            return self._velocities.tolist()

    def get_motor_torque(self, motor_index: int) -> float:
        """Get specified motor torque (Nm)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._torques[motor_index]

    def get_motor_torques(self) -> List[float]:
        """Get all motor torques (Nm)"""
        with self._data_lock:
            self._has_new_data = False
            return self._torques.tolist()

    def get_motor_driver_temperature(self, motor_index: int) -> float:
        """Get specified motor driver temperature (°C)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._driver_temperature[motor_index]

    def get_motor_temperature(self, motor_index: int) -> float:
        """Get specified motor temperature (°C)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._motor_temperature[motor_index]

    def get_motor_voltage(self, motor_index: int) -> float:
        """Get specified motor voltage (V)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            self._has_new_data = False
            return self._voltage[motor_index]

    def get_motor_pulse_per_rotation(self, motor_index: int) -> float:
        """Get specified motor pulses per rotation"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._pulse_per_rotation[motor_index]

    def get_motor_wheel_radius(self, motor_index: int) -> float:
        """Get specified motor wheel radius (m)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._wheel_radius[motor_index]

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

        with self._data_lock:
            self._velocities = np.array(velocities)
            self._torques = np.array(torques)
            self._driver_temperature = np.array(driver_temperature)
            self._motor_temperature = np.array(motor_temperature)
            self._voltage = np.array(voltage)

            if pulse_per_rotation is not None:
                self._pulse_per_rotation = np.array(pulse_per_rotation)

            # Convert to rad
            self._positions = (np.array(positions) - 65535.0 /
                               2.0) / self._pulse_per_rotation * 2 * np.pi

            if wheel_radius is not None:
                self._wheel_radius = np.array(wheel_radius)

            if error_codes is not None:
                if len(error_codes) != self.motor_count:
                    raise ValueError(
                        f"Expected {self.motor_count} error codes, got {len(error_codes)}"
                    )
                self._error_codes = error_codes.copy()

                # Update state based on error codes
                for i, error_code in enumerate(error_codes):
                    if error_code is not None:
                        self._states[i] = "error"
                    elif self._states[i] == "error":
                        self._states[i] = "normal"

            if current_targets is not None:
                self._current_targets = current_targets.copy()

            self._last_update_time = time.time_ns()
            self._has_new_data = True

    def clear_new_data_flag(self):
        """Clear new data flag"""
        with self._data_lock:
            self._has_new_data = False

    def get_motor_summary(self) -> Dict[str, Any]:
        """Get status summary"""
        with self._data_lock:
            summary = {
                'name': self.name,
                'motor_count': self.motor_count,
                'states': self._states.copy(),
                'error_codes': self._error_codes.copy(),
                'positions': self._positions.tolist(),
                'velocities': self._velocities.tolist(),
                'torques': self._torques.tolist(),
                'driver_temperature': self._driver_temperature.tolist(),
                'motor_temperature': self._motor_temperature.tolist(),
                'voltage': self._voltage.tolist(),
                'pulse_per_rotation': self._pulse_per_rotation.tolist(),
                'wheel_radius': self._wheel_radius.tolist(),
                'last_update_time': self._last_update_time,
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

    def get_motor_status(self, motor_index: int) -> Dict[str, Any]:
        """Get specified motor state"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )

        with self._data_lock:
            status = {
                'index': motor_index,
                'state': self._states[motor_index],
                'error_code': self._error_codes[motor_index],
                'position': self._positions[motor_index],
                'velocity': self._velocities[motor_index],
                'torque': self._torques[motor_index],
                'driver_temperature': self._driver_temperature[motor_index],
                'motor_temperature': self._motor_temperature[motor_index],
                'voltage': self._voltage[motor_index],
                'pulse_per_rotation': self._pulse_per_rotation[motor_index],
                'wheel_radius': self._wheel_radius[motor_index]
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

    def get_simple_motor_status(self) -> Dict[str, Any]:
        """Get simple motor status"""
        with self._data_lock:
            return {
                'pos': self._positions.tolist(),
                'vel': self._velocities.tolist(),
                'eff': self._torques.tolist(),
                'ts': {
                        "s": self._last_update_time // 1_000_000_000,
                        "ns": self._last_update_time % 1_000_000_000,
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
            trans_positions = np.array(command.position_command) / (
                2 * np.pi) * pulse_per_rotation + 65535.0 / 2.0

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
            trans_positions = raw_positions / (2 * np.pi) * pulse_per_rotation + 65535.0 / 2.0

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

        return MotorBase._construct_target_motor_msg(self, self._pulse_per_rotation, command)

    def __str__(self) -> str:
        """String representation"""
        normal_count = sum(1 for state in self.states if state == "normal")
        error_count = sum(1 for state in self.states if state == "error")
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
