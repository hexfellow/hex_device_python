#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from enum import Enum
import threading
import time
import numpy as np
from copy import deepcopy


class CommandType(Enum):
    """指令类型枚举"""
    BRAKE = "brake"
    SPEED = "speed"
    POSITION = "position"
    TORQUE = "torque"


@dataclass
class MotorCommand:
    """电机指令结构体

    可以选择输入四种指令：
    1. brake指令 - bool类型
    2. 速度指令 - 浮点数组类型
    3. 位置指令 - 浮点数组类型
    4. 力矩指令 - 浮点数组类型
    """
    command_type: CommandType
    brake_command: Optional[List[bool]] = None
    speed_command: Optional[List[float]] = None
    position_command: Optional[List[float]] = None
    torque_command: Optional[List[float]] = None

    def __post_init__(self):
        """验证指令数据的有效性"""
        if self.command_type == CommandType.BRAKE:
            if self.brake_command is None:
                raise ValueError("brake指令类型需要提供brake_command参数")
            if self.speed_command is not None or self.position_command is not None or self.torque_command is not None:
                raise ValueError(
                    "brake指令类型不应包含speed_command、position_command或torque_command"
                )

        elif self.command_type == CommandType.SPEED:
            if self.speed_command is None:
                raise ValueError("speed指令类型需要提供speed_command参数")
            if self.brake_command is not None or self.position_command is not None or self.torque_command is not None:
                raise ValueError(
                    "speed指令类型不应包含brake_command、position_command或torque_command"
                )
            if not isinstance(self.speed_command, list) or not all(
                    isinstance(x, (int, float)) for x in self.speed_command):
                raise ValueError("speed_command必须是浮点数组")

        elif self.command_type == CommandType.POSITION:
            if self.position_command is None:
                raise ValueError("position指令类型需要提供position_command参数")
            if self.brake_command is not None or self.speed_command is not None or self.torque_command is not None:
                raise ValueError(
                    "position指令类型不应包含brake_command、speed_command或torque_command"
                )
            if not isinstance(self.position_command, list) or not all(
                    isinstance(x, (int, float))
                    for x in self.position_command):
                raise ValueError("position_command必须是浮点数组")

        elif self.command_type == CommandType.TORQUE:
            if self.torque_command is None:
                raise ValueError("torque指令类型需要提供torque_command参数")
            if self.brake_command is not None or self.speed_command is not None or self.position_command is not None:
                raise ValueError(
                    "torque指令类型不应包含brake_command、speed_command或position_command"
                )
            if not isinstance(self.torque_command, list) or not all(
                    isinstance(x, (int, float)) for x in self.torque_command):
                raise ValueError("torque_command必须是浮点数组")

    @classmethod
    def create_brake_command(cls, brake: bool) -> 'MotorCommand':
        """创建brake指令"""
        return cls(command_type=CommandType.BRAKE, brake_command=brake)

    @classmethod
    def create_speed_command(cls, speeds: List[float]) -> 'MotorCommand':
        """创建速度指令"""
        return cls(command_type=CommandType.SPEED, speed_command=speeds)

    @classmethod
    def create_position_command(
            cls,
            positions: List[float],
            pulse_per_rotation: np.ndarray = None) -> 'MotorCommand':
        """创建位置指令"""
        if pulse_per_rotation is None:
            return cls(command_type=CommandType.POSITION,
                       position_command=positions)

        #trans to encoder position
        trans_positions = np.array(positions) / (
            2 * np.pi) * pulse_per_rotation + 65535.0 / 2.0
        return cls(command_type=CommandType.POSITION,
                   position_command=trans_positions.tolist())

    @classmethod
    def create_torque_command(cls, torques: List[float]) -> 'MotorCommand':
        """创建力矩指令"""
        return cls(command_type=CommandType.TORQUE, torque_command=torques)


class MotorError(Enum):
    """电机错误枚举，用于实现从proto中的MotorError到python类的映射"""
    ME_COMMUNICATION_ERROR = 0
    ME_OVER_CURRENT = 1
    ME_OVER_VOLTAGE = 2
    ME_UNDER_VOLTAGE = 3
    ME_MOTOR_OVER_TEMPERATURE = 4
    ME_DRIVER_OVER_TEMPERATURE = 5
    ME_GENERAL_ERROR = 6


class MotorBase(ABC):
    """
    电机基类
    管理多个电机的数组形式，定义了电机的基本接口和通用功能
    该类对应proto中的MotorStatus
    """

    def __init__(self, motor_count: int, name: str = ""):
        """
        初始化电机基类
        Args:
            motor_count: 电机数量
            name: 电机组名称
        """
        self.motor_count = motor_count
        self.name = name or f"MotorGroup"

        # 电机状态数组（使用字符串状态）
        self._states = ["normal"] * motor_count  # "normal", "error"
        self._error_codes = [None] * motor_count  # 使用None表示无错误

        # 电机运动数据
        self._torques = np.zeros(motor_count)  # 扭矩 (Nm)
        self._velocities = np.zeros(motor_count)  # 速度 (rad/s)
        self._positions = np.zeros(motor_count)  # 位置 (rad)
        self._pulse_per_rotation = np.zeros(motor_count)  # 每转脉冲数
        self._wheel_radius = np.zeros(motor_count)  # 轮子半径

        # 电机状态参数（optional）
        self._driver_temperature = np.zeros(motor_count)  # 驱动器温度 (°C)
        self._motor_temperature = np.zeros(motor_count)  # 电机温度 (°C)
        self._voltage = np.zeros(motor_count)  # 电压 (V)

        # 目标指令
        self._current_targets = [None] * motor_count  # 当前设备正在运行的指令
        self._target_command = None  # 当前目标指令

        # 时间戳
        self._last_update_time = None

        # 线程锁
        self._data_lock = threading.Lock()
        self._command_lock = threading.Lock()

        # 数据更新标志
        self._has_new_data = False

    @property
    def states(self) -> List[str]:
        """获取所有电机状态"""
        with self._data_lock:
            return self._states.copy()

    @property
    def error_codes(self) -> List[Optional[int]]:
        """获取所有电机错误代码"""
        with self._data_lock:
            return self._error_codes.copy()

    @property
    def positions(self) -> np.ndarray:
        """获取所有电机位置 (rad)"""
        with self._data_lock:
            return self._positions.copy()

    @property
    def velocities(self) -> np.ndarray:
        """获取所有电机速度 (rad/s)"""
        with self._data_lock:
            return self._velocities.copy()

    @property
    def torques(self) -> np.ndarray:
        """获取所有电机扭矩 (Nm)"""
        with self._data_lock:
            return self._torques.copy()

    @property
    def driver_temperature(self) -> np.ndarray:
        """获取所有电机驱动器温度 (°C)"""
        with self._data_lock:
            return self._driver_temperature.copy()

    @property
    def motor_temperature(self) -> np.ndarray:
        """获取所有电机温度 (°C)"""
        with self._data_lock:
            return self._motor_temperature.copy()

    @property
    def voltage(self) -> np.ndarray:
        """获取所有电机电压 (V)"""
        with self._data_lock:
            return self._voltage.copy()

    @property
    def pulse_per_rotation(self) -> np.ndarray:
        """获取所有电机每转脉冲数"""
        with self._data_lock:
            return self._pulse_per_rotation.copy()

    @property
    def wheel_radius(self) -> np.ndarray:
        """获取所有电机轮子半径 (m)"""
        with self._data_lock:
            return self._wheel_radius.copy()

    @property
    def target_positions(self) -> np.ndarray:
        """获取所有电机目标位置 (rad)"""
        with self._command_lock:
            if self._target_command and self._target_command.command_type == CommandType.POSITION:
                return np.array(self._target_command.position_command)
            return np.zeros(self.motor_count)

    @property
    def target_velocities(self) -> np.ndarray:
        """获取所有电机目标速度 (rad/s)"""
        with self._command_lock:
            if self._target_command and self._target_command.command_type == CommandType.SPEED:
                return np.array(self._target_command.speed_command)
            return np.zeros(self.motor_count)

    @property
    def target_torques(self) -> np.ndarray:
        """获取所有电机目标扭矩 (Nm)"""
        with self._command_lock:
            if self._target_command and self._target_command.command_type == CommandType.TORQUE:
                return np.array(self._target_command.torque_command)
            return np.zeros(self.motor_count)

    @property
    def has_new_data(self) -> bool:
        """检查是否有新数据"""
        with self._data_lock:
            return self._has_new_data

    def get_motor_state(self, motor_index: int) -> str:
        """获取指定电机状态"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._states[motor_index]

    def get_motor_position(self, motor_index: int) -> float:
        """获取指定电机位置 (rad)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._positions[motor_index]

    def get_motor_positions(self) -> List[float]:
        """获取所有电机位置 (rad)"""
        with self._data_lock:
            return self._positions.tolist()

    def get_motor_velocity(self, motor_index: int) -> float:
        """获取指定电机速度 (rad/s)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._velocities[motor_index]

    def get_motor_velocities(self) -> List[float]:
        """获取所有电机速度 (rad/s)"""
        with self._data_lock:
            return self._velocities.tolist()

    def get_motor_torque(self, motor_index: int) -> float:
        """获取指定电机扭矩 (Nm)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._torques[motor_index]

    def get_motor_torques(self) -> List[float]:
        """获取所有电机扭矩 (Nm)"""
        with self._data_lock:
            return self._torques.tolist()

    def get_motor_driver_temperature(self, motor_index: int) -> float:
        """获取指定电机驱动器温度 (°C)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._driver_temperature[motor_index]

    def get_motor_temperature(self, motor_index: int) -> float:
        """获取指定电机温度 (°C)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._motor_temperature[motor_index]

    def get_motor_voltage(self, motor_index: int) -> float:
        """获取指定电机电压 (V)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._voltage[motor_index]

    def get_motor_pulse_per_rotation(self, motor_index: int) -> float:
        """获取指定电机每转脉冲数"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._pulse_per_rotation[motor_index]

    def get_motor_wheel_radius(self, motor_index: int) -> float:
        """获取指定电机轮子半径 (m)"""
        if not 0 <= motor_index < self.motor_count:
            raise IndexError(
                f"Motor index {motor_index} out of range [0, {self.motor_count})"
            )
        with self._data_lock:
            return self._wheel_radius[motor_index]

    def motor_command(self, command_type: CommandType, values: List[float]):
        """
        设置电机指令
        
        Args:
            command_type: 指令类型 (BRAKE, SPEED, POSITION, TORQUE)
            values: 指令值列表
                - BRAKE: 忽略values参数
                - SPEED: 速度值列表 (rad/s)
                - POSITION: 位置值列表 (rad)
                - TORQUE: 力矩值列表 (Nm)
        """
        if command_type == CommandType.BRAKE:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} brake values, got {len(values)}"
                )
            command = MotorCommand.create_brake_command(True)
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
                values, self._pulse_per_rotation)
        elif command_type == CommandType.TORQUE:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} torque values, got {len(values)}"
                )
            command = MotorCommand.create_torque_command(values)
        else:
            raise ValueError(f"Unknown command type: {command_type}")

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
        更新所有电机数据
        
        Args:
            positions: 位置列表 (encoder position)
            velocities: 速度列表 (rad/s)
            torques: 扭矩列表 (Nm)
            driver_temperature: 驱动器温度列表 (°C)
            motor_temperature: 电机温度列表 (°C)
            voltage: 电压列表 (V)
            pulse_per_rotation: 每转脉冲数列表，None表示不更新
            wheel_radius: 轮子半径列表 (m)，None表示不更新
            error_codes: 错误代码列表，None表示无错误
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

            #trans to rad
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

            self._last_update_time = time.time()
            self._has_new_data = True

    def clear_new_data_flag(self):
        """清除新数据标志"""
        with self._data_lock:
            self._has_new_data = False

    def get_motor_summary(self) -> Dict[str, Any]:
        """获取状态摘要"""
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

            # 添加目标指令信息
            if self._target_command:
                summary['target_command'] = {
                    'command_type': self._target_command.command_type.value,
                    'brake_command': self._target_command.brake_command,
                    'speed_command': self._target_command.speed_command,
                    'position_command': self._target_command.position_command,
                    'torque_command': self._target_command.torque_command
                }
            else:
                summary['target_command'] = None

            return summary

    def get_motor_status(self, motor_index: int) -> Dict[str, Any]:
        """获取指定电机状态"""
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

            # 添加目标指令信息
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

    def _construct_target_motor_msg(
            self,
            command: MotorCommand = None) -> public_api_types_pb2.MotorTargets:
        """构造下行消息"""
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
            for target in command.position_command:
                single_motor_target.position = int(target)
                motor_targets.targets.append(deepcopy(single_motor_target))
        elif command.command_type == CommandType.TORQUE:
            for target in command.torque_command:
                single_motor_target.torque = target
                motor_targets.targets.append(deepcopy(single_motor_target))
        else:
            raise ValueError("construct_down_message: command_type error")
        return motor_targets

    def _construct_custom_motor_msg(
            self, command_type: CommandType,
            values: List[float]) -> public_api_types_pb2.MotorTargets:
        """
        设置电机指令
        
        Args:
            command_type: 指令类型 (BRAKE, SPEED, POSITION, TORQUE)
            values: 指令值列表
                - BRAKE: 忽略values参数
                - SPEED: 速度值列表 (rad/s)
                - POSITION: 位置值列表 (rad)
                - TORQUE: 力矩值列表 (Nm)
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
                values, self._pulse_per_rotation)
        elif command_type == CommandType.TORQUE:
            if len(values) != self.motor_count:
                raise ValueError(
                    f"Expected {self.motor_count} torque values, got {len(values)}"
                )
            command = MotorCommand.create_torque_command(values)
        else:
            raise ValueError(f"Unknown command type: {command_type}")

        return self._construct_target_motor_msg(command)

    def __str__(self) -> str:
        """字符串表示"""
        normal_count = sum(1 for state in self.states if state == "normal")
        error_count = sum(1 for state in self.states if state == "error")
        return f"{self.name}(Count:{self.motor_count}, Normal:{normal_count}, Errors:{error_count})"

    def __repr__(self) -> str:
        """详细字符串表示"""
        return f"MotorBase(motor_count={self.motor_count}, name='{self.name}')"

    def __len__(self) -> int:
        """返回电机数量"""
        return self.motor_count

    def __getitem__(self, motor_index: int) -> Dict[str, Any]:
        """通过索引获取电机状态"""
        return self.get_motor_status(motor_index)
