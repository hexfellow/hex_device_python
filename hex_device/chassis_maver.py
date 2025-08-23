#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-8-1
################################################################

from typing import Optional, List, Dict, Any, Tuple
import numpy as np
from copy import deepcopy
from .common_utils import delay, log_common, log_info, log_warn
from .device_base import DeviceBase
from .common_utils import log_err
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .motor_base import MotorBase, MotorError, MotorCommand, CommandType
from .generated.public_api_types_pb2 import (
    BaseStatus, BaseState, BaseCommand, SimpleBaseMoveCommand, XyzSpeed,
    MotorTargets, BaseEstimatedOdometry, WarningCategory)
import time

from hex_device import motor_base


class ChassisMaver(DeviceBase, MotorBase):
    """
    底盘Maver类
    
    继承自DeviceBase和MotorBase，主要实现对BaseStatus的映射
    该类对应proto中的BaseStatus，管理底盘状态和电机控制
    
    支持的机器人类型:
    - RtCustomPcwVehicle: 自定义PCW车辆
    - RtPcwVehicle: PCW车辆
    """
    
    # 支持的机器人类型列表
    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtCustomPcwVehicle,
        public_api_types_pb2.RobotType.RtPcwVehicle
    ]

    def __init__(self, motor_count: int = 8, name: str = "ChassisMaver", control_hz: int = 500, 
                 send_message_callback=None):
        """
        初始化底盘Maver
        
        Args:
            motor_count: 电机数量，默认为8
            name: 设备名称
            control_hz: 控制频率
            send_message_callback: 发送消息的回调函数，用于发送下行消息
        """
        # 初始化父类
        DeviceBase.__init__(self, name, send_message_callback)
        MotorBase.__init__(self, motor_count, name)
        # 确保 name 属性正确设置（DeviceBase 的 name 优先级更高）
        self.name = name or "ChassisMaver"
        self._control_hz = control_hz
        self._target_zero_resistance = False
        self._target_velocity = (0.0, 0.0, 0.0)  # 初始化目标速度

        # 底盘状态
        self._base_state = BaseState.BsParked
        self._api_control_initialized = False
        self._simple_control_mode = None

        # 电池信息
        self._battery_voltage = 0.0
        self._battery_thousandth = 0

        # 可选字段
        self._battery_charging = None
        self._parking_stop_detail = None
        self._warning = None

        # 里程计信息
        self.__vehicle_origin_position = np.eye(3)  # 用于清除里程计偏差
        self._vehicle_speed = (0.0, 0.0, 0.0)  # (x, y, z) m/s, m/s, rad/s
        self._vehicle_position = (0.0, 0.0, 0.0)  # (x, y, yaw) m, m, rad

        # 控制相关
        self._last_command_time = time.time()
        self._command_timeout = 0.1  # 100ms超时
        self.__last_warning_time = time.time()  # 添加警告时间属性
        
        # 机器人类型 - 将在匹配时设置
        self.robot_type = None

    def set_robot_type(self, robot_type):
        """
        设置机器人类型
        
        Args:
            robot_type: 机器人类型
        """
        if robot_type in self.SUPPORTED_ROBOT_TYPES:
            self.robot_type = robot_type
        else:
            raise ValueError(f"不支持的机器人类型: {robot_type}")

    @classmethod
    def supports_robot_type(cls, robot_type):
        """
        检查是否支持指定的机器人类型
        
        Args:
            robot_type: 机器人类型
            
        Returns:
            bool: 是否支持
        """
        return robot_type in cls.SUPPORTED_ROBOT_TYPES


    async def _init(self) -> bool:
        """
        初始化底盘
        
        Returns:
            bool: 是否成功初始化
        """
        try:
            msg = self._construct_init_message()
            await self._send_message(msg)
            self.set_has_new_data()
            return True
        except Exception as e:
            log_err(f"ChassisMaver初始化失败: {e}")
            return False

    def _update(self, api_up_data) -> bool:
        """
        更新底盘数据
        
        Args:
            api_up_data: 从API接收的上行数据 (APIUp)
            
        Returns:
            bool: 是否成功更新
        """
        try:
            # 检查是否包含BaseStatus
            if not api_up_data.HasField('base_status'):
                return False

            base_status = api_up_data.base_status

            # 更新底盘状态
            self._base_state = base_status.state
            self._api_control_initialized = base_status.api_control_initialized
            self._battery_voltage = base_status.battery_voltage
            self._battery_thousandth = base_status.battery_thousandth

            # 更新可选字段
            if base_status.HasField('battery_charging'):
                self._battery_charging = base_status.battery_charging
            if base_status.HasField('parking_stop_detail'):
                self._parking_stop_detail = base_status.parking_stop_detail
            if base_status.HasField('warning'):
                self._warning = base_status.warning
            if base_status.HasField('estimated_odometry'):
                self._vehicle_speed = (base_status.estimated_odometry.speed_x,
                                       base_status.estimated_odometry.speed_y,
                                       base_status.estimated_odometry.speed_z)
                self._vehicle_position = (base_status.estimated_odometry.pos_x,
                                          base_status.estimated_odometry.pos_y,
                                          base_status.estimated_odometry.pos_z)

            # 更新电机数据
            self._update_motor_data_from_base_status(base_status)
            self.set_has_new_data()
            return True
        except Exception as e:
            log_err(f"ChassisMaver数据更新失败: {e}")
            return False

    def _update_motor_data_from_base_status(self, base_status: BaseStatus):
        """
        从BaseStatus更新电机数据
        
        Args:
            base_status: BaseStatus对象
        """
        motor_status_list = base_status.motor_status

        if len(motor_status_list) != self.motor_count:
            log_warn(
                f"警告: 电机数量不匹配，期望{self.motor_count}，实际{len(motor_status_list)}")
            return

        # 解析电机数据
        positions = []
        velocities = []
        torques = []
        driver_temperature = []
        motor_temperature = []
        pulse_per_rotation = []
        wheel_radius = []
        voltage = []
        error_codes = []

        for motor_status in motor_status_list:
            # 位置 (从encoder position转换)
            positions.append(float(motor_status.position))
            # 速度 (从speed转换)
            velocities.append(motor_status.speed)
            # 扭矩
            torques.append(motor_status.torque)
            # 额外参数
            pulse_per_rotation.append(motor_status.pulse_per_rotation)
            wheel_radius.append(motor_status.wheel_radius)

            # 温度
            driver_temp = motor_status.driver_temperature if motor_status.HasField(
                'driver_temperature') else 0.0
            motor_temp = motor_status.motor_temperature if motor_status.HasField(
                'motor_temperature') else 0.0
            driver_temperature.append(driver_temp)
            motor_temperature.append(motor_temp)

            # 电压
            volt = motor_status.voltage if motor_status.HasField(
                'voltage') else 0.0
            voltage.append(volt)

            # 错误代码
            error_code = None
            if motor_status.error:
                # 取第一个错误
                error_code = motor_status.error[0].value
            error_codes.append(error_code)

        # 更新电机数据
        self.update_data(positions=positions,
                         velocities=velocities,
                         torques=torques,
                         driver_temperature=driver_temperature,
                         motor_temperature=motor_temperature,
                         voltage=voltage,
                         pulse_per_rotation=pulse_per_rotation,
                         wheel_radius=wheel_radius,
                         error_codes=error_codes)

    async def _periodic(self):
        """
        周期性执行函数
        
        执行底盘的周期性任务，包括：
        - 状态检查
        - 命令超时检查
        - 安全监控
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time

        await self._init()
        log_info("ChassisMaver init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                ## check if data is updated
                if not self.has_new_data():
                    continue

                # check error
                if self.get_parking_stop_detail() != public_api_types_pb2.ParkingStopDetail():
                    if start_time - self.__last_warning_time > 1.0:
                        log_err(
                            f"emergency stop: {self.get_parking_stop_detail()}"
                        )
                    self.__last_warning_time = start_time

                # 检查电机状态
                for i in range(self.motor_count):
                    if self.get_motor_state(i) == "error":
                        log_err(f"警告: 电机{i}出现错误")

                # send control message
                if self._simple_control_mode == True:
                    if self._target_zero_resistance:
                        msg = self._construct_zero_resistance_message(True, True)
                        await self._send_message(msg)

                    else:
                        msg = self._construct_zero_resistance_message(False, True)
                        await self._send_message(msg)

                        if time.time() - self._last_command_time > self._command_timeout:
                            msg = self._construct_simple_control_message(
                                (0.0, 0.0, 0.0))
                        else:
                            msg = self._construct_simple_control_message(
                                self._target_velocity)
                        await self._send_message(msg)

                elif self._simple_control_mode == False:
                    if self._target_zero_resistance:
                        msg = self._construct_zero_resistance_message(True, False)
                    else:
                        msg = self._construct_zero_resistance_message(False, False)
                    await self._send_message(msg)

                    if time.time() - self._last_command_time > self._command_timeout:
                        self.motor_command(CommandType.BRAKE, [])
                        msg = self._construct_wheel_control_message()
                    else:
                        msg = self._construct_wheel_control_message()
                    await self._send_message(msg)

                elif self._simple_control_mode is None:
                    msg = self._construct_init_message()
                    await self._send_message(msg)

            except Exception as e:
                log_err(f"ChassisMaver periodic failed: {e}")

    def clear_odom_bias(self):
        """ reset odometry position """
        with self._data_lock:
            x, y, yaw = self._vehicle_position
            print(f"clear odom bias: {x}, {y}, {yaw}")
            # Convert (x, y, yaw) to 2D transformation matrix
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            self.__vehicle_origin_position = np.array([
                [cos_yaw, -sin_yaw, x],
                [sin_yaw,  cos_yaw, y],
                [0.0,      0.0,     1.0]
            ])

    # 底盘特有方法
    def get_base_state(self) -> int:
        """获取底盘状态"""
        return self._base_state

    def is_api_control_initialized(self) -> bool:
        """检查API控制是否已初始化"""
        return self._api_control_initialized

    def get_battery_info(self) -> Dict[str, Any]:
        """获取电池信息"""
        return {
            'voltage': self._battery_voltage,
            'thousandth': self._battery_thousandth,
            'charging': self._battery_charging
        }

    def get_vehicle_speed(self) -> Tuple[float, float, float]:
        """获取车辆速度 (m/s, m/s, rad/s)"""
        return self._vehicle_speed

    def get_vehicle_position(self) -> Tuple[float, float, float]:
        """ get vehicle position
        Odometry position, unit: m
        """
        with self._data_lock:
            self.__has_new = False
            
            # Convert current position to transformation matrix
            x, y, yaw = self._vehicle_position
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            current_matrix = np.array([
                [cos_yaw, -sin_yaw, x],
                [sin_yaw,  cos_yaw, y],
                [0.0,      0.0,     1.0]
            ])
            
            # Calculate relative transformation: current * inverse(origin)
            origin_inv = np.linalg.inv(self.__vehicle_origin_position)
            relative_matrix = origin_inv @ current_matrix
            
            # Extract position and orientation from relative matrix
            relative_x = relative_matrix[0, 2]
            relative_y = relative_matrix[1, 2]
            relative_yaw = np.arctan2(relative_matrix[1, 0], relative_matrix[0, 0])
            
            return (relative_x, relative_y, relative_yaw)

    def get_parking_stop_detail(self):
        """获取停车停止详情"""
        return self._parking_stop_detail

    def get_warning(self) -> Optional[int]:
        """获取警告信息"""
        return self._warning

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
        设置底盘指令
        
        Args:
            command_type: 指令类型
            values: 指令值列表
        """
        if self._simple_control_mode == True:
            raise NotImplementedError("motor_command not implemented for _simple_control_mode: True")
        elif self._simple_control_mode == None:
            self._simple_control_mode = False

        super().motor_command(command_type, values)
        self._last_command_time = time.time()

    def set_vehicle_speed(self, speed_x: float, speed_y: float, speed_z: float):
        """
        设置XYZ速度
        
        Args:
            speed_x: X方向速度 (m/s)
            speed_y: Y方向速度 (m/s)
            speed_z: Z方向角速度 (rad/s)
        """
        if self._simple_control_mode == False:
            raise NotImplementedError("set_vehicle_speed not implemented for _simple_control_mode: False")
        elif self._simple_control_mode == None:
            self._simple_control_mode = True

        with self._command_lock:
            self._target_velocity = (speed_x, speed_y, speed_z)
            self._last_command_time = time.time()

    # msg constructer
    # construct control message
    def _construct_wheel_control_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a control message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        motor_targets = self._construct_target_msg()
        base_command.motor_targets.CopyFrom(motor_targets)
        msg.base_command.CopyFrom(base_command)
        return msg

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

    def _construct_init_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a init message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        base_command.api_control_initialize = True
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
        """获取底盘状态摘要"""
        summary = super().get_status_summary()

        # 添加底盘特有信息
        chassis_summary = {
            'base_state': public_api_types_pb2.BaseState.Name(self._base_state),
            'api_control_initialized': self._api_control_initialized,
            'battery_info': self.get_battery_info(),
            'vehicle_speed': self._vehicle_speed,
            'vehicle_position': self._vehicle_position,
            'parking_stop_detail': self._parking_stop_detail,
            'warning': public_api_types_pb2.WarningCategory.Name(self._warning) if self._warning else None,
        }

        summary.update(chassis_summary)
        return summary

    def __str__(self) -> str:
        """字符串表示"""
        state_name = public_api_types_pb2.BaseState.Name(self._base_state)
        return f"{self.name}(State:{state_name}, Motors:{self.motor_count}, API:{self._api_control_initialized})"

    def __repr__(self) -> str:
        """详细字符串表示"""
        state_name = public_api_types_pb2.BaseState.Name(self._base_state)
        return f"ChassisMaver(motor_count={self.motor_count}, name='{self.name}', base_state={state_name})"
