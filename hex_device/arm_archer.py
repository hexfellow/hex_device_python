#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import pprint
import time
from typing import Optional, List
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base import DeviceBase
from .motor_base import MotorBase, MotorError, MotorCommand, CommandType
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .generated.public_api_types_pb2 import (ArmStatus)
from .arm_config import get_arm_config, ArmConfig, arm_config_manager
from copy import deepcopy


class ArmArcher(DeviceBase, MotorBase):
    """
    ArmArcher类

    继承自DeviceBase和MotorBase，主要实现对ArmArcher的控制

    支持的机器人类型:
    - RtArmSaberD6X: 自定义PCW车辆
    - RtArmSaberD7X: PCW车辆
    """

    # 支持的机器人类型列表
    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtArmArcherD6Y,
        public_api_types_pb2.RobotType.RtArmSaberD6X,
        public_api_types_pb2.RobotType.RtArmSaberD7X,
    ]

    # 机械臂系列到机器人类型的映射
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
        初始化底盘Maver
        
        Args:
            motor_count: 电机数量
            robot_type: 机械臂系列
            name: 设备名称
            control_hz: 控制频率
            send_message_callback: 发送消息的回调函数，用于发送下行消息
        """
        DeviceBase.__init__(self, name, send_message_callback)
        MotorBase.__init__(self, motor_count, name)

        self.name = name or "ArmArcher"
        self._control_hz = control_hz
        self._arm_series = robot_type

        # arm status
        self._arm_state = public_api_types_pb2.ArmState.AsParked
        self._api_control_initialized = False
        self._calibrated = False
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

        # 控制相关
        self._last_command_time = None
        self._command_timeout = 0.3  # 300ms
        self.__last_warning_time = time.perf_counter()  # last log warning time

    def _set_robot_type(self, robot_type):
        """
        设置机器人类型
        
        Args:
            robot_type: 机器人类型
        """
        if robot_type in self.SUPPORTED_ROBOT_TYPES:
            self.robot_type = robot_type
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")

    @classmethod
    def _supports_robot_type(cls, robot_type):
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
        初始化机械臂
        
        Returns:
            bool: 是否成功初始化
        """
        try:
            msg = self._construct_init_message()
            await self._send_message(msg)
            msg = self._construct_calibrate_message()
            await self._send_message(msg)
            return True
        except Exception as e:
            log_err(f"ArmArcher初始化失败: {e}")
            return False

    def _update(self, api_up_data) -> bool:
        """
        更新机械臂数据
        
        Args:
            api_up_data: 从API接收的上行数据 (APIUp)
            
        Returns:
            bool: 是否成功更新
        """
        try:
            if not api_up_data.HasField('arm_status'):
                return False

            arm_status = api_up_data.arm_status

            # 更新机械臂状态
            self._arm_state = arm_status.state
            self._api_control_initialized = arm_status.api_control_initialized
            self._calibrated = arm_status.calibrated

            if arm_status.HasField('parking_stop_detail'):
                self._parking_stop_detail = arm_status.parking_stop_detail
            else:
                self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()

            # 更新电机数据
            self._update_motor_data_from_arm_status(arm_status)
            self.set_has_new_data()
            return True
        except Exception as e:
            log_err(f"ArmArcher更新数据失败: {e}")
            return False

    def _update_motor_data_from_arm_status(self, arm_status: ArmStatus):
        motor_status_list = arm_status.motor_status

        if len(motor_status_list) != self.motor_count:
            log_warn(
                f"警告: 电机数量不匹配，期望{self.motor_count}，实际{len(motor_status_list)}")
            return

        # 解析电机数据
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
                error_code = motor_status.error[0].value
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
        周期性执行函数
        
        执行机械臂的周期性任务，包括：
        - 状态检查
        - 命令超时检查
        - 安全监控
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
                        log_err(f"警告: 电机{i}出现错误")

                # perpare sending message
                if self._api_control_initialized == False:
                    msg = self._construct_init_message()
                    await self._send_message(msg)
                elif self._calibrated == False:
                    # If there is anything that requires special action, modify this calibrate sending logic.
                    msg = self._construct_calibrate_message()
                    await self._send_message(msg)
                else:
                    # command timeout
                    if self._last_command_time is None:
                        msg = self._construct_init_message()
                        await self._send_message(msg)
                    elif (start_time -
                          self._last_command_time) > self._command_timeout:
                        msg = self._construct_custom_motor_msg(
                            CommandType.BRAKE, [0.0] * self.motor_count)
                        await self._send_message(msg)
                    else:
                        # normal command
                        try:
                            msg = self._construct_joint_command_msg()
                            await self._send_message(msg)
                        except Exception as e:
                            log_err(f"ArmArcher构造关节命令消息失败: {e}")
                            continue

            except Exception as e:
                log_err(f"ArmArcher周期性任务异常: {e}")
                continue

    # 机械臂特有方法
    def motor_command(self, command_type: CommandType, values: List[float]):
        """
        设置电机指令
        """
        period = float(1.0 / self._control_hz)
        if command_type == CommandType.POSITION:
            values = self.validate_joint_positions(values, dt=period)
        elif command_type == CommandType.SPEED:
            values = self.validate_joint_velocities(values, dt=period)
        super().motor_command(command_type, values)
        self._last_command_time = time.perf_counter()

    def _construct_joint_command_msg(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        motor_targets = self._construct_target_motor_msg()
        arm_command.motor_targets.CopyFrom(motor_targets)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def get_parking_stop_detail(
            self) -> public_api_types_pb2.ParkingStopDetail:
        """获取停车停止详情"""
        return deepcopy(self._parking_stop_detail)

    # msg constructer
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

    # 配置相关方法
    def get_arm_config(self) -> Optional[ArmConfig]:
        """获取当前机械臂的配置"""
        return deepcopy(get_arm_config(self._arm_series))

    def get_joint_limits(self) -> Optional[List[List[float]]]:
        """获取关节限制"""
        return deepcopy(arm_config_manager.get_joint_limits(self._arm_series))

    def validate_joint_positions(self,
                                 positions: List[float],
                                 dt: float = 0.002) -> List[float]:
        """
        验证关节位置是否在限制范围内，并返回修正后的位置列表
        
        Args:
            positions: 目标位置列表 (rad)
            dt: 时间步长 (s)，用于速度限制计算
            
        Returns:
            List[float]: 修正后的位置列表
        """
        last_positions = arm_config_manager.get_last_positions(self._arm_series)
        
        # 如果没有上一次位置记录，使用当前电机位置作为初始位置
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
        验证关节速度是否在限制范围内，并返回修正后的速度列表
        """
        return arm_config_manager.validate_joint_velocities(
            self._arm_series, velocities, dt)

    def get_joint_names(self) -> Optional[List[str]]:
        """获取关节名称"""
        return deepcopy(arm_config_manager.get_joint_names(self._arm_series))

    def get_expected_motor_count(self) -> Optional[int]:
        """获取期望的电机数量"""
        return deepcopy(arm_config_manager.get_motor_count(self._arm_series))

    def check_motor_count_match(self) -> bool:
        """检查电机数量是否匹配配置"""
        expected_count = self.get_expected_motor_count()
        if expected_count is None:
            return False
        return self.motor_count == expected_count

    def get_arm_series(self) -> int:
        """获取机械臂系列"""
        return deepcopy(self._arm_series)

    def get_arm_name(self) -> Optional[str]:
        """获取机械臂名称"""
        config = self.get_arm_config()
        return deepcopy(config.name) if config else None

    def reload_arm_config_from_dict(self, config_data: dict) -> bool:
        """
        从字典数据重载当前机械臂的配置参数
        
        Args:
            config_data: 配置数据字典
            
        Returns:
            bool: 重载是否成功
        """
        try:
            success = arm_config_manager.reload_from_dict(
                self._arm_series, config_data)
            if success:
                print(f"机械臂配置从字典重载成功: {config_data.get('name', 'unknown')}")
            else:
                print(f"机械臂配置从字典重载失败: {config_data.get('name', 'unknown')}")
            return success
        except Exception as e:
            print(f"机械臂配置从字典重载异常: {e}")
            return False

    def set_initial_positions(self, positions: List[float]):
        """
        设置机械臂的初始位置，用于速度限制计算
        
        Args:
            positions: 初始位置列表 (rad)
        """
        arm_config_manager.set_initial_positions(self._arm_series, positions)

    def set_initial_velocities(self, velocities: List[float]):
        """
        设置机械臂的初始速度，用于加速度限制计算
        
        Args:
            velocities: 初始速度列表 (rad/s)
        """
        arm_config_manager.set_initial_velocities(self._arm_series, velocities)

    def get_last_positions(self) -> Optional[List[float]]:
        """
        获取上一次位置记录
        
        Returns:
            List[float]: 上一次位置列表，如果没有记录则返回None
        """
        return arm_config_manager.get_last_positions(self._arm_series)

    def get_last_velocities(self) -> Optional[List[float]]:
        """
        获取上一次速度记录
        
        Returns:
            List[float]: 上一次速度列表，如果没有记录则返回None
        """
        return arm_config_manager.get_last_velocities(self._arm_series)

    def clear_position_history(self):
        """清除位置历史记录"""
        arm_config_manager.clear_position_history(self._arm_series)

    def clear_velocity_history(self):
        """清除速度历史记录"""
        arm_config_manager.clear_velocity_history(self._arm_series)

    def clear_motion_history(self):
        """清除所有运动历史记录（位置和速度）"""
        arm_config_manager.clear_motion_history(self._arm_series)
