#!/usr/bin/env python3
# -*- coding: utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import numpy as np


class DofType(Enum):
    """自由度类型"""
    SIX_AXIS = "six_axis"
    SEVEN_AXIS = "seven_axis"


@dataclass
class SixAxisData:
    """六轴数据"""
    motor_cnt: int = 6


@dataclass
class SevenAxisData:
    """七轴数据"""
    motor_cnt: int = 7


@dataclass
class JointParam:
    """关节参数"""
    joint_name: str
    joint_limit: List[
        float]  # [min_pos, max_pos, min_vel, max_vel, min_acc, max_acc]


@dataclass
class JointParams:
    """关节参数集合"""
    joints: List[JointParam]


@dataclass
class ArmConfig:
    """机械臂配置"""
    name: str
    dof_num: DofType
    arm_series: int
    motor_model: List[int]
    robot_config: JointParams


class ArmConfigManager:
    """机械臂配置管理器"""

    def __init__(self):
        self._configs: Dict[int, ArmConfig] = {}
        self._last_positions: Dict[int, List[float]] = {}  # 记录每个机械臂系列的上一次位置
        self._last_velocities: Dict[int, List[float]] = {}  # 记录每个机械臂系列的上一次速度
        self._load_default_configs()

    def _load_default_configs(self):
        """加载默认配置"""
        # 0x00 - saber750d_6dof (1轴)
        self._configs[0x00] = ArmConfig(
            name="saber750d_6dof",
            dof_num=DofType.SIX_AXIS,
            arm_series=0x00,
            motor_model=[0x80] * 1,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-3.15, 3.15, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x09 - saber750d_6dof (6轴)
        self._configs[0x09] = ArmConfig(
            name="saber750d_6dof",
            dof_num=DofType.SIX_AXIS,
            arm_series=0x09,
            motor_model=[0x80] * 6,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_2",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_3",
                    joint_limit=[-1.047, 3.14159265359, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_4",
                    joint_limit=[-2.792526, 2.792526, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_5",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_6",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x0A - saber750d_7dof (7轴)
        self._configs[0x0A] = ArmConfig(
            name="saber750d_7dof",
            dof_num=DofType.SEVEN_AXIS,
            arm_series=0x0A,
            motor_model=[0x80] * 7,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_2",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_3",
                    joint_limit=[-2.792526, 2.792526, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_4",
                    joint_limit=[-1.047, 3.14159265359, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_5",
                    joint_limit=[-2.792526, 2.792526, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_6",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_7",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x0B - saber750h_6dof (6轴)
        self._configs[0x0B] = ArmConfig(
            name="saber750h_6dof",
            dof_num=DofType.SIX_AXIS,
            arm_series=0x0B,
            motor_model=[0x80] * 6,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_2",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_3",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_4",
                    joint_limit=[-2.792526, 2.792526, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_5",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_6",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x0C - saber750h_7dof (7轴)
        self._configs[0x0C] = ArmConfig(
            name="saber750h_7dof",
            dof_num=DofType.SEVEN_AXIS,
            arm_series=0x0C,
            motor_model=[0x80] * 7,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_2",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_3",
                    joint_limit=[-2.792526, 2.792526, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_4",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_5",
                    joint_limit=[-2.792526, 2.792526, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_6",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_7",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x0E - saber_d6x (6轴)
        self._configs[0x0E] = ArmConfig(
            name="saber_d6x",
            dof_num=DofType.SIX_AXIS,
            arm_series=0x0E,
            motor_model=[0x80] * 6,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_2",
                           joint_limit=[-1.57, 2.094, -4.5, 4.5, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_3",
                    joint_limit=[-0.393, 3.14159265359, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_4",
                           joint_limit=[-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_5",
                           joint_limit=[-1.6, 1.6, -4.5, 4.5, -0.0, 0.0]),
                JointParam(joint_name="joint_6",
                           joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x10 - archer_d6y (6轴)
        self._configs[0x10] = ArmConfig(
            name="archer_d6y",
            dof_num=DofType.SIX_AXIS,
            arm_series=16,
            motor_model=[0x80] * 6,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-2.7, 3.1, -3.77, 3.77, -0.0, 0.0]),
                JointParam(joint_name="joint_2",
                           joint_limit=[-1.57, 2.094, -3.77, 3.77, -0.0, 0.0]),
                JointParam(
                    joint_name="joint_3",
                    joint_limit=[0.0, 3.14159265359, -3.77, 3.77, -0.0, 0.0]),
                JointParam(joint_name="joint_4",
                           joint_limit=[-1.56, 1.56, -12.56, 12.56, -0.0, 0.0]),
                JointParam(joint_name="joint_5",
                           joint_limit=[-1.56, 1.56, -12.56, 12.56, -0.0, 0.0]),
                JointParam(joint_name="joint_6",
                           joint_limit=[-1.57, 1.57, -12.56, 12.56, -0.0, 0.0])
            ]))

    def get_config(self, arm_series: int) -> Optional[ArmConfig]:
        """获取指定系列的机械臂配置"""
        return self._configs.get(arm_series)

    def get_all_configs(self) -> Dict[int, ArmConfig]:
        """获取所有配置"""
        return self._configs.copy()

    def add_config(self, arm_series: int, config: ArmConfig):
        """添加新的机械臂配置"""
        self._configs[arm_series] = config

    def remove_config(self, arm_series: int):
        """移除指定系列的配置"""
        if arm_series in self._configs:
            del self._configs[arm_series]

    def _validate_config(self, config: ArmConfig) -> bool:
        """
        验证配置的有效性
        
        Args:
            config: 要验证的配置
            
        Returns:
            bool: 配置是否有效
        """
        try:
            # 检查基本参数
            if not config.name or not config.robot_config.joints:
                return False

            # 检查参数限制合法性
            for joint in config.robot_config.joints:
                if len(joint.joint_limit) != 6:
                    return False

                min_pos, max_pos, min_vel, max_vel, min_acc, max_acc = joint.joint_limit

                if min_pos >= max_pos:
                    return False

                if min_vel >= max_vel:
                    return False

                # 检查加速度限制的合理性（允许相等，因为可能是0）
                if min_acc > max_acc:
                    return False

            # 检查电机模型数量与关节数量是否匹配
            if len(config.motor_model) != len(config.robot_config.joints):
                return False

            return True

        except Exception:
            return False

    def reload_from_dict(self, arm_series: int, config_data: dict) -> bool:
        """
        从字典数据重载机械臂配置
        
        Args:
            arm_series: 机械臂系列标识
            config_data: 配置数据字典
            
        Returns:
            bool: 重载是否成功
        """
        try:
            # 从字典创建配置对象
            new_config = self._create_config_from_dict(arm_series, config_data)

            existing_config = self.get_config(arm_series)
            if existing_config is None:
                # 如果不存在，直接添加新配置
                self.add_config(arm_series, new_config)
                return True

            # 验证新配置的关节数量是否与现有配置匹配
            existing_joint_count = len(existing_config.robot_config.joints)
            new_joint_count = len(new_config.robot_config.joints)

            if existing_joint_count != new_joint_count:
                raise ValueError(f"关节数量不匹配: 现有配置有 {existing_joint_count} 个关节，"
                                 f"新配置有 {new_joint_count} 个关节")

            # 验证新配置的基本参数
            if new_config.arm_series != arm_series:
                raise ValueError(
                    f"机械臂系列不匹配: 期望 {arm_series}，实际 {new_config.arm_series}")

            old_config = existing_config

            try:
                self._configs[arm_series] = new_config

                if not self._validate_config(new_config):
                    # 如果验证失败，回滚到旧配置
                    self._configs[arm_series] = old_config
                    return False

                return True

            except Exception as e:
                # 发生异常时回滚到旧配置
                self._configs[arm_series] = old_config
                raise e

        except Exception as e:
            print(f"从字典重载配置失败: {e}")
            return False

    def _create_config_from_dict(self, arm_series: int,
                                 config_data: dict) -> ArmConfig:
        """
        从字典数据创建配置对象
        
        Args:
            arm_series: 机械臂系列标识
            config_data: 配置数据字典
            
        Returns:
            ArmConfig: 创建的配置对象
        """
        # 解析自由度类型
        dof_type_str = config_data.get('dof_num', 'six_axis')
        if dof_type_str == 'six_axis':
            dof_num = DofType.SIX_AXIS
        elif dof_type_str == 'seven_axis':
            dof_num = DofType.SEVEN_AXIS
        else:
            raise ValueError(f"不支持的自由度类型: {dof_type_str}")

        # 创建关节参数
        joints = []
        for joint_data in config_data.get('joints', []):
            joint = JointParam(joint_name=joint_data['joint_name'],
                               joint_limit=joint_data['joint_limit'])
            joints.append(joint)

        # 创建配置对象
        config = ArmConfig(name=config_data.get(
            'name', f'arm_series_{arm_series:02X}'),
                           dof_num=dof_num,
                           arm_series=arm_series,
                           motor_model=config_data.get('motor_model',
                                                       [0x80] * len(joints)),
                           robot_config=JointParams(joints=joints))

        return config

    def get_motor_count(self, arm_series: int) -> Optional[int]:
        """获取指定系列的电机数量"""
        config = self.get_config(arm_series)
        if config:
            if config.dof_num == DofType.SIX_AXIS:
                return 6
            elif config.dof_num == DofType.SEVEN_AXIS:
                return 7
        return None

    def get_joint_limits(self, arm_series: int) -> Optional[List[List[float]]]:
        """获取指定系列的关节限制"""
        config = self.get_config(arm_series)
        if config:
            return [joint.joint_limit for joint in config.robot_config.joints]
        return None

    def validate_joint_positions(self,
                                 arm_series: int,
                                 positions: List[float],
                                 dt: float = 0.001) -> List[float]:
        """
        验证关节位置是否在限制范围内，并返回修正后的位置列表
        
        Args:
            arm_series: 机械臂系列
            positions: 目标位置列表 (rad)
            dt: 时间步长 (s)，用于速度限制计算
            
        Returns:
            List[float]: 修正后的位置列表
        """
        config = self.get_config(arm_series)
        if not config or len(positions) != len(config.robot_config.joints):
            return positions.copy()  # 如果配置无效，返回原位置

        validated_positions = []
        last_positions = self._last_positions.get(arm_series, None)

        for i, (position,
                joint) in enumerate(zip(positions,
                                        config.robot_config.joints)):
            min_pos, max_pos = joint.joint_limit[0], joint.joint_limit[1]
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]

            # 首先处理位置限制
            if position < min_pos:
                validated_position = min_pos
            elif position > max_pos:
                validated_position = max_pos
            else:
                validated_position = position

            # 如果有上一次位置记录，进行速度限制检查
            if last_positions is not None and i < len(last_positions):
                last_position = last_positions[i]

                # 计算当前速度 (rad/s)
                current_velocity = (position - last_position) / dt

                # 检查速度是否超出限制
                if current_velocity < min_vel or current_velocity > max_vel:
                    # 速度超出限制，根据速度限制插值计算合理位置
                    if current_velocity > max_vel:
                        # 速度过大，限制在最大速度
                        max_displacement = max_vel * dt
                        validated_position = last_position + max_displacement
                    elif current_velocity < min_vel:
                        # 速度过小，限制在最小速度
                        min_displacement = min_vel * dt
                        validated_position = last_position + min_displacement

                    # 再次检查位置限制
                    if validated_position < min_pos:
                        validated_position = min_pos
                    elif validated_position > max_pos:
                        validated_position = max_pos

            validated_positions.append(validated_position)

        # 更新记录的上一次位置
        self._last_positions[arm_series] = validated_positions.copy()

        return validated_positions

    def validate_joint_velocities(self,
                                  arm_series: int,
                                  velocities: List[float],
                                  dt: float = 0.001) -> List[float]:
        """
        验证关节速度是否在限制范围内，并返回修正后的速度列表
        
        Args:
            arm_series: 机械臂系列
            velocities: 目标速度列表 (rad/s)
            dt: 时间步长 (s)，用于加速度限制计算
            
        Returns:
            List[float]: 修正后的速度列表
        """
        config = self.get_config(arm_series)
        if not config or len(velocities) != len(config.robot_config.joints):
            return velocities.copy()  # 如果配置无效，返回原速度

        validated_velocities = []
        last_velocities = self._last_velocities.get(arm_series, None)

        for i, (velocity,
                joint) in enumerate(zip(velocities,
                                        config.robot_config.joints)):
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]
            min_acc, max_acc = joint.joint_limit[4], joint.joint_limit[5]

            # 首先处理速度限制
            if velocity < min_vel:
                validated_velocity = min_vel
            elif velocity > max_vel:
                validated_velocity = max_vel
            else:
                validated_velocity = velocity

            # 如果有上一次速度记录，进行加速度限制检查
            if last_velocities is not None and i < len(last_velocities):
                last_velocity = last_velocities[i]

                # 计算当前加速度 (rad/s²)
                current_acceleration = (validated_velocity -
                                        last_velocity) / dt

                # 检查加速度是否超出限制
                if current_acceleration < min_acc or current_acceleration > max_acc:
                    # 加速度超出限制，根据加速度限制计算合理速度
                    if current_acceleration > max_acc:
                        # 加速度过大，限制在最大加速度
                        max_velocity_change = max_acc * dt
                        validated_velocity = last_velocity + max_velocity_change
                        # 限制速度不超过最大速度
                        validated_velocity = min(validated_velocity, max_vel)
                    elif current_acceleration < min_acc:
                        # 加速度过小，限制在最小加速度
                        min_velocity_change = min_acc * dt
                        validated_velocity = last_velocity + min_velocity_change
                        # 限制速度不小于最小速度
                        validated_velocity = max(validated_velocity, min_vel)

            validated_velocities.append(validated_velocity)

        # 更新记录的上一次速度
        self._last_velocities[arm_series] = validated_velocities.copy()

        return validated_velocities

    def get_joint_names(self, arm_series: int) -> Optional[List[str]]:
        """获取指定系列的关节名称"""
        config = self.get_config(arm_series)
        if config:
            return [joint.joint_name for joint in config.robot_config.joints]
        return None

    def set_initial_positions(self, arm_series: int, positions: List[float]):
        """
        设置机械臂的初始位置，用于速度限制计算
        
        Args:
            arm_series: 机械臂系列
            positions: 初始位置列表 (rad)
        """
        config = self.get_config(arm_series)
        if config and len(positions) == len(config.robot_config.joints):
            self._last_positions[arm_series] = positions.copy()

    def set_initial_velocities(self, arm_series: int, velocities: List[float]):
        """
        设置机械臂的初始速度，用于加速度限制计算
        
        Args:
            arm_series: 机械臂系列
            velocities: 初始速度列表 (rad/s)
        """
        config = self.get_config(arm_series)
        if config and len(velocities) == len(config.robot_config.joints):
            self._last_velocities[arm_series] = velocities.copy()

    def clear_position_history(self, arm_series: int):
        """
        清除指定机械臂系列的位置历史记录
        
        Args:
            arm_series: 机械臂系列
        """
        if arm_series in self._last_positions:
            del self._last_positions[arm_series]

    def clear_velocity_history(self, arm_series: int):
        """
        清除指定机械臂系列的速度历史记录
        
        Args:
            arm_series: 机械臂系列
        """
        if arm_series in self._last_velocities:
            del self._last_velocities[arm_series]

    def clear_motion_history(self, arm_series: int):
        """
        清除指定机械臂系列的所有运动历史记录（位置和速度）
        
        Args:
            arm_series: 机械臂系列
        """
        self.clear_position_history(arm_series)
        self.clear_velocity_history(arm_series)

    def get_last_positions(self, arm_series: int) -> Optional[List[float]]:
        """
        获取指定机械臂系列的上一次位置记录
        
        Args:
            arm_series: 机械臂系列
            
        Returns:
            List[float]: 上一次位置列表，如果没有记录则返回None
        """
        return self._last_positions.get(arm_series, None)

    def set_last_positions(self, arm_series: int, positions: List[float]):
        """
        设置指定机械臂系列的上一次位置记录
        
        Args:
            arm_series: 机械臂系列
            positions: 位置列表
        """
        self._last_positions[arm_series] = positions.copy()

    def get_last_velocities(self, arm_series: int) -> Optional[List[float]]:
        """
        获取指定机械臂系列的上一次速度记录
        
        Args:
            arm_series: 机械臂系列
            
        Returns:
            List[float]: 上一次速度列表，如果没有记录则返回None
        """
        return self._last_velocities.get(arm_series, None)


# 全局配置管理器实例
arm_config_manager = ArmConfigManager()

# 便捷函数
def set_arm_initial_positions(arm_series: int, positions: List[float]):
    """设置机械臂的初始位置，用于速度限制计算"""
    return arm_config_manager.set_initial_positions(arm_series, positions)


def set_arm_initial_velocities(arm_series: int, velocities: List[float]):
    """设置机械臂的初始速度，用于加速度限制计算"""
    return arm_config_manager.set_initial_velocities(arm_series, velocities)


def clear_arm_position_history(arm_series: int):
    """清除指定机械臂系列的位置历史记录"""
    return arm_config_manager.clear_position_history(arm_series)


def clear_arm_velocity_history(arm_series: int):
    """清除指定机械臂系列的速度历史记录"""
    return arm_config_manager.clear_velocity_history(arm_series)


def clear_arm_motion_history(arm_series: int):
    """清除指定机械臂系列的所有运动历史记录（位置和速度）"""
    return arm_config_manager.clear_motion_history(arm_series)


def get_arm_last_positions(arm_series: int) -> Optional[List[float]]:
    """获取指定机械臂系列的上一次位置记录"""
    return arm_config_manager.get_last_positions(arm_series)


def get_arm_last_velocities(arm_series: int) -> Optional[List[float]]:
    """获取指定机械臂系列的上一次速度记录"""
    return arm_config_manager.get_last_velocities(arm_series)


def load_default_arm_config() -> Dict[int, ArmConfig]:
    """加载默认机械臂配置（类似于 Rust 的 load_default_arm_config 函数）"""
    return arm_config_manager.get_all_configs()


def get_arm_config(arm_series: int) -> Optional[ArmConfig]:
    """获取指定系列的机械臂配置"""
    return arm_config_manager.get_config(arm_series)


def add_arm_config(arm_series: int, config: ArmConfig):
    """添加新的机械臂配置"""
    arm_config_manager.add_config(arm_series, config)
