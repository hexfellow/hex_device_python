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
from .common_utils import log_err
import numpy as np


class DofType(Enum):
    """Degree of freedom type"""
    SIX_AXIS = "six_axis"
    SEVEN_AXIS = "seven_axis"


@dataclass
class SixAxisData:
    """Six-axis data"""
    motor_cnt: int = 6


@dataclass
class SevenAxisData:
    """Seven-axis data"""
    motor_cnt: int = 7


@dataclass
class JointParam:
    """Joint parameters"""
    joint_name: str
    joint_limit: List[
        float]  # [min_pos, max_pos, min_vel, max_vel, min_acc, max_acc]


@dataclass
class JointParams:
    """Collection of joint parameters"""
    joints: List[JointParam]


@dataclass
class ArmConfig:
    """Robotic arm configuration"""
    name: str
    dof_num: DofType
    arm_series: int
    motor_model: List[int]
    robot_config: JointParams


class ArmConfigManager:
    """Robotic arm configuration manager"""

    def __init__(self):
        self._configs: Dict[int, ArmConfig] = {}
        self._last_positions: Dict[int, List[float]] = {}  # Record last position
        self._last_velocities: Dict[int, List[float]] = {}  # Record last velocity
        self._load_default_configs()

    def _load_default_configs(self):
        """Load default configurations"""
        # 0x00 - saber750d_6dof (1-axis)
        self._configs[0x00] = ArmConfig(
            name="saber750d_6dof",
            dof_num=DofType.SIX_AXIS,
            arm_series=0x00,
            motor_model=[0x80] * 1,
            robot_config=JointParams(joints=[
                JointParam(joint_name="joint_1",
                           joint_limit=[-3.15, 3.15, -4.5, 4.5, -0.0, 0.0])
            ]))

        # 0x09 - saber750d_6dof (6-axis)
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

        # 0x0A - saber750d_7dof (7-axis)
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

        # 0x0B - saber750h_6dof (6-axis)
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

        # 0x0C - saber750h_7dof (7-axis)
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

        # 0x0E - saber_d6x (6-axis)
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

        # 0x10 - archer_d6y (6-axis)
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
        """Get robotic arm configuration for specified series"""
        return self._configs.get(arm_series)

    def get_all_configs(self) -> Dict[int, ArmConfig]:
        """Get all configurations"""
        return self._configs.copy()

    def add_config(self, arm_series: int, config: ArmConfig):
        """Add new robotic arm configuration"""
        self._configs[arm_series] = config

    def remove_config(self, arm_series: int):
        """Remove configuration for specified series"""
        if arm_series in self._configs:
            del self._configs[arm_series]

    def _validate_config(self, config: ArmConfig) -> bool:
        """
        Validate configuration validity
        
        Args:
            config: Configuration to validate
            
        Returns:
            bool: Whether configuration is valid
        """
        try:
            if not config.name or not config.robot_config.joints:
                return False

            for joint in config.robot_config.joints:
                if len(joint.joint_limit) != 6:
                    return False

                min_pos, max_pos, min_vel, max_vel, min_acc, max_acc = joint.joint_limit

                if min_pos >= max_pos:
                    return False

                if min_vel >= max_vel:
                    return False

                # Check acceleration limit reasonableness (equality allowed, as it may be 0)
                if min_acc > max_acc:
                    return False

            if len(config.motor_model) != len(config.robot_config.joints):
                return False

            return True

        except Exception:
            return False

    def reload_from_dict(self, arm_series: int, config_data: dict) -> bool:
        """
        Reload robotic arm configuration from dictionary data
        
        Args:
            arm_series: Robotic arm series identifier
            config_data: Configuration data dictionary
            
        Returns:
            bool: Whether reload was successful
        """
        try:
            new_config = self._create_config_from_dict(arm_series, config_data)

            existing_config = self.get_config(arm_series)
            if existing_config is None:
                # If it doesn't exist, directly add new configuration
                self.add_config(arm_series, new_config)
                return True

            # Validate whether new configuration joint count matches existing configuration
            existing_joint_count = len(existing_config.robot_config.joints)
            new_joint_count = len(new_config.robot_config.joints)

            if existing_joint_count != new_joint_count:
                raise ValueError(f"Joint count mismatch: existing configuration has {existing_joint_count} joints, "
                                 f"new configuration has {new_joint_count} joints")

            if new_config.arm_series != arm_series:
                raise ValueError(
                    f"Robotic arm series mismatch: expected {arm_series}, actual {new_config.arm_series}")

            old_config = existing_config

            try:
                self._configs[arm_series] = new_config

                if not self._validate_config(new_config):
                    # If validation fails, rollback to old configuration
                    self._configs[arm_series] = old_config
                    return False

                return True

            except Exception as e:
                # Rollback to old configuration when exception occurs
                self._configs[arm_series] = old_config
                raise e

        except Exception as e:
            log_err(f"Failed to reload configuration from dictionary: {e}")
            return False

    def _create_config_from_dict(self, arm_series: int,
                                 config_data: dict) -> ArmConfig:
        """
        Create configuration object from dictionary data
        
        Args:
            arm_series: Robotic arm series identifier
            config_data: Configuration data dictionary
            
        Returns:
            ArmConfig: Created configuration object
        """
        dof_type_str = config_data.get('dof_num', 'six_axis')
        if dof_type_str == 'six_axis':
            dof_num = DofType.SIX_AXIS
        elif dof_type_str == 'seven_axis':
            dof_num = DofType.SEVEN_AXIS
        else:
            raise ValueError(f"Unsupported degree of freedom type: {dof_type_str}")

        joints = []
        for joint_data in config_data.get('joints', []):
            joint = JointParam(joint_name=joint_data['joint_name'],
                               joint_limit=joint_data['joint_limit'])
            joints.append(joint)

        config = ArmConfig(name=config_data.get(
            'name', f'arm_series_{arm_series:02X}'),
                           dof_num=dof_num,
                           arm_series=arm_series,
                           motor_model=config_data.get('motor_model',
                                                       [0x80] * len(joints)),
                           robot_config=JointParams(joints=joints))

        return config

    def get_motor_count(self, arm_series: int) -> Optional[int]:
        """Get motor count for specified series"""
        config = self.get_config(arm_series)
        if config:
            if config.dof_num == DofType.SIX_AXIS:
                return 6
            elif config.dof_num == DofType.SEVEN_AXIS:
                return 7
        return None

    def get_joint_limits(self, arm_series: int) -> Optional[List[List[float]]]:
        """Get joint limits for specified series"""
        config = self.get_config(arm_series)
        if config:
            return [joint.joint_limit for joint in config.robot_config.joints]
        return None

    def validate_joint_positions(self,
                                 arm_series: int,
                                 positions: List[float],
                                 dt: float = 0.001) -> List[float]:
        """
        Validate whether joint positions are within limit range and return corrected position list
        
        Args:
            arm_series: Robotic arm series
            positions: Target position list (rad)
            dt: Time step (s), used for velocity limit calculation
            
        Returns:
            List[float]: Corrected position list
        """
        config = self.get_config(arm_series)
        if not config or len(positions) != len(config.robot_config.joints):
            return positions.copy()  # If configuration is invalid, return original positions

        validated_positions = []
        last_positions = self._last_positions.get(arm_series, None)

        for i, (position,
                joint) in enumerate(zip(positions,
                                        config.robot_config.joints)):
            min_pos, max_pos = joint.joint_limit[0], joint.joint_limit[1]
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]

            # First handle position limits
            if position < min_pos:
                validated_position = min_pos
            elif position > max_pos:
                validated_position = max_pos
            else:
                validated_position = position

            # If there is last position record, perform velocity limit check
            if last_positions is not None and i < len(last_positions):
                last_position = last_positions[i]

                # Calculate current velocity (rad/s)
                current_velocity = (position - last_position) / dt

                # Check if velocity exceeds limits
                if current_velocity < min_vel or current_velocity > max_vel:
                    # Velocity exceeds limits
                    if current_velocity > max_vel:
                        max_displacement = max_vel * dt
                        validated_position = last_position + max_displacement
                    elif current_velocity < min_vel:
                        # Velocity too small, limit to minimum velocity
                        min_displacement = min_vel * dt
                        validated_position = last_position + min_displacement

                    # Check position limits again
                    if validated_position < min_pos:
                        validated_position = min_pos
                    elif validated_position > max_pos:
                        validated_position = max_pos

            validated_positions.append(validated_position)

        # Update recorded last position
        self._last_positions[arm_series] = validated_positions.copy()

        return validated_positions

    def validate_joint_velocities(self,
                                  arm_series: int,
                                  velocities: List[float],
                                  dt: float = 0.001) -> List[float]:
        """
        Validate whether joint velocities are within limit range and return corrected velocity list
        
        Args:
            arm_series: Robotic arm series
            velocities: Target velocity list (rad/s)
            dt: Time step (s), used for acceleration limit calculation
            
        Returns:
            List[float]: Corrected velocity list
        """
        config = self.get_config(arm_series)
        if not config or len(velocities) != len(config.robot_config.joints):
            return velocities.copy()  # If configuration is invalid, return original velocities

        validated_velocities = []
        last_velocities = self._last_velocities.get(arm_series, None)

        for i, (velocity,
                joint) in enumerate(zip(velocities,
                                        config.robot_config.joints)):
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]
            min_acc, max_acc = joint.joint_limit[4], joint.joint_limit[5]

            # First handle velocity limits
            if velocity < min_vel:
                validated_velocity = min_vel
            elif velocity > max_vel:
                validated_velocity = max_vel
            else:
                validated_velocity = velocity

            # If there is last velocity record, perform acceleration limit check
            if last_velocities is not None and i < len(last_velocities):
                last_velocity = last_velocities[i]

                # Calculate current acceleration (rad/s²)
                current_acceleration = (validated_velocity -
                                        last_velocity) / dt

                # Check if acceleration exceeds limits
                if current_acceleration < min_acc or current_acceleration > max_acc:
                    if current_acceleration > max_acc:
                        max_velocity_change = max_acc * dt
                        validated_velocity = last_velocity + max_velocity_change
                        validated_velocity = min(validated_velocity, max_vel)
                    elif current_acceleration < min_acc:
                        min_velocity_change = min_acc * dt
                        validated_velocity = last_velocity + min_velocity_change
                        validated_velocity = max(validated_velocity, min_vel)

            validated_velocities.append(validated_velocity)

        # Update recorded last velocity
        self._last_velocities[arm_series] = validated_velocities.copy()

        return validated_velocities

    def get_joint_names(self, arm_series: int) -> Optional[List[str]]:
        """Get joint names for specified series"""
        config = self.get_config(arm_series)
        if config:
            return [joint.joint_name for joint in config.robot_config.joints]
        return None

    def set_initial_positions(self, arm_series: int, positions: List[float]):
        """
        Set initial positions of robotic arm, used for velocity limit calculation
        
        Args:
            arm_series: Robotic arm series
            positions: Initial position list (rad)
        """
        config = self.get_config(arm_series)
        if config and len(positions) == len(config.robot_config.joints):
            self._last_positions[arm_series] = positions.copy()

    def set_initial_velocities(self, arm_series: int, velocities: List[float]):
        """
        Set initial velocities of robotic arm, used for acceleration limit calculation
        
        Args:
            arm_series: Robotic arm series
            velocities: Initial velocity list (rad/s)
        """
        config = self.get_config(arm_series)
        if config and len(velocities) == len(config.robot_config.joints):
            self._last_velocities[arm_series] = velocities.copy()

    def clear_position_history(self, arm_series: int):
        """
        Clear position history records for specified robotic arm series
        
        Args:
            arm_series: Robotic arm series
        """
        if arm_series in self._last_positions:
            del self._last_positions[arm_series]

    def clear_velocity_history(self, arm_series: int):
        """
        Clear velocity history records for specified robotic arm series
        
        Args:
            arm_series: Robotic arm series
        """
        if arm_series in self._last_velocities:
            del self._last_velocities[arm_series]

    def clear_motion_history(self, arm_series: int):
        """
        Clear all motion history records for specified robotic arm series (position and velocity)
        
        Args:
            arm_series: Robotic arm series
        """
        self.clear_position_history(arm_series)
        self.clear_velocity_history(arm_series)

    def get_last_positions(self, arm_series: int) -> Optional[List[float]]:
        """
        Get last position record for specified robotic arm series
        
        Args:
            arm_series: Robotic arm series
            
        Returns:
            List[float]: Last position list, returns None if no record exists
        """
        return self._last_positions.get(arm_series, None)

    def set_last_positions(self, arm_series: int, positions: List[float]):
        """
        Set last position record for specified robotic arm series
        
        Args:
            arm_series: Robotic arm series
            positions: Position list
        """
        self._last_positions[arm_series] = positions.copy()

    def get_last_velocities(self, arm_series: int) -> Optional[List[float]]:
        """
        Get last velocity record for specified robotic arm series
        
        Args:
            arm_series: Robotic arm series
            
        Returns:
            List[float]: Last velocity list, returns None if no record exists
        """
        return self._last_velocities.get(arm_series, None)


# Global configuration manager instance
arm_config_manager = ArmConfigManager()

def set_arm_initial_positions(arm_series: int, positions: List[float]):
    """Set initial positions of robotic arm, used for velocity limit calculation"""
    return arm_config_manager.set_initial_positions(arm_series, positions)


def set_arm_initial_velocities(arm_series: int, velocities: List[float]):
    """Set initial velocities of robotic arm, used for acceleration limit calculation"""
    return arm_config_manager.set_initial_velocities(arm_series, velocities)


def clear_arm_position_history(arm_series: int):
    """Clear position history records for specified robotic arm series"""
    return arm_config_manager.clear_position_history(arm_series)


def clear_arm_velocity_history(arm_series: int):
    """Clear velocity history records for specified robotic arm series"""
    return arm_config_manager.clear_velocity_history(arm_series)


def clear_arm_motion_history(arm_series: int):
    """Clear all motion history records for specified robotic arm series (position and velocity)"""
    return arm_config_manager.clear_motion_history(arm_series)


def get_arm_last_positions(arm_series: int) -> Optional[List[float]]:
    """Get last position record for specified robotic arm series"""
    return arm_config_manager.get_last_positions(arm_series)


def get_arm_last_velocities(arm_series: int) -> Optional[List[float]]:
    """Get last velocity record for specified robotic arm series"""
    return arm_config_manager.get_last_velocities(arm_series)


def load_default_arm_config() -> Dict[int, ArmConfig]:
    """Load default robotic arm configuration (similar to Rust's load_default_arm_config function)"""
    return arm_config_manager.get_all_configs()


def get_arm_config(arm_series: int) -> Optional[ArmConfig]:
    """Get robotic arm configuration for specified series"""
    return arm_config_manager.get_config(arm_series)


def add_arm_config(arm_series: int, config: ArmConfig):
    """Add new robotic arm configuration"""
    arm_config_manager.add_config(arm_series, config)
