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
from typing import Optional, Tuple, List, Dict, Any, Union
from .common_utils import delay, log_common, log_info, log_warn, log_err
from .device_base import DeviceBase
from .motor_base import MitMotorCommand, MotorBase, MotorError, MotorCommand, CommandType, Timestamp
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .arm_config import get_arm_config, ArmConfig, arm_config_manager


class Arm(DeviceBase, MotorBase):
    """
    Arm class

    Inherits from DeviceBase and MotorBase, mainly implements control of Arm

    """

    SUPPORTED_ROBOT_TYPES = [
        public_api_types_pb2.RobotType.RtArmSaberD6x,
        public_api_types_pb2.RobotType.RtArmSaberD7x,
        public_api_types_pb2.RobotType.RtArmArcherD6Y_P1,
        public_api_types_pb2.RobotType.RtArmArcherY6L_V1,
        public_api_types_pb2.RobotType.RtArmArcherY6_H1,
        public_api_types_pb2.RobotType.RtArmFireflyY6_H1,
        public_api_types_pb2.RobotType.RtHelloArcherY6_H1,
        public_api_types_pb2.RobotType.RtHelloFireflyY6_H1,
    ]

    ARM_SERIES_TO_ROBOT_TYPE = {
        14: public_api_types_pb2.RobotType.RtArmSaberD6x,
        15: public_api_types_pb2.RobotType.RtArmSaberD7x,
        16: public_api_types_pb2.RobotType.RtArmArcherD6Y_P1,
        17: public_api_types_pb2.RobotType.RtArmArcherY6L_V1,
        25: public_api_types_pb2.RobotType.RtArmArcherY6_H1,
        27: public_api_types_pb2.RobotType.RtArmFireflyY6_H1,
        26: public_api_types_pb2.RobotType.RtHelloArcherY6_H1,
        27: public_api_types_pb2.RobotType.RtHelloFireflyY6_H1,
    }

    def __init__(self,
                 robot_type,
                 motor_count,
                 name: str = "Arm",
                 control_hz: int = 500,
                 send_message_callback=None):
        """
        Initialize chassis Maver
        
        Args:
            motor_count: Number of motors
            robot_type: Robotic arm series
            name: Device name
            control_hz: Control frequency
            send_message_callback: Callback function for sending messages, used to send downstream messages
        """
        DeviceBase.__init__(self, name, send_message_callback)

        # Convert function for old revert function
        if robot_type in [public_api_types_pb2.RobotType.RtArmSaberD6x,
                            public_api_types_pb2.RobotType.RtArmSaberD7x,
                            public_api_types_pb2.RobotType.RtArmArcherD6Y_P1,
                            public_api_types_pb2.RobotType.RtArmArcherY6L_V1]:
            MotorBase.__init__(self, motor_count, name, 
            convert_positions_to_rad_func=self.convert_positions_to_rad_func, 
            convert_rad_to_positions_func=self.convert_rad_to_positions_func)
        else:
            MotorBase.__init__(self, motor_count, name)

        self.name = name or "Arm"
        self._control_hz = control_hz
        self._period = 1.0 / control_hz
        self._set_robot_type(robot_type)

        # saber arm close MIT command by default
        self._enable_mit = True
        if robot_type == 14 or robot_type == 15:
            self._enable_mit = False

        # arm status
        self._api_control_initialized = False
        self._calibrated = False
        self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        self._session_holder = 0
        self._previous_session_holder = None

        # Control related
        if robot_type in [public_api_types_pb2.RobotType.RtHelloArcherY6_H1, public_api_types_pb2.RobotType.RtHelloFireflyY6_H1]:
            self._command_timeout_check = True
        else:
            self._command_timeout_check = False
        self._last_command_time = None
        self._command_timeout = 0.3  # 300ms
        self.__last_warning_time = time.perf_counter()  # last log warning time
        self._my_session_id = 0   # my session id, was assigned by server
        self._send_init = self._send_init
        self._send_clear_parking_stop: Optional[bool] = None

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
        Initialize robotic arm
        
        Returns:
            bool: Whether initialization was successful
        """
        try:
            # self.start()
            return True
        except Exception as e:
            log_err(f"Arm initialization failed: {e}")
            return False

    def _update(self, api_up_data, timestamp: Timestamp) -> bool:
        """
        Update robotic arm data
        
        Args:
            api_up_data: Upstream data received from API (APIUp)
            
        Returns:
            bool: Whether update was successful
        """
        try:
            if not api_up_data.HasField('arm_status'):
                return False
            arm_status = api_up_data.arm_status

            # Update motor data
            self._push_motor_data(arm_status.motor_status, timestamp)

            with self._status_lock:
                # update my session id
                self._my_session_id = api_up_data.session_id
                # Update robotic arm status
                self._api_control_initialized = arm_status.api_control_initialized
                self._calibrated = arm_status.calibrated
                self._session_holder = arm_status.session_holder

                if self._session_holder != self._previous_session_holder:
                    if self._session_holder == self._my_session_id:
                        log_info(f"Arm: You can control the arm now! Your session ID: {self._session_holder}")
                    else:
                        log_warn(f"Arm: Can not control the arm, now holder is ID: {self._session_holder}, waiting...")
                self._previous_session_holder = self._session_holder

                if arm_status.HasField('parking_stop_detail'):
                    self._parking_stop_detail = arm_status.parking_stop_detail
                else:
                    self._parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
            return True

        except Exception as e:
            log_err(f"Arm _update failed: {e}")
            return False

    async def _periodic(self):
        """
        Periodic execution function
        
        Execute periodic tasks for the robotic arm, including:
        - Status check
        - Command timeout check
        - Safety monitoring
        """
        cycle_time = 1000.0 / self._control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time
        self.__last_error_time = start_time

        await self._init()
        log_info("Arm init success")
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            try:
                # check arm error
                error = self.get_parking_stop_detail()
                if error != public_api_types_pb2.ParkingStopDetail():
                    if start_time - self.__last_error_time > 1.0:
                        log_err(f"emergency stop: {error}")
                        self.__last_error_time = start_time

                    # auto clear api communication timeout
                    if error.category == public_api_types_pb2.ParkingStopCategory.PscAPICommunicationTimeout:
                        if start_time - self.__last_warning_time > 1.0:
                            log_warn(f"You have disconnected from arm, trying to connect again.")
                            self.__last_warning_time = start_time
                        msg = self._construct_clear_parking_stop_message()
                        await self._send_message(msg)
                        # when timeout, the session holder will be release, should re-api-control-initialize again
                        self.start()

                # check motor error
                if start_time - self.__last_error_time > 1.0:
                    error_codes = self.get_motor_error_codes()
                    if error_codes is not None:
                        error_motors = []
                        for i in range(self.motor_count):
                            if error_codes[i] is not None:
                                error_motors.append(f"Motor {i} (error_code: {error_codes[i]})")
                        if error_motors:
                            error_msg = "Arm Error: " + ", ".join(error_motors) + "."
                            log_err(error_msg)
                            self.__last_error_time = start_time

                # check motor warning
                if start_time - self.__last_warning_time > 10.0:
                    warnings = self.get_motor_warnings()
                    if warnings is not None:
                        warning_motors = []
                        for i in range(self.motor_count):
                            if warnings[i] != "":
                                warning_motors.append(f"Motor {i} (warning: {warnings[i]})")
                        if warning_motors:
                            warning_msg = "Arm Warning: " + ", ".join(warning_motors) + "."
                            log_warn(warning_msg)
                            self.__last_warning_time = start_time

                # prepare sending message
                with self._status_lock:
                    s = self._send_init
                    a = self._api_control_initialized
                    c = self._calibrated
                    sh = self._session_holder
                    mi = self._my_session_id
                    sp = self._send_clear_parking_stop
                    self._send_clear_parking_stop = None

                # print(f"api_control_initialized: {a}, calibrated: {c}, session holder: {sh}, my session id: {mi}")
                
                ## send init message
                if s is None:
                    pass
                elif s:
                    msg = self._construct_init_message()
                    await self._send_message(msg)
                    self._clear_send_init()
                elif not s:
                    msg = self._construct_init_message(False)
                    await self._send_message(msg)
                    self._clear_send_init()

                ## send clear parking stop message
                if sp == True:
                    msg = self._construct_clear_parking_stop_message()
                    await self._send_message(msg)

                ## check if is holder:
                if self.robot_type == public_api_types_pb2.RobotType.RtHelloArcherY6_H1 or self.robot_type == public_api_types_pb2.RobotType.RtHelloFireflyY6_H1:
                    pass
                elif sh != mi:
                    if start_time - self.__last_warning_time > 5.0:
                        log_info(f"Arm: Waiting to get the control of the arm...")
                        self.__last_warning_time = start_time
                    continue
                
                if a == True and c == True:
                    ### no command
                    if self._last_command_time is None:
                        empty_msg = self._construct_empty_message()
                        await self._send_message(empty_msg)
                    ### command timeout
                    elif self._command_timeout_check and (start_time -
                          self._last_command_time) > self._command_timeout:
                        try:
                            motor_msg = self._construct_custom_motor_msg(
                                CommandType.BRAKE, [True] * self.motor_count)
                            msg = self._construct_custom_joint_command_msg(motor_msg)
                            await self._send_message(msg)
                        except Exception as e:
                            log_err(f"Arm failed to construct custom joint command message: {e}")
                            continue
                    ### normal command
                    else:
                        try:
                            # if no new command, use the last command 
                            command = None
                            with self._command_lock:
                                if self._target_command is None:
                                    raise ValueError(
                                        "Construct down msg failed, No target command")
                                command = self._target_command
                            if isinstance(command, MotorCommand):
                                # common motor command
                                msg = self._construct_joint_command_msg(command)
                                await self._send_message(msg)
                            else:
                                # special command
                                msg = self._construct_special_arm_command_msg(command)
                                await self._send_message(msg)
                        except Exception as e:
                            log_err(f"Arm failed to construct joint command message: {e}")
                            continue
                elif c == False:
                    # If there is anything that requires special action, modify this calibrate sending logic.
                    msg = self._construct_calibrate_message()
                    await self._send_message(msg)

            except Exception as e:
                log_err(f"Arm periodic task exception: {e}")
                continue

    # Robotic arm specific methods
    def command_timeout_check(self, check_or_not: bool = True):
        """
        Set whether to check command timeout
        """
        self._command_timeout_check = check_or_not

    def motor_command(self, command_type: CommandType, values: Union[List[bool], List[float], List[MitMotorCommand], np.ndarray]):
        """
        Set motor command
        Note:
            1. Only when CommandType is POSITION or SPEED, will validate the values.
            2. When CommandType is BRAKE, the values can be any, but the length must be the same as the motor count.
        Args:
            command_type: Command type
            values: List of command values or numpy array
        """
        # Convert numpy array to list if needed
        if isinstance(values, np.ndarray):
            values = values.tolist()

        if (command_type == CommandType.MIT or command_type == CommandType.TORQUE) and not self._enable_mit:
            raise ValueError("Due to specific configurations, certain robotic arms may require consultation before they can be safely operated. \
            The MIT command is not enabled by default on this arm. Please contact customer service to inquire about activating MIT support.")
        
        super().motor_command(command_type, values)
        self._last_command_time = time.perf_counter()

    def clear_parking_stop(self):
        """Clear parking stop"""
        with self._status_lock:
            self._send_clear_parking_stop = True

    def get_parking_stop_detail(
            self) -> public_api_types_pb2.ParkingStopDetail:
        """Get parking stop details"""
        return copy.deepcopy(self._parking_stop_detail)

    # Special command
    def enable_free_drag(self, gravity_acc: list[float]):
        """
        @brief: For enabling free drag.

        Args:
            gravity_acc: Gravity acceleration (ax, ay, az) for compensation.
                Optional, length 3 when provided.
        """
        if len(gravity_acc) != 3:
            raise ValueError("gravity_acc must be a list of 3 floats")
        acceleration_source: public_api_types_pb2.ImuAcceleration = public_api_types_pb2.ImuAcceleration(
            ax=gravity_acc[0],
            ay=gravity_acc[1],
            az=gravity_acc[2]
        )
        msg = self._construct_ApiFreeDragCommand_msg(acceleration_source)
        with self._command_lock:
            self._target_command = msg
            self._last_command_time = time.perf_counter()

    def enable_zero_current_control(self):
        """
        @brief: For enabling current control.
        """
        msg = self._construct_ApiZeroCurrentCommand_msg()
        with self._command_lock:
            self._target_command = msg
            self._last_command_time = time.perf_counter()

    def end_effector_control(
        self,
        position: Union[List[float], Tuple[float, float, float]],
        orientation: Union[List[float], Tuple[float, float, float, float], None] = None,
        gravity_acc: Optional[List[float]] = None,
    ):
        """
        @brief: For enabling end effector control.

        Args:
            position: End effector position (x, y, z) in meters, length 3.
            orientation: End effector orientation as quaternion (qw, qx, qy, qz).
                Optional, default [1, 0, 0, 0] (identity).
            gravity_acc: Gravity acceleration (ax, ay, az) for compensation.
                Optional, length 3 when provided.
        """
        if len(position) != 3:
            raise ValueError("position must be a sequence of 3 floats (x, y, z)")
        px, py, pz = float(position[0]), float(position[1]), float(position[2])

        if orientation is not None:
            if len(orientation) != 4:
                raise ValueError("orientation must be a sequence of 4 floats (qw, qx, qy, qz)")
            qw, qx, qy, qz = float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        end_pose = public_api_types_pb2.Pose(px=px, py=py, pz=pz, qw=qw, qx=qx, qy=qy, qz=qz)

        if gravity_acc is not None:
            if len(gravity_acc) != 3:
                raise ValueError("gravity_acc must be a list of 3 floats")
            acceleration_source = public_api_types_pb2.ImuAcceleration(
                ax=gravity_acc[0],
                ay=gravity_acc[1],
                az=gravity_acc[2],
            )
        else:
            acceleration_source = None

        msg = self._construct_ApiEndEffectorCommand_msg(end_pose, acceleration_source)
        with self._command_lock:
            self._target_command = msg
            self._last_command_time = time.perf_counter()

    def joint_position_control(self, joint_positions: List[float]):
        """
        @brief: A more convenient positioning mode that will plan the target position before moving.

        Args:
            joint_positions: Joint positions in rad, length must be the same as the motor count.
        """
        if len(joint_positions) != self.motor_count:
            raise ValueError("joint_positions must be a list of floats, length must be the same as the motor count")
        msg = self._construct_ApiJointPositionCommand_msg(joint_positions)
        with self._command_lock:
            self._target_command = msg
            self._last_command_time = time.perf_counter()

    def compensated_mit_control(self, mit_commands: List[MitMotorCommand], gravity_acc: List[float]):
        """
        @brief: Compensated MIT control mode. It will be more precise.
        
        Args:
            mit_commands: MIT commands, each element contains torque, speed, position, kp, kd
            gravity_acc: Gravity acceleration (ax, ay, az) for compensation.
        """
        if len(gravity_acc) != 3:
            raise ValueError("gravity_acc must be a list of 3 floats")
        acceleration_source: public_api_types_pb2.ImuAcceleration = public_api_types_pb2.ImuAcceleration(
            ax=gravity_acc[0],
            ay=gravity_acc[1],
            az=gravity_acc[2]
        )
        msg = self._construct_ApiCompensatedMitMotorTargets_msg(mit_commands, acceleration_source)
        with self._command_lock:
            self._target_command = msg
            self._last_command_time = time.perf_counter()

    # A soft lock for saber arm, will move on soon.
    def enable_mit(self):
        """Enable MIT"""
        self._enable_mit = True

    # old revert function, will be removed soon
    def convert_positions_to_rad_func(self, positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray:
        """
        Convert positions to radians

        Args:
            positions: Positions
            pulse_per_rotation: Pulse per rotation
        """
        return (positions - 65535.0 / 2.0) / pulse_per_rotation * 2 * np.pi

    def convert_rad_to_positions_func(self, positions: np.ndarray, pulse_per_rotation: np.ndarray) -> np.ndarray:
        """
        Convert radians to positions
        
        Args:
            positions: Positions
            pulse_per_rotation: Pulse per rotation
        """
        return positions / (2 * np.pi) * pulse_per_rotation + 65535.0 / 2.0

    # msg constructor
    def _construct_empty_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a empty message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_api_control_command = public_api_types_pb2.ArmApiControlCommand()
        motor_targets = public_api_types_pb2.MotorTargets()
        # Create empty motor_targets with length 0 (no targets)
        arm_api_control_command.motor_targets.CopyFrom(motor_targets)
        arm_exclusive_command.arm_api_control_command.CopyFrom(arm_api_control_command)
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_init_message(self, api_control_initialize: bool = True) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a init message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_exclusive_command.api_control_initialize = api_control_initialize
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_calibrate_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a calibrate message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_exclusive_command.calibrate = True
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_clear_parking_stop_message(self):
        """
        @brief: For constructing a clear_parking_stop message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_shared_command = public_api_types_pb2.ArmSharedCommand()
        arm_shared_command.clear_parking_stop = True
        arm_command.arm_shared_command.CopyFrom(arm_shared_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_target_motor_msg(
            self,
            pulse_per_rotation,
            dt,
            command) -> public_api_types_pb2.MotorTargets:
        """Construct downstream message"""
        # validate joint positions and velocities
        validated_command = copy.deepcopy(command)

        if validated_command.command_type == CommandType.POSITION:
            validated_positions = self.validate_joint_positions(command.position_command, dt)
            validated_command.position_command = validated_positions
        elif validated_command.command_type == CommandType.SPEED:
            validated_velocities = self.validate_joint_velocities(command.speed_command, dt)
            validated_command.speed_command = validated_velocities

        motor_targets = super()._construct_target_motor_msg(pulse_per_rotation, validated_command)
        
        return motor_targets

    def _construct_joint_command_msg(self, command: MotorCommand) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_api_control_command = public_api_types_pb2.ArmApiControlCommand()

        pulse_per_rotation_arr = self.get_motor_pulse_per_rotations()
        if pulse_per_rotation_arr is not None:
            motor_targets = self._construct_target_motor_msg(pulse_per_rotation_arr, self._period, command)
            arm_api_control_command.motor_targets.CopyFrom(motor_targets)
            arm_exclusive_command.arm_api_control_command.CopyFrom(arm_api_control_command)
            arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
            msg.arm_command.CopyFrom(arm_command)
            return msg
        else:
            raise ValueError(f"Cannot construct joint command message: pulse_per_rotation data not available (not set yet)")

    def _construct_custom_joint_command_msg(self, motor_msg: public_api_types_pb2.MotorTargets) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a custom joint command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_api_control_command = public_api_types_pb2.ArmApiControlCommand()
        
        arm_api_control_command.motor_targets.CopyFrom(motor_msg)
        arm_exclusive_command.arm_api_control_command.CopyFrom(arm_api_control_command)
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    def _construct_ApiFreeDragCommand_msg(self, acceleration_source: public_api_types_pb2.ImuAcceleration) -> public_api_types_pb2.ArmApiFreeDragCommand:
        """
        @brief: For constructing a ApiFreeDragCommand message.
        """
        msg = public_api_types_pb2.ArmApiFreeDragCommand()
        msg.acceleration_source.custom_acceleration.CopyFrom(acceleration_source)
        return msg

    def _construct_ApiZeroCurrentCommand_msg(self) -> public_api_types_pb2.ArmApiZeroCurrentCommand:
        """
        @brief: For constructing a ApiZeroCurrentCommand message.
        """
        msg = public_api_types_pb2.ArmApiZeroCurrentCommand()
        return msg

    def _construct_ApiEndEffectorCommand_msg(self, end_pose: public_api_types_pb2.Pose, 
                                             acceleration_source: public_api_types_pb2.ImuAcceleration = None,
                                            ) -> public_api_types_pb2.ArmApiEndEffectorCommand:
        """
        @brief: For constructing a ApiEndEffectorCommand message.
        """
        msg = public_api_types_pb2.ArmApiEndEffectorCommand()
        msg.end_pose.CopyFrom(end_pose)
        # warning!!! If not provided AccelerationSource, gravity compensation will close.
        if acceleration_source is not None:
            msg.acceleration_source.CopyFrom(acceleration_source)
        return msg

    def _construct_ApiJointPositionCommand_msg(self, joint_positions: List[float]) -> public_api_types_pb2.ArmApiJointPositionCommand:
        """
        @brief: For constructing a ApiJointPositionCommand message.
        """
        msg = public_api_types_pb2.ArmApiJointPositionCommand()
        msg.joint_positions.extend(joint_positions)
        return msg

    def _construct_ApiCompensatedMitMotorTargets_msg(self, mit_targets: List[MitMotorCommand], 
                                                     acceleration_source: public_api_types_pb2.ImuAcceleration) -> public_api_types_pb2.ArmApiCompensatedMitMotorTargets:
        """
        @brief: For constructing a ApiCompensatedMitMotorTargets message.
        """
        msg = public_api_types_pb2.ArmApiCompensatedMitMotorTargets()
        proto_targets = [
            public_api_types_pb2.MitMotorTarget(
                torque=float(cmd.torque),
                speed=float(cmd.speed),
                position=float(cmd.position),
                kp=float(cmd.kp),
                kd=float(cmd.kd),
            )
            for cmd in mit_targets
        ]
        msg.mit_target.extend(proto_targets)
        msg.acceleration_source.custom_acceleration.CopyFrom(acceleration_source)
        return msg

    def _construct_special_arm_command_msg(self, command: Any) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a special arm command message.
        """
        msg = public_api_down_pb2.APIDown()
        arm_command = public_api_types_pb2.ArmCommand()
        arm_exclusive_command = public_api_types_pb2.ArmExclusiveCommand()
        arm_api_control_command = public_api_types_pb2.ArmApiControlCommand()
        if isinstance(command, public_api_types_pb2.ArmApiFreeDragCommand):
            arm_api_control_command.arm_api_free_drag_command.CopyFrom(command)
        elif isinstance(command, public_api_types_pb2.ArmApiZeroCurrentCommand):
            arm_api_control_command.arm_api_zero_current_command.CopyFrom(command)
        elif isinstance(command, public_api_types_pb2.ArmApiEndEffectorCommand):
            arm_api_control_command.arm_api_end_effector_command.CopyFrom(command)
        elif isinstance(command, public_api_types_pb2.ArmApiJointPositionCommand):
            arm_api_control_command.arm_api_joint_position_command.CopyFrom(command)
        elif isinstance(command, public_api_types_pb2.ArmApiCompensatedMitMotorTargets):
            arm_api_control_command.arm_api_compensated_mit_motor_targets.CopyFrom(command)
        else:
            raise ValueError(f"Invalid arm command type: {type(command)}")
        arm_exclusive_command.arm_api_control_command.CopyFrom(arm_api_control_command)
        arm_command.arm_exclusive_command.CopyFrom(arm_exclusive_command)
        msg.arm_command.CopyFrom(arm_command)
        return msg

    # Configuration related methods
    def get_session_holder(self) -> int:
        """Get session holder"""
        with self._status_lock:
            return self._session_holder

    def get_my_session_id(self) -> int:
        """Get my session id"""
        with self._status_lock:
            return self._my_session_id
            
    def get_arm_config(self) -> Optional[ArmConfig]:
        """Get current robotic arm configuration"""
        return copy.deepcopy(get_arm_config(self.robot_type))

    def get_joint_limits(self) -> Optional[List[List[float]]]:
        """Get joint limits"""
        return copy.deepcopy(arm_config_manager.get_joint_limits(self.robot_type))

    def validate_joint_positions(self,
                                 positions: List[float],
                                 dt: float = 0.002) -> List[float]:
        """
        Validate whether joint positions are within limit range and return corrected position list
        
        Args:
            positions: Target position list (rad)
            dt: Time step (s), used for velocity limit calculation
            
        Returns:
            List[float]: Corrected position list
        """
        last_positions = arm_config_manager.get_last_positions(self.robot_type)
        
        if last_positions is None:
            current_positions = self.cache_positions.tolist()
            if len(current_positions) == len(positions):
                arm_config_manager.set_last_positions(self.robot_type, current_positions)
            else:
                log_warn(f"Arm: Current motor positions count({len(current_positions)}) does not match the target positions count({len(positions)})")
        
        return arm_config_manager.validate_joint_positions(
            self.robot_type, positions, dt)

    def validate_joint_velocities(self,
                                  velocities: List[float],
                                  dt: float = 0.002) -> List[float]:
        """
        Validate whether joint velocities are within limit range and return corrected velocity list
        """
        return arm_config_manager.validate_joint_velocities(
            self.robot_type, velocities, dt)

    def get_joint_names(self) -> Optional[List[str]]:
        """Get joint names"""
        return copy.deepcopy(arm_config_manager.get_joint_names(self.robot_type))

    def get_expected_motor_count(self) -> Optional[int]:
        """Get expected motor count"""
        return copy.deepcopy(arm_config_manager.get_motor_count(self.robot_type))

    def check_motor_count_match(self) -> bool:
        """Check if motor count matches configuration"""
        expected_count = self.get_expected_motor_count()
        if expected_count is None:
            return False
        return self.motor_count == expected_count

    def get_arm_series(self) -> int:
        """Get robotic arm series"""
        return copy.deepcopy(self.robot_type)

    def get_arm_name(self) -> Optional[str]:
        """Get robotic arm name"""
        config = self.get_arm_config()
        return copy.deepcopy(config.name) if config else None

    def reload_arm_config_from_dict(self, config_data: dict) -> bool:
        """
        Reload current robotic arm configuration parameters from dictionary data. Parameters are used to provide limiting indicators for velocity and position commands.
        
        Args:
            config_data: Configuration data dictionary
            
        Returns:
            bool: Whether reload was successful
        """
        try:
            success = arm_config_manager.reload_from_dict(
                self.robot_type, config_data)
            if success:
                log_common(f"Arm: reload arm config success: {config_data.get('name', 'unknown')}")
            else:
                log_err(f"Arm: reload arm config from dict failed: {config_data.get('name', 'unknown')}")
            return success
        except Exception as e:
            log_err(f"Arm: reload arm config from dict exception: {e}")
            return False

    def set_initial_positions(self, positions: List[float]):
        """
        Set initial positions of robotic arm, used for velocity limit calculation
        
        Args:
            positions: Initial position list (rad)
        """
        arm_config_manager.set_initial_positions(self.robot_type, positions)

    def set_initial_velocities(self, velocities: List[float]):
        """
        Set initial velocities of robotic arm, used for acceleration limit calculation
        
        Args:
            velocities: Initial velocity list (rad/s)
        """
        arm_config_manager.set_initial_velocities(self.robot_type, velocities)

    def get_last_positions(self) -> Optional[List[float]]:
        """
        Get last position record
        
        Returns:
            List[float]: Last position list, returns None if no record exists
        """
        return arm_config_manager.get_last_positions(self.robot_type)

    def get_last_velocities(self) -> Optional[List[float]]:
        """
        Get last velocity record
        
        Returns:
            List[float]: Last velocity list, returns None if no record exists
        """
        return arm_config_manager.get_last_velocities(self.robot_type)

    def clear_position_history(self):
        """Clear position history records"""
        arm_config_manager.clear_position_history(self.robot_type)

    def clear_velocity_history(self):
        """Clear velocity history records"""
        arm_config_manager.clear_velocity_history(self.robot_type)

    def clear_motion_history(self):
        """Clear all motion history records (position and velocity)"""
        arm_config_manager.clear_motion_history(self.robot_type)
