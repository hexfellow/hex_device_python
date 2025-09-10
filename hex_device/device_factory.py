#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
from typing import Optional, Tuple, List, Type, Dict, Any
from .device_base import DeviceBase

class DeviceFactory:
    """
    设备工厂类，负责根据robot_type创建和管理设备实例
    """

    def __init__(self):
        self._device_classes: List[Type[DeviceBase]] = []

    def register_device_class(self, device_class):
        """
        注册设备类
        
        Args:
            device_class: 设备类，必须支持_supports_robot_type类方法
        """
        if hasattr(device_class, '_supports_robot_type'):
            self._device_classes.append(device_class)
        else:
            raise ValueError(
                f"设备类 {device_class.__name__} 必须支持 _supports_robot_type 类方法")

    def create_device_for_robot_type(
        self,
        robot_type,
        send_message_callback=None,
        api_up=None,
    ):
        """
        根据robot_type创建设备实例
        
        Args:
            robot_type: 机器人类型
            send_message_callback: 发送消息回调函数
            api_up: API上行数据，用于提取设备构造参数
            **kwargs: 其他参数
            
        Returns:
            设备实例或None
        """
        for device_class in self._device_classes:
            if device_class._supports_robot_type(robot_type):
                # 从api_up中提取构造参数
                constructor_params = self._extract_constructor_params(
                    device_class, robot_type, api_up)

                # 合并参数
                all_params = {
                    'send_message_callback': send_message_callback,
                    **constructor_params,
                }

                device = device_class(**all_params)
                device._set_robot_type(robot_type)
                return device

        return None

    def _extract_constructor_params(self, device_class, robot_type, api_up):
        """
        从api_up中提取设备构造参数
        
        Args:
            device_class: 设备类
            robot_type: 机器人类型
            api_up: API上行数据
            
        Returns:
            dict: 构造参数字典
        """
        params = {}

        if api_up is None:
            return params

        # 根据设备类名提取不同的参数
        class_name = device_class.__name__

        if class_name == 'ArmArcher':
            params['robot_type'] = robot_type

            # 从api_up中获取motor_count
            motor_count = self._get_motor_count_from_api_up(api_up)
            if motor_count is not None:
                params['motor_count'] = motor_count

        elif class_name == 'ChassisMaver':
            # 从api_up中获取motor_count
            motor_count = self._get_motor_count_from_api_up(api_up)
            if motor_count is not None:
                params['motor_count'] = motor_count

        ## TODO:后续如何增加不同的设备，需要根据新类需要的参数增加新的额外参数提取方法。
        ## 前面已经使用了try进行错误捕获，这里如果参数捕获有问题，直接raise即可。

        return params

    def _get_motor_count_from_api_up(self, api_up):
        """
        从api_up中获取电机数量
        
        Args:
            api_up: API上行数据
            
        Returns:
            int: 电机数量或None
        """
        if api_up is None:
            return None

        # 检查arm_status
        if hasattr(api_up, 'arm_status') and api_up.arm_status:
            if hasattr(api_up.arm_status, 'motor_status'):
                return len(api_up.arm_status.motor_status)

        # 检查base_status
        if hasattr(api_up, 'base_status') and api_up.base_status:
            if hasattr(api_up.base_status, 'motor_status'):
                return len(api_up.base_status.motor_status)

        return None

    def get_supported_robot_types(self):
        """
        获取所有支持的机器人类型
        
        Returns:
            List: 支持的机器人类型列表
        """
        supported_types = []
        for device_class in self._device_classes:
            if hasattr(device_class, 'SUPPORTED_ROBOT_TYPES'):
                supported_types.extend(device_class.SUPPORTED_ROBOT_TYPES)
        return supported_types
