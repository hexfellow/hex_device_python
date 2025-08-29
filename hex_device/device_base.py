#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any
import threading
import time

class DeviceBase(ABC):
    """
    设备基类
    定义所有设备的通用接口和基本功能
    """
    
    def __init__(self, name: str = "", send_message_callback=None):
        """
        初始化设备基类
        Args:
            name: 设备名称
        """
        self.name = name or "Device"

        # 发送消息回调函数
        self._send_message_callback = send_message_callback

        # 时间戳
        self._last_update_time = time.time()
        
        # 线程锁
        self._data_lock = threading.Lock()
        
        # 数据更新标志
        self._has_new_data = False
        
    # 通用函数
    def _set_send_message_callback(self, callback):
        """
        设置发送消息的回调函数
        
        Args:
            callback: 异步回调函数，接受一个参数（消息对象）
        """
        self._send_message_callback = callback

    async def _send_message(self, msg):
        """
        发送消息的通用方法
        
        Args:
            msg: 要发送的消息对象
        """
        if self._send_message_callback:
            await self._send_message_callback(msg)
        else:
            raise AttributeError("send_message: send_message_callback is not set")

    def set_has_new_data(self):
        with self._data_lock:
            self._has_new_data = True

    def has_new_data(self) -> bool:
        """检查是否有新数据"""
        with self._data_lock:
            return self._has_new_data
    
    def clear_new_data_flag(self):
        """清除新数据标志"""
        with self._data_lock:
            self._has_new_data = False
    
    def get_status_summary(self) -> Dict[str, Any]:
        """获取设备状态摘要"""
        return {
            'name': self.name,
            'has_new_data': self.has_new_data(),
            'last_update_time': self._last_update_time,
        }
    
    # 抽象方法 - 子类必须实现
    @abstractmethod
    async def _init(self) -> bool:
        """
        初始化设备
        
        Returns:
            bool: 是否成功初始化
        """
        pass
    
    @abstractmethod
    def _update(self, api_up_data) -> bool:
        """
        更新设备数据
        
        Args:
            api_up_data: 从API接收的上行数据 (APIUp)
            
        Returns:
            bool: 是否成功更新
        """
        pass

    @abstractmethod
    async def _periodic(self):
        """
        周期性执行函数
        
        子类必须实现此方法，用于执行设备的周期性任务，
        如状态检查、数据更新、控制计算等。
        
        Returns:
            bool: 是否成功执行
        """
        pass
    
    def _update_timestamp(self):
        """更新时间戳"""
        with self._data_lock:
            self._last_update_time = time.time()
            self._has_new_data = True
    
    def __str__(self) -> str:
        """字符串表示"""
        return f"{self.name}, {self.has_new_data}, {self._last_update_time}"
    
    def __repr__(self) -> str:
        """详细字符串表示"""
        return f"DeviceBase(name='{self.name}', has_new_data={self.has_new_data}, last_update_time={self._last_update_time})"
    