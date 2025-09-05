#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import time
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .common_utils import is_valid_ws_url, InvalidWSURLException, delay
from .common_utils import log_warn, log_info, log_err, log_common
from .error_type import WsError, ProtocolError
from .device_base import DeviceBase

import asyncio
import threading
import websockets
from typing import Optional, Tuple, List, Type, Dict, Any
from websockets.exceptions import ConnectionClosed

RECV_CYCLES = 1000
RAW_DATA_LEN = 50


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


class HexDeviceApi:
    """
    @brief: HexDeviceApi provides an API interface for HexDevice to communicate with WebSocket.
    @params:
        ws_url: the url of the websocket server
        control_hz: the frequency of the control loop
    """

    def __init__(self, ws_url: str, control_hz: int = 1000):
        # variables init
        self.ws_url = ws_url
        try:
            self.__ws_url: str = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            log_err("Invalid WebSocket URL: " + str(e))

        self.__websocket = None
        self.device_list = []  ## list of device
        self.__raw_data = []  ## raw data buffer
        self.__control_hz = control_hz

        # 设备工厂
        self._device_factory = DeviceFactory()

        # 设备任务管理
        self._device_tasks = {}  # 存储设备及其对应的异步任务

        # 自动注册可用的设备类
        self._register_available_device_classes()

        self.__shutdown_event = None  # the handle event for shutdown api
        self.__loop = None  ## async loop thread
        self.__loop_thread = threading.Thread(target=self.__loop_start,
                                              daemon=True)

        # init api
        self.__loop_thread.start()

    def register_device(self, device: DeviceBase):
        """
        注册设备并设置发送消息回调
        
        Args:
            device: 设备对象，需要支持set_send_message_callback方法
        """
        # set send message callback
        if hasattr(device, '_set_send_message_callback'):
            device._set_send_message_callback(self._send_down_message)

        self.device_list.append(device)

        # start _periodic task for each device
        self.start_device_periodic_task(device)

    def _register_available_device_classes(self):
        """
        自动注册可用的设备类
        """
        try:
            from .chassis_maver import ChassisMaver
            self.register_device_class(ChassisMaver)
            log_info("已注册 ChassisMaver 设备类")
        except ImportError as e:
            log_warn(f"无法导入 ChassisMaver: {e}")

        try:
            from .arm_archer import ArmArcher
            self.register_device_class(ArmArcher)
            log_info("已注册 ArmArcher 设备类")
        except ImportError as e:
            log_warn(f"无法导入 ArmArcher: {e}")

        # TODO: 添加更多设备类的注册
        # lift、rotate lift...

    def register_device_class(self, device_class):
        """
        注册设备类到工厂
        
        Args:
            device_class: 设备类
        """
        self._device_factory.register_device_class(device_class)

    def find_device_by_robot_type(self, robot_type) -> Optional[DeviceBase]:
        """
        根据robot_type查找设备
        
        Args:
            robot_type: 机器人类型
            
        Returns:
            匹配的设备或None
        """
        for device in self.device_list:
            if hasattr(device,
                       'robot_type') and device.robot_type == robot_type:
                return device
        return None

    def create_and_register_device(self, robot_type,
                                   api_up) -> Optional[DeviceBase]:
        """
        根据robot_type创建并注册设备
        
        Args:
            robot_type: 机器人类型
            **kwargs: 设备构造参数
            
        Returns:
            创建的设备实例或None
        """
        device = self._device_factory.create_device_for_robot_type(
            robot_type,
            send_message_callback=self._send_down_message,
            api_up=api_up)

        if device:
            self.device_list.append(device)
            self.start_device_periodic_task(device)

        return device

    def start_device_periodic_task(self, device: DeviceBase):
        """
        启动设备的周期性任务
        
        Args:
            device: 设备实例
        """
        if device in self._device_tasks:
            log_warn(f"Periodic task for {device.name} already exists")
            return

        # 创建异步任务
        task = asyncio.create_task(self._device_periodic_runner(device))
        self._device_tasks[device] = task
        log_common(f"Begin periodic task for {device.name}")

    async def _device_periodic_runner(self, device: DeviceBase):
        """
        设备周期性任务运行器
        
        Args:
            device: 设备实例
        """
        try:
            await device._periodic()
        except asyncio.CancelledError:
            log_info(f"设备 {device.name} 的周期性任务被取消")
        except Exception as e:
            log_err(f"设备 {device.name} 的周期性任务出错: {e}")
        finally:
            # 清理任务引用
            if device in self._device_tasks:
                del self._device_tasks[device]

    def stop_device_periodic_task(self, device: DeviceBase):
        """
        停止设备的周期性任务
        
        Args:
            device: 设备实例
        """
        if device in self._device_tasks:
            task = self._device_tasks[device]
            task.cancel()
            log_info(f"取消设备 {device.name} 的周期性任务")

    async def stop_all_device_tasks(self):
        """
        停止所有设备的周期性任务
        """
        tasks_to_cancel = list(self._device_tasks.values())
        for task in tasks_to_cancel:
            task.cancel()

        if tasks_to_cancel:
            try:
                await asyncio.gather(*tasks_to_cancel, return_exceptions=True)
            except Exception as e:
                log_err(f"停止设备任务时出错: {e}")

        self._device_tasks.clear()
        log_info("所有设备周期性任务已停止")

    def get_device_task_status(self) -> Dict[str, Any]:
        """
        获取设备任务状态
        
        Returns:
            Dict: 设备任务状态信息
        """
        status = {
            'total_devices': len(self.device_list),
            'active_tasks': len(self._device_tasks),
            'device_tasks': {}
        }

        for device, task in self._device_tasks.items():
            status['device_tasks'][device.name] = {
                'task_done': task.done(),
                'task_cancelled': task.cancelled(),
                'device_type': device.__class__.__name__,
                'robot_type': getattr(device, 'robot_type', None)
            }

        return status

    # message function
    async def _send_down_message(self, data: public_api_down_pb2.APIDown):
        msg = data.SerializeToString()
        if self.__websocket is None:
            raise AttributeError("_send_down_message: websocket tx is None")
        else:
            await self.__websocket.send(msg)

    async def __capture_data_frame(self) -> Optional[public_api_up_pb2.APIUp]:
        """
        @brief: Continuously monitor WebSocket connections until:
        1. Received a valid binary Protobuf message
        2. Protocol error occurred
        3. Connection closed
        4. No data due to timeout
        
        @params:
            websocket: Established WebSocket connection object
            
        @return:
            base_backend.APIUp object or None
        """
        while True:
            try:
                # Check if websocket is connected
                if self.__websocket is None:
                    log_err("WebSocket is not connected")
                    await asyncio.sleep(1)
                    continue

                # Timeout
                message = await asyncio.wait_for(self.__websocket.recv(),
                                                 timeout=3.0)
                # Only process binary messages
                if isinstance(message, bytes):
                    try:
                        # Protobuf parse
                        api_up = public_api_up_pb2.APIUp()
                        api_up.ParseFromString(message)

                        if not api_up.IsInitialized():
                            raise ProtocolError("Incomplete message")
                        return api_up

                    except Exception as e:
                        log_err(f"Protobuf encode fail: {e}")
                        raise ProtocolError("Invalid message format") from e

                elif isinstance(message, str):
                    log_common(f"ignore string message: {message[:50]}...")
                    continue

            except asyncio.TimeoutError:
                log_err("No data received for 3 seconds")
                continue

            except ConnectionClosed as e:
                log_err(
                    f"Connection closed (code: {e.code}, reason: {e.reason})")
                try:
                    await self.__reconnect()
                    continue
                except ConnectionError as e:
                    log_err(f"Reconnect failed: {e}")
                    self.close()

            except Exception as e:
                log_err(f"Unknown error: {str(e)}")
                raise WsError("Unexpected error") from e

    # websocket function
    async def __connect_ws(self):
        """
        @brief: Connect to the WebSocket server.
        """
        try:
            self.__websocket = await websockets.connect(self.__ws_url,
                                                        ping_interval=20,
                                                        ping_timeout=60,
                                                        close_timeout=5)
        except Exception as e:
            log_err(f"Failed to open WebSocket connection: {e}")
            log_common(
                "Public API haved exited, please check your network connection and restart the server again."
            )
            exit(1)

    async def __reconnect(self):
        retry_count = 0
        max_retries = 5
        base_delay = 1

        while retry_count < max_retries:
            try:
                if self.__websocket:
                    await self.__websocket.close()
                self.__websocket = await websockets.connect(self.__ws_url,
                                                            ping_interval=20,
                                                            ping_timeout=60,
                                                            close_timeout=5)
                return
            except Exception as e:
                delay = base_delay * (2**retry_count)
                log_warn(
                    f"Reconnect failed (attempt {retry_count+1}): {e}, retrying in {delay}s"
                )
                await asyncio.sleep(delay)
                retry_count += 1
        raise ConnectionError("Maximum reconnect retries exceeded")

    # process manager
    ## sync function
    def __loop_start(self):
        """
        @brief: 开启async线程，通过该函数隔离async线程
        @return:
            None
        """
        self.__loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.__loop)
        self.__loop.run_until_complete(self.__main_loop())

    def close(self):
        if self.__loop and self.__loop.is_running():
            log_warn("HexDevice API is closing...")
            asyncio.run_coroutine_threadsafe(self.__async_close(), self.__loop)

    def is_api_exit(self) -> bool:
        """
        @brief: 检查API是否退出
        @return:
            bool: True if API is exiting, False otherwise
        """
        if self.__loop is None:
            return False
        return self.__loop.is_closed()

    ## async function
    async def __async_close(self):
        """
        @brief: 关闭async线程
        @return:
            None
        """
        try:
            if self.__websocket:
                await self.__websocket.close()
        except Exception as e:
            log_err(f"Error closing websocket: {e}")
        finally:
            if self.__shutdown_event is not None:
                self.__shutdown_event.set()

    async def __main_loop(self):
        self.__shutdown_event = asyncio.Event()
        log_common("HexDevice Api started.")

        # 建立 WebSocket 连接
        await self.__connect_ws()
        log_common("WebSocket connected.")

        task1 = asyncio.create_task(self.__periodic_data_parser())
        self.__tasks = [task1]
        await self.__shutdown_event.wait()

        # 停止所有设备任务
        await self.stop_all_device_tasks()

        # 停止主任务
        for task in self.__tasks:
            task.cancel()

        # 等待所有任务完成，处理取消异常
        try:
            await asyncio.gather(*self.__tasks, return_exceptions=True)
        except Exception as e:
            log_err(f"Error during task cleanup: {e}")

        log_err("HexDevice api main_loop exited.")

    async def __periodic_data_parser(self):
        """
        @brief: 周期性解析数据
        @return:
            None
        """
        while True:
            try:
                api_up = await self.__capture_data_frame()
                if len(self.__raw_data) >= RAW_DATA_LEN:
                    self.__raw_data.pop(0)
                self.__raw_data.append(api_up)
            except Exception as e:
                log_err(f"__periodic_data_parser error: {e}")
                continue

            # 获取 robot_type 的类型信息
            robot_type = api_up.robot_type
            robot_type_name = public_api_types_pb2.RobotType.Name(robot_type)
            # print(f"robot_type 类型: {type(robot_type)}, 值: {robot_type}, 名称: {robot_type_name}")

            # 检查 robot_type 是否有效
            if isinstance(api_up.robot_type, int):
                device = self.find_device_by_robot_type(robot_type)

                if device:
                    device._update(api_up)
                else:
                    log_info(f"create new device: {robot_type_name}")

                    try:
                        device = self.create_and_register_device(
                            robot_type, api_up)
                    except Exception as e:
                        log_err(f"create_and_register_device error: {e}")
                        continue

                    if device:
                        device._update(api_up)
                    else:
                        log_warn(f"unknown device type: {robot_type_name}")
            else:
                continue

    # data getter
    def _get_raw_data(self) -> Tuple[public_api_up_pb2.APIUp, int]:
        """
        The original data is acquired and stored in the form of a sliding window sequence. 
        By parsing this sequence, a lossless data stream can be obtained.
        The maximum length of this buffer is RAW_DATA_LEN.
        You can use '_parse_wheel_data' to parse the raw data.
        """
        if len(self.__raw_data) == 0:
            return (None, 0)
        return (self.__raw_data.pop(0), len(self.__raw_data))
