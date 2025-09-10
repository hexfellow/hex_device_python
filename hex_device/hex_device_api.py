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
from .device_factory import DeviceFactory

import asyncio
import threading
import websockets
from typing import Optional, Tuple, List, Type, Dict, Any
from websockets.exceptions import ConnectionClosed

RAW_DATA_LEN = 50

class HexDeviceApi:
    """
    @brief: HexDeviceApi provides an API interface for HexDevice to communicate with WebSocket.
    @params:
        ws_url: the url of the websocket server
        control_hz: the frequency of the control loop
    """

    def __init__(self, ws_url: str, control_hz: int = 500):
        # variables init
        self.ws_url = ws_url
        try:
            self.__ws_url: str = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            log_err("Invalid WebSocket URL: " + str(e))

        self.__websocket = None
        self.__raw_data = []  ## raw data buffer
        self.__control_hz = control_hz

        # 设备工厂
        self._device_factory = DeviceFactory()
        # 注册可用的设备类
        self._register_available_device_classes()

        # 内部设备管理（用于任务管理和内部操作）
        self._internal_device_list = []  # 内部设备列表，不可被用户修改
        self._device_id_counter = 0  # 设备ID计数器
        self._device_id_map = {}  # 设备ID到设备的映射
        self._device_to_id_map = {}  # 设备到ID的反向映射
        
        # 设备任务管理
        self._device_tasks = {}  # 存储设备ID及其对应的异步任务

        self.__shutdown_event = None  # the handle event for shutdown api
        self.__loop = None  ## async loop thread
        self.__loop_thread = threading.Thread(target=self.__loop_start,
                                              daemon=True)
        # init api
        self.__loop_thread.start()

    @property
    def device_list(self):
        """
        用户设备列表接口（只读）
        
        返回内部设备列表的只读视图，用户无法通过此列表修改内部设备管理
        """
        class ReadOnlyDeviceList:
            def __init__(self, internal_list):
                self._internal_list = internal_list
            
            def __getitem__(self, index):
                return self._internal_list[index]
            
            def __len__(self):
                return len(self._internal_list)
            
            def __iter__(self):
                return iter(self._internal_list)
            
            def __contains__(self, item):
                return item in self._internal_list
            
            def __repr__(self):
                return repr(self._internal_list)
            
            def __str__(self):
                return str(self._internal_list)
            
            def index(self, item):
                return self._internal_list.index(item)
            
            def count(self, item):
                return self._internal_list.count(item)
            
            # 禁用修改方法
            def append(self, *args, **kwargs):
                raise AttributeError("Cannot modify read-only device list")
            
            def remove(self, *args, **kwargs):
                raise AttributeError("Cannot modify read-only device list")
            
            def pop(self, *args, **kwargs):
                raise AttributeError("Cannot modify read-only device list")
            
            def clear(self, *args, **kwargs):
                raise AttributeError("Cannot modify read-only device list")
            
            def extend(self, *args, **kwargs):
                raise AttributeError("Cannot modify read-only device list")
            
            def insert(self, *args, **kwargs):
                raise AttributeError("Cannot modify read-only device list")
        
        return ReadOnlyDeviceList(self._internal_device_list)

    def _register_available_device_classes(self):
        """
        自动注册可用的设备类
        """
        try:
            from .chassis_maver import ChassisMaver
            self._register_device_class(ChassisMaver)
            log_info("已注册 ChassisMaver 设备类")
        except ImportError as e:
            log_warn(f"无法导入 ChassisMaver: {e}")

        try:
            from .arm_archer import ArmArcher
            self._register_device_class(ArmArcher)
            log_info("已注册 ArmArcher 设备类")
        except ImportError as e:
            log_warn(f"无法导入 ArmArcher: {e}")

        # TODO: 添加更多设备类的注册
        # lift、rotate lift...

    def _register_device_class(self, device_class):
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
        for device in self._internal_device_list:
            if hasattr(device,
                       'robot_type') and device.robot_type == robot_type:
                return device
        return None

    def _create_and_register_device(self, robot_type,
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
            # 为设备分配唯一ID
            device_id = self._device_id_counter
            self._device_id_counter += 1
            
            # 添加到内部设备列表
            self._internal_device_list.append(device)
            self._device_id_map[device_id] = device
            self._device_to_id_map[device] = device_id  # 反向映射
            
            self._start_device_periodic_task(device_id)

        return device

    def _start_device_periodic_task(self, device_id: int):
        """
        启动设备的周期性任务
        
        Args:
            device_id: 设备ID
        """
        if device_id in self._device_tasks:
            device = self._device_id_map.get(device_id)
            device_name = device.name if device else f"device_{device_id}"
            log_warn(f"Periodic task for {device_name} already exists")
            return

        device = self._device_id_map.get(device_id)
        if not device:
            log_err(f"Device with ID {device_id} not found")
            return

        # 创建异步任务
        task = asyncio.create_task(self._device_periodic_runner(device_id))
        self._device_tasks[device_id] = task
        log_common(f"Begin periodic task for {device.name}")

    async def _device_periodic_runner(self, device_id: int):
        """
        设备周期性任务运行器
        
        Args:
            device_id: 设备ID
        """
        device: DeviceBase = self._device_id_map.get(device_id)
        if not device:
            log_err(f"Device with ID {device_id} not found in periodic runner")
            return
            
        try:
            await device._periodic()
        except asyncio.CancelledError:
            log_info(f"设备 {device.name} 的周期性任务被取消")
        except Exception as e:
            log_err(f"设备 {device.name} 的周期性任务出错: {e}")
        finally:
            # 清理任务引用
            if device_id in self._device_tasks:
                del self._device_tasks[device_id]

    def _check_and_cleanup_orphaned_tasks(self):
        """
        检查并清理被遗弃的任务
        
        当设备实例被替换或删除时，可能存在仍在运行的任务。
        此方法会检查并清理这些被遗弃的任务。
        
        Returns:
            int: 清理的任务数量
        """
        orphaned_count = 0
        tasks_to_remove = []
        
        for device_id, task in self._device_tasks.items():
            device = self._device_id_map.get(device_id)
            if device and device not in self._internal_device_list:
                log_warn(f"发现被遗弃的任务: 设备ID {device_id} ({device.name})")
                task.cancel()
                tasks_to_remove.append(device_id)
                orphaned_count += 1
        
        # 清理被遗弃的任务
        for device_id in tasks_to_remove:
            del self._device_tasks[device_id]
        
        if orphaned_count > 0:
            log_info(f"已清理 {orphaned_count} 个被遗弃的任务")
        
        return orphaned_count

    def _get_orphaned_tasks_info(self):
        """
        获取被遗弃任务的信息
        
        Returns:
            Dict: 被遗弃任务的信息
        """
        orphaned_tasks = {}
        for device_id, task in self._device_tasks.items():
            device = self._device_id_map.get(device_id)
            if device and device not in self._internal_device_list:
                orphaned_tasks[device_id] = {
                    'task': task,
                    'device': device,
                    'device_name': device.name,
                    'task_done': task.done(),
                    'task_cancelled': task.cancelled()
                }
        return orphaned_tasks

    async def _stop_all_device_tasks(self):
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
            'total_devices': len(self._internal_device_list),
            'active_tasks': len(self._device_tasks),
            'device_tasks': {}
        }

        for device_id, task in self._device_tasks.items():
            device = self._device_id_map.get(device_id)
            if device:
                status['device_tasks'][device.name] = {
                    'device_id': device_id,
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
        max_retries = 3
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
        await self._stop_all_device_tasks()

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
        check_counter = 0
        ORPHANED_TASK_CHECK_INTERVAL = 100
        
        while True:
            try:
                api_up = await self.__capture_data_frame()
                if len(self.__raw_data) >= RAW_DATA_LEN:
                    self.__raw_data.pop(0)
                self.__raw_data.append(api_up)
            except Exception as e:
                log_err(f"__periodic_data_parser error: {e}")
                continue

            # 定期检查被遗弃的任务
            check_counter += 1
            if check_counter >= ORPHANED_TASK_CHECK_INTERVAL:
                check_counter = 0
                orphaned_count = self._check_and_cleanup_orphaned_tasks()
                if orphaned_count > 0:
                    log_warn(f"found {orphaned_count} orphaned tasks")

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
                        device = self._create_and_register_device(
                            robot_type, api_up)
                    except Exception as e:
                        log_err(f"_create_and_register_device error: {e}")
                        continue

                    if device:
                        device._update(api_up)
                    else:
                        log_warn(f"unknown device type: {robot_type_name}")
            else:
                continue

    # data getter
    def get_raw_data(self) -> Tuple[public_api_up_pb2.APIUp, int]:
        """
        The original data is acquired and stored in the form of a sliding window sequence. 
        By parsing this sequence, a lossless data stream can be obtained.
        The maximum length of this buffer is RAW_DATA_LEN.
        You can use '_parse_wheel_data' to parse the raw data.
        """
        if len(self.__raw_data) == 0:
            return (None, 0)
        return (self.__raw_data.pop(0), len(self.__raw_data))
