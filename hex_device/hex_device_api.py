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
from .device_base_optional import OptionalDeviceBase

import asyncio
import threading
import websockets
from typing import Optional, Tuple, List, Type, Dict, Any, Union
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

        self._device_factory = DeviceFactory()
        # Register available device classes
        self._register_available_device_classes()

        # Internal device management (for task management and internal operations)
        self._internal_device_list = []  # Internal device list
        self._device_id_counter = 0  # Device ID counter
        self._device_id_map: Dict[int, Union[DeviceBase, OptionalDeviceBase]] = {}  # Device ID to device mapping
        self._device_to_id_map = {}  # Device to ID reverse mapping
        
        # Optional device management
        self._optional_device_list: List[OptionalDeviceBase] = []  # Optional device list
        
        # Device task management
        self._device_tasks = {}  # Store device IDs and their corresponding async tasks

        self.__shutdown_event = None  # the handle event for shutdown api
        self.__loop = None  ## async loop thread
        self.__loop_thread = threading.Thread(target=self.__loop_start,
                                              daemon=True)
        # init api
        self.__loop_thread.start()

    @property
    def device_list(self):
        """
        User device list interface (read-only)
        
        Returns a read-only view of the internal device list, users cannot modify internal device management through this list
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
            
            # Disable modification methods
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

    @property
    def optional_device_list(self):
        """
        User optional device list interface (read-only)
        
        Returns a read-only view of the optional device list, users cannot modify internal device management through this list
        """
        class ReadOnlyOptionalDeviceList:
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
                return f"ReadOnlyOptionalDeviceList({self._internal_list})"

        return ReadOnlyOptionalDeviceList(self._optional_device_list)

    def _register_available_device_classes(self):
        """
        Automatically register available device classes
        """
        try:
            from .chassis import Chassis
            self._register_device_class(Chassis)
            log_info("Registered Chassis device class")
        except ImportError as e:
            log_warn(f"Unable to import Chassis: {e}")

        try:
            from .arm_archer import ArmArcher
            self._register_device_class(ArmArcher)
            log_info("Registered ArmArcher device class")
        except ImportError as e:
            log_warn(f"Unable to import ArmArcher: {e}")

        try:
            from .hands import Hands
            self._register_device_class(Hands)
            log_info("Registered Hands device class")
        except ImportError as e:
            log_warn(f"Unable to import Hands: {e}")

        # TODO: Add registration for more device classes
        # lift、rotate lift...

    def _register_device_class(self, device_class):
        """
        Register device class to factory
        
        Args:
            device_class: Device class
        """
        self._device_factory.register_device_class(device_class)

    def find_device_by_robot_type(self, robot_type) -> Optional[DeviceBase]:
        """
        Find device by robot_type
        
        Args:
            robot_type: Robot type
            
        Returns:
            Matching device or None
        """
        for device in self._internal_device_list:
            if hasattr(device,
                       'robot_type') and device.robot_type == robot_type:
                return device
        return None

    def find_optional_device(self, message_type: str) -> Optional[OptionalDeviceBase]:
        """
        Find optional device by message_type
        
        Args:
            message_type: Message type (e.g., 'hand_status', 'imu_data', 'gamepad_read')
            
        Returns:
            Matching optional device or None
        """
        for device in self._optional_device_list:
            if hasattr(device, 'supports_message_type') and device.supports_message_type(message_type):
                return device
        return None

    def _create_and_register_device(self, robot_type,
                                   api_up) -> Optional[DeviceBase]:
        """
        Create and register device based on robot_type
        
        Args:
            robot_type: Robot type
            **kwargs: Device constructor parameters
            
        Returns:
            Created device instance or None
        """
        device = self._device_factory.create_device_for_robot_type(
            robot_type,
            send_message_callback=self._send_down_message,
            api_up=api_up)

        if device:
            # Assign unique ID to device
            device_id = self._device_id_counter
            self._device_id_counter += 1
            
            # Add to internal device list
            self._internal_device_list.append(device)
            self._device_id_map[device_id] = device
            self._device_to_id_map[device] = device_id  # Reverse mapping
            
            self._start_device_periodic_task(device_id)

        return device

    def _create_and_register_optional_device(self, message_type: str, api_up) -> Optional[OptionalDeviceBase]:
        """
        Create and register optional device based on message_type
        
        Args:
            message_type: Message type (e.g., 'hand_status', 'imu_data', 'gamepad_read')
            api_up: API upstream data
            
        Returns:
            Created optional device instance or None
        """
        device = self._device_factory.create_optional_device(
            message_type,
            send_message_callback=self._send_down_message,
            api_up=api_up
        )

        if device:
            # Add to optional device list
            self._optional_device_list.append(device)

            # Note: Optional devices don't need periodic tasks by _read_only
            # They are updated only when data arrives
            if not device._read_only:
                device_id = self._device_id_counter
                self._device_id_counter += 1
                self._device_id_map[device_id] = device
                self._device_to_id_map[device] = device_id
                self._start_device_periodic_task(device_id)

        return device

    def _start_device_periodic_task(self, device_id: int):
        """
        Start device periodic task
        
        Args:
            device_id: Device ID
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

        # Create async task
        task = asyncio.create_task(self._device_periodic_runner(device_id))
        self._device_tasks[device_id] = task
        log_common(f"Begin periodic task for {device.name}")

    async def _device_periodic_runner(self, device_id: int):
        """
        Device periodic task runner
        
        Args:
            device_id: Device ID
        """
        device: Union[DeviceBase, OptionalDeviceBase] = self._device_id_map.get(device_id)
        if not device:
            log_err(f"Device with ID {device_id} not found in periodic runner")
            return
            
        try:
            await device._periodic()
        except asyncio.CancelledError:
            log_info(f"Periodic task for device {device.name} was cancelled")
        except Exception as e:
            log_err(f"Periodic task for device {device.name} encountered error: {e}")
        finally:
            # Clean up task reference
            if device_id in self._device_tasks:
                del self._device_tasks[device_id]

    def _check_and_cleanup_orphaned_tasks(self):
        """
        Check and clean up orphaned tasks
        
        When device instances are replaced or deleted, there may be tasks still running.
        This method will check and clean up these orphaned tasks.
        
        Returns:
            int: Number of cleaned up tasks
        """
        orphaned_count = 0
        tasks_to_remove = []
        
        # Combine all device lists for checking
        all_devices = self._internal_device_list + self._optional_device_list
        
        for device_id, task in self._device_tasks.items():
            device = self._device_id_map.get(device_id)
            if device and device not in all_devices:
                log_warn(f"Found orphaned task: device ID {device_id} ({device.name})")
                task.cancel()
                tasks_to_remove.append(device_id)
                orphaned_count += 1
        
        # Clean up orphaned tasks
        for device_id in tasks_to_remove:
            del self._device_tasks[device_id]
        
        if orphaned_count > 0:
            log_info(f"Cleaned up {orphaned_count} orphaned tasks")
        
        return orphaned_count

    def _get_orphaned_tasks_info(self):
        """
        Get information about orphaned tasks
        
        Returns:
            Dict: Information about orphaned tasks
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
        Stop all device periodic tasks
        """
        tasks_to_cancel = list(self._device_tasks.values())
        for task in tasks_to_cancel:
            task.cancel()

        if tasks_to_cancel:
            try:
                await asyncio.gather(*tasks_to_cancel, return_exceptions=True)
            except Exception as e:
                log_err(f"Error stopping device tasks: {e}")

        self._device_tasks.clear()
        log_info("All device periodic tasks have been stopped")

    def get_device_task_status(self) -> Dict[str, Any]:
        """
        Get device task status
        
        Returns:
            Dict: Device task status information
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
            # WebSocket is not connected, skip sending message
            return
        
        try:
            await self.__websocket.send(msg)
        except ConnectionClosed:
            # Connection was closed during send, this is expected
            pass
        except Exception as e:
            # Log other unexpected errors but don't raise to avoid spam
            log_err(f"Failed to send message: {e}")

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
            log_err(
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
        @brief: Start async thread, isolate async thread through this function
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
        @brief: Check if API is exiting
        @return:
            bool: True if API is exiting, False otherwise
        """
        if self.__loop is None:
            return False
        return self.__loop.is_closed()

    ## async function
    async def __async_close(self):
        """
        @brief: Close async thread
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

        # Establish WebSocket connection
        await self.__connect_ws()
        log_common("WebSocket connected.")

        task1 = asyncio.create_task(self.__periodic_data_parser())
        self.__tasks = [task1]
        await self.__shutdown_event.wait()

        # Stop all device tasks
        await self._stop_all_device_tasks()

        # Stop main tasks
        for task in self.__tasks:
            task.cancel()

        # Wait for all tasks to complete, handle cancellation exceptions
        try:
            await asyncio.gather(*self.__tasks, return_exceptions=True)
        except Exception as e:
            log_err(f"Error during task cleanup: {e}")

        log_err("HexDevice api main_loop exited.")

    async def __periodic_data_parser(self):
        """
        @brief: Periodic data parsing
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

            # Periodically check for orphaned tasks
            check_counter += 1
            if check_counter >= ORPHANED_TASK_CHECK_INTERVAL:
                check_counter = 0
                orphaned_count = self._check_and_cleanup_orphaned_tasks()
                if orphaned_count > 0:
                    log_warn(f"found {orphaned_count} orphaned tasks")

            # Get robot_type type information
            robot_type = api_up.robot_type
            robot_type_name = public_api_types_pb2.RobotType.Name(robot_type)
            # print(f"robot_type 类型: {type(robot_type)}, 值: {robot_type}, 名称: {robot_type_name}")

            # Check if robot_type is valid
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

            # Process optional fields
            self._process_optional_fields(api_up)

    def _process_optional_fields(self, api_up):
        """
        Process optional fields in APIUp message
        
        Args:
            api_up: APIUp message containing optional fields
        """
        # Define the mapping of optional field names to their data
        optional_fields = [
            ('hand_status', getattr(api_up, 'hand_status', None)),
        ]
        
        for field_name, field_data in optional_fields:
            # Check if the field has data
            if field_data is not None and self._has_optional_field(api_up, field_name):
                try:
                    # Find existing device or create new one (similar to robot_type logic)
                    optional_device = self.find_optional_device(field_name)
                    
                    if optional_device:
                        # Update existing device
                        success = optional_device._update_optional_data(field_name, field_data)
                        if not success:
                            log_warn(f"Failed to update optional device data for {field_name}")
                    else:
                        log_info(f"create new optional device: {field_name}")
                        
                        try:
                            # Create and register new optional device
                            optional_device = self._create_and_register_optional_device(field_name, api_up)
                        except Exception as e:
                            log_err(f"_create_and_register_optional_device error: {e}")
                            continue
                        
                        if optional_device:
                            # Update newly created device
                            success = optional_device._update_optional_data(field_name, field_data)
                            if not success:
                                log_warn(f"Failed to update new optional device data for {field_name}")
                        else:
                            log_warn(f"unknown optional device type: {field_name}")
                        
                except Exception as e:
                    log_err(f"Error processing optional field {field_name}: {e}")

    def _has_optional_field(self, api_up, field_name: str) -> bool:
        """
        Check if APIUp message has the specified optional field
        
        Args:
            api_up: APIUp message
            field_name: Name of the optional field
            
        Returns:
            bool: True if field exists and has data
        """
        try:
            if hasattr(api_up, 'HasField'):
                return api_up.HasField(field_name)
            else:
                # Fallback: check if attribute exists and is not None
                return hasattr(api_up, field_name) and getattr(api_up, field_name) is not None
        except Exception:
            return False

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
