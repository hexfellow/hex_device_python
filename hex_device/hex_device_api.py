#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .common_utils import is_valid_ws_url, InvalidWSURLException, delay, log_debug
from .common_utils import log_warn, log_info, log_err, log_common
from .error_type import WsError, ProtocolError
from .device_base import DeviceBase
from .device_factory import DeviceFactory
from .device_base_optional import OptionalDeviceBase
from .motor_base import Timestamp
from .hex_socket import HexSocketParser, HexSocketOpcode
from .kcp_client_core import KCPClient, KCPConfig
from .generated.version import CURRENT_PROTOCOL_MAJOR_VERSION, CURRENT_PROTOCOL_MINOR_VERSION
from . import __version__

import time
import os
import asyncio
import threading
import websockets
import socket
from urllib.parse import urlparse
from typing import Optional, Tuple, List, Type, Dict, Any, Union
from websockets.exceptions import ConnectionClosed

RAW_DATA_LEN = 50
ORPHANED_TASK_CHECK_INTERVAL = 100

class ReportFrequency:
    """
    Report frequency
    """
    Rf1000Hz = 0
    Rf500Hz = 3
    Rf250Hz = 4
    Rf100Hz = 1
    Rf50Hz = 2
    Rf1Hz = 5

class HexDeviceApi:
    """
    @brief: HexDeviceApi provides an API interface for HexDevice to communicate with WebSocket.
    @params:
        ws_url: the url of the websocket server
        control_hz: the frequency of the control loop
    """

    @staticmethod
    def _get_report_frequency_from_control_hz(control_hz: int) -> int:
        """
        Get report frequency based on control_hz (round down to nearest available frequency)
        
        Args:
            control_hz: Control frequency in Hz
            
        Returns:
            Report frequency enum value
        """
        # Available frequencies from high to low: 1000, 500, 250, 100, 50, 1
        if control_hz >= 1000:
            return ReportFrequency.Rf1000Hz
        elif control_hz >= 500:
            return ReportFrequency.Rf500Hz
        elif control_hz >= 250:
            return ReportFrequency.Rf250Hz
        elif control_hz >= 100:
            return ReportFrequency.Rf100Hz
        elif control_hz >= 50:
            return ReportFrequency.Rf50Hz
        else:
            return ReportFrequency.Rf1Hz

    def __init__(self, ws_url: str, control_hz: int = 500, enable_kcp: bool = True, local_port: int = None):
        # protocol version
        self.protocol_major_version = CURRENT_PROTOCOL_MAJOR_VERSION
        self.protocol_minor_version = CURRENT_PROTOCOL_MINOR_VERSION
        log_info(f"HexDeviceApi: Protocol version: {self.protocol_major_version}.{self.protocol_minor_version}, package version: {__version__}")

        # variables init
        self.ws_url = ws_url
        try:
            self.__ws_url, _ = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            log_err("Invalid WebSocket URL: " + str(e))
        self.parsed_url = urlparse(self.__ws_url)
        self.local_port = local_port  # Local port to bind tcp socket (None for random port)
        self.enable_kcp = enable_kcp

        self.__kcp_client: Optional[KCPClient] = None
        self.__kcp_parser = HexSocketParser()
        self.__websocket = None
        self.__raw_data = []  ## raw data buffer
        self.__control_hz = control_hz
        self.__report_frequency = self._get_report_frequency_from_control_hz(control_hz)
        log_info(f"Your target frequency is {control_hz}Hz, the report frequency was set to {public_api_types_pb2.ReportFrequency.Name(self.__report_frequency)}Hz now.")

        # time synchronization
        self.__use_ptp = self.__select_time_source()
        self.__time_bias = None

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
        self._device_tasks = {}  # Store device IDs and their corresponding futures (from run_coroutine_threadsafe)
        
        # Counter for orphaned task checking
        self._check_counter = 0  # Global counter for tracking function calls
        self._process_lock = threading.Lock()  # Thread lock for _process_api_up
        
        # # Frequency tracking for _process_api_up
        # self._process_api_up_call_count = 0  # Total call count
        # self._process_api_up_start_time = time.time()  # Start time for frequency calculation
        # self._process_api_up_last_print_time = time.time()  # Last time we printed frequency
        # self._process_api_up_print_interval = 5.0  # Print frequency every 5 seconds

        self.__shutdown_event = None  # the handle event for shutdown api
        self.__loop = None  ## async loop thread
        self.__loop_thread = threading.Thread(target=self.__loop_start,
                                              daemon=True)
        self.__is_closing = threading.Event()
        # init api
        self.__loop_thread.start()
        if self.local_port:
            log_info(f"HexDeviceApi initialized (local port: {self.local_port}).")
        else:
            log_info(f"HexDeviceApi initialized.")

    # Device interface
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
            log_debug("Registered Chassis device class")
        except ImportError as e:
            log_warn(f"Unable to import Chassis: {e}")

        try:
            from .arm import Arm
            self._register_device_class(Arm)
            log_debug("Registered Arm device class")
        except ImportError as e:
            log_warn(f"Unable to import Arm: {e}")

        try:
            from .linear_lift import LinearLift
            self._register_device_class(LinearLift)
            log_debug("Registered LinearLift device class")
        except ImportError as e:
            log_warn(f"Unable to import LinearLift: {e}")

        try:
            from .zeta_lift import ZetaLift
            self._register_device_class(ZetaLift)
            log_debug("Registered ZetaLift device class")
        except ImportError as e:
            log_warn(f"Unable to import ZetaLift: {e}")

        try:
            from .hands import Hands
            # Register Hands for all supported device types (with duplicate check)
            registered_count = 0
            for device_type in Hands.SUPPORTED_DEVICE_TYPE:
                # Check if already registered to avoid duplicate registration
                if device_type not in self._device_factory._optional_device_classes:
                    self._register_optional_device_class(device_type, Hands)
                    registered_count += 1
            log_debug(f"Registered Hands optional device class for {registered_count} new device types out of {len(Hands.SUPPORTED_DEVICE_TYPE)} total: {[dt for dt in Hands.SUPPORTED_DEVICE_TYPE]}")
        except ImportError as e:
            log_warn(f"Unable to import Hands: {e}")

        try:
            from .imu import Imu
            for device_type in Imu.SUPPORTED_DEVICE_TYPE:
                self._register_optional_device_class(device_type, Imu)
            log_debug("Registered Imu optional device class")
        except ImportError as e:
            log_warn(f"Unable to import Imu: {e}")
            
        try:
            from .gamepad import Gamepad
            for device_type in Gamepad.SUPPORTED_DEVICE_TYPE:
                self._register_optional_device_class(device_type, Gamepad)
            log_debug("Registered Gamepad optional device class")
        except ImportError as e:
            log_warn(f"Unable to import Gamepad: {e}")

        try:
            from .sdt_hello import SdtHello
            for device_type in SdtHello.SUPPORTED_DEVICE_TYPE:
                self._register_optional_device_class(device_type, SdtHello)
            log_debug("Registered SdtHello optional device class")
        except ImportError as e:
            log_warn(f"Unable to import SdtHello: {e}")

        # TODO: Add registration for more device classes
        # lift、rotate lift...

    def _register_device_class(self, device_class):
        """
        Register device class to factory
        
        Args:
            device_class: Device class
        """
        self._device_factory.register_device_class(device_class)

    def _register_optional_device_class(self, device_type: int, device_class):
        """
        Register optional device class to factory

        Args:
            device_type: Device type (SecondaryDeviceType)
            device_class: Optional device class
        """
        self._device_factory.register_optional_device_class(device_type, device_class)

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
            self.__control_hz,
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

    def _create_and_register_optional_device(self, device_id: int, device_type, secondary_device_status) -> Optional[OptionalDeviceBase]:
        """
        Create and register optional device based on device_id and device_type
        
        Args:
            device_id: Device ID from SecondaryDeviceStatus
            device_type: Device type
            secondary_device_status: The SecondaryDeviceStatus object
            
        Returns:
            Created optional device instance or None
        """
        # init optional device
        device = self._device_factory.create_optional_device(
            device_id,
            device_type,
            secondary_device_status=secondary_device_status,
            control_hz=self.__control_hz,
            send_message_callback=self._send_down_message,
        )

        if device:
            # Add to optional device list
            self._optional_device_list.append(device)

            # Note: Optional devices don't need periodic tasks by _read_only
            # They are updated only when data arrives
            if not device._read_only:
                internal_device_id = self._device_id_counter
                self._device_id_counter += 1
                self._device_id_map[internal_device_id] = device
                self._device_to_id_map[device] = internal_device_id
                self._start_device_periodic_task(internal_device_id)

        return device

    # Device task management
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

        # Create async task using run_coroutine_threadsafe to handle cross-thread scheduling
        if self.__loop:
            future = asyncio.run_coroutine_threadsafe(
                self._device_periodic_runner(device_id), 
                self.__loop
            )
            self._device_tasks[device_id] = future
            log_common(f"Begin periodic task for {device.name}")
        else:
            log_err(f"Event loop not available, cannot start periodic task for {device.name}")

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
            log_debug(f"Periodic task for device {device.name} was cancelled")
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
                log_debug(f"Found orphaned task: device ID {device_id} ({device.name})")
                task.cancel()
                tasks_to_remove.append(device_id)
                orphaned_count += 1
        
        # Clean up orphaned tasks
        for device_id in tasks_to_remove:
            del self._device_tasks[device_id]
        
        if orphaned_count > 0:
            log_debug(f"Cleaned up {orphaned_count} orphaned tasks")
        
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
        for future in tasks_to_cancel:
            future.cancel()

        # Wait a bit for cancellations to complete
        if tasks_to_cancel:
            try:
                await asyncio.sleep(0.1)  # Give time for cancellation to propagate
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

    # Message function
    def _is_support_version(self, api_up) -> bool:
        """
        Check if the protocol version is supported
        @return:
            bool: True if protocol version is supported, False otherwise
        """
        if not hasattr(api_up, 'protocol_major_version'):
            log_err("Your hardware version is too lower!!! please use hex_device v1.2.2 or lower.")
            log_err("Your hardware version is too lower!!! please use hex_device v1.2.2 or lower.")
            log_err("Your hardware version is too lower!!! please use hex_device v1.2.2 or lower.")
            return False
        else:
            version = api_up.protocol_major_version
            if version < 1.0:
                log_err(f"The hardware firmware version is too low({version})!!! Please use a lower version of hex_device.")
                log_err(f"The hardware firmware version is too low({version})!!! Please use a lower version of hex_device.")
                log_err(f"The hardware firmware version is too low({version})!!! Please use a lower version of hex_device.")
                return False
        return True

    def _construct_enable_kcp_message(self, client_port: int) -> public_api_down_pb2.APIDown:
        """
        Construct enable KCP message
        Args:
            local_port: The local port of the KCP client
        Returns:
            Enable KCP message
        """
        default_config = KCPConfig()
        msg = public_api_down_pb2.APIDown()
        kcp_command = public_api_types_pb2.EnableKcp()
        kcp_config = public_api_types_pb2.KcpConfig()
        kcp_command.client_peer_port = client_port
        kcp_config.window_size_snd_wnd = default_config.send_window_size
        kcp_config.window_size_rcv_wnd = default_config.receive_window_size
        kcp_config.interval_ms = default_config.update_interval
        kcp_config.no_delay = default_config.no_delay
        kcp_config.nc = default_config.no_congestion_control
        kcp_config.resend = default_config.resend_count
        kcp_command.kcp_config.CopyFrom(kcp_config)
        msg.enable_kcp.CopyFrom(kcp_command)
        return msg

    def _construct_tcp_report_frequency_message(self, report_frequency) -> public_api_down_pb2.APIDown:
        """
        Construct TCP report frequency message
        """
        msg = public_api_down_pb2.APIDown()
        msg.set_report_frequency = report_frequency
        return msg

    def _construct_kcp_start_message(self) -> public_api_down_pb2.APIDown:
        """
        Construct KCP start message
        """
        msg = public_api_down_pb2.APIDown()
        msg.placeholder_message = True
        return msg

    async def _send_down_message(self, data: public_api_down_pb2.APIDown):
        msg = data.SerializeToString()
        
        if not self.enable_kcp or self.__kcp_client is None:
            if self.__websocket is None:
                log_warn("TCP connection is not established, skipping message send")
                return
            
            try:
                await self.__websocket.send(msg)
            except ConnectionClosed:
                log_err("WebSocket connection was closed during message send, please check your network connection and restart the server again.")
                self.close()
            except Exception as e:
                log_err(f"Failed to send message via TCP: {e}")
                
        elif self.enable_kcp and self.__kcp_client is not None:
            try:
                frame = HexSocketParser.create_header(msg, HexSocketOpcode.Binary)
                self.__kcp_client.send(frame)
            except Exception as e:
                log_err(f"Failed to send message via KCP: {e}")

    async def __capture_data_frame_from_websocket(self, raise_on_timeout: bool = False) -> Optional[public_api_up_pb2.APIUp]:
        """
        @brief: Continuously monitor WebSocket connections until:
        1. Received a valid binary Protobuf message
        2. Protocol error occurred
        3. Connection closed
        4. No data due to timeout
        
        @param raise_on_timeout: If True, raise TimeoutError instead of continuing on timeout
        @return:
            base_backend.APIUp object or None
        """
        while True:
            try:
                # Check if websocket is connected
                if self.__websocket is None:
                    if self.__is_closing.is_set():
                        return None
                    log_err("WebSocket is disconnected")
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

                else:
                    log_warn(f"Received unexpected message type: {type(message)}, content: {message}")
                    continue

            except asyncio.TimeoutError:
                if raise_on_timeout:
                    # Re-raise timeout for caller to handle
                    raise
                else:
                    log_err("No data received for 3 seconds")
                    continue

            except ConnectionClosed as e:
                if self.__is_closing.is_set():
                    return
                log_err(
                    f"Connection closed (code: {e.code}, reason: {e.reason})")
                try:
                    await self.__reconnect_ws()
                    continue
                except ConnectionError as e:
                    log_err(f"Reconnect failed: {e}")
                    self.close()

            except Exception as e:
                log_err(f"Unknown error: {str(e)}")
                raise WsError("Unexpected error") from e

    # Websocket function
    def _create_socket_with_nodelay(self, host: str, port: int, local_port: int = None) -> socket.socket:
        """
        Create a connected socket with TCP_NODELAY and TCP_QUICKACK for fast retransmission
        
        Args:
            host: Target host
            port: Target port
            local_port: Local port to bind (None for random port)
            
        Returns:
            Connected socket with TCP optimizations for fast retransmission
        """
        if local_port is not None and local_port != 0:
            # Create socket manually and bind to specific local port
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                # Allow reuse of local address to avoid "Address already in use" errors
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # Bind to specific local port (0.0.0.0 means any local interface)
                sock.bind(('::', local_port))
                log_debug(f"Socket bound to local port {local_port}")
                # Connect to remote host
                sock.connect((host, port))
            except OSError as e:
                sock.close()
                log_err(f"Failed to bind to local port {local_port}: {e}")
                raise
        else:
            # Use default behavior (random local port)
            sock = socket.create_connection((host, port))
        
        # Enable TCP_NODELAY to disable Nagle's algorithm for low latency
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # Enable TCP_QUICKACK for fast retransmission
        # This enables quick acknowledgments which helps with fast retransmission
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
            log_debug("TCP_QUICKACK enabled")
        except (OSError, AttributeError):
            # TCP_QUICKACK may not be available on all platforms (Linux-specific)
            log_warn("TCP_QUICKACK not supported on this platform")
        
        return sock

    async def __connect_ws(self):
        """
        @brief: Connect to the device server.
        """
        try:
            # Create socket with TCP_NODELAY and TCP_QUICKACK for fast retransmission
            sock = self._create_socket_with_nodelay(self.parsed_url.hostname, self.parsed_url.port, self.local_port)
            self.__websocket = await websockets.connect(self.__ws_url,
                                                        ping_interval=20,
                                                        ping_timeout=60,
                                                        close_timeout=5,
                                                        sock=sock)
            log_debug("WebSocket connection established.")
        except Exception as e:
            log_err(f"Failed to open WebSocket connection: {e}")
            log_err(
                "Public API haved exited, please check your network connection and restart the server again."
            )
            exit(1)

    async def __reconnect_ws(self):
        """
        @brief: Reconnect to the device server.
        """
        retry_count = 0
        max_retries = 3
        base_delay = 1

        while retry_count < max_retries:
            try:
                if self.__websocket:
                    await self.__websocket.close()
                # Create socket with TCP_NODELAY and TCP_QUICKACK for fast retransmission
                sock = self._create_socket_with_nodelay(self.parsed_url.hostname, self.parsed_url.port, self.local_port)
                self.__websocket = await websockets.connect(self.__ws_url,
                                                            ping_interval=20,
                                                            ping_timeout=60,
                                                            close_timeout=5,
                                                            sock=sock)
                log_info(f"Successfully reconnected using WebSocket protocol")
                return
            except Exception as e:
                delay = base_delay * (2**retry_count)
                log_warn(
                    f"Reconnect failed (attempt {retry_count+1}) using WebSocket: {e}, retrying in {delay}s"
                )
                await asyncio.sleep(delay)
                retry_count += 1
        raise ConnectionError("Maximum reconnect retries exceeded")

    def __select_time_source(self) -> bool:
        """
        Select time source
        """
        value = os.getenv('HEX_PTP_CLOCK')
        if value is None:
            return False
        else:
            log_info("PTP time synchronization is enabled.")
            return True

    def __sync_monotonic_time(self, timestamp: Timestamp) -> Timestamp:
        """
        Sync monotonic time
        
        Args:
            timestamp: Timestamp object to sync
            
        Returns:
            Synced Timestamp object
        """
        if self.__use_ptp:
            return timestamp
        else:
            if self.__time_bias is not None:
                # Add time bias to timestamp
                synced_ns = timestamp.to_ns() + self.__time_bias
                return Timestamp.from_ns(synced_ns)
            else:
                # Calculate time bias and return current time
                current_time_ns = time.perf_counter_ns()
                self.__time_bias = current_time_ns - timestamp.to_ns()
                return Timestamp.from_ns(current_time_ns)

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

    def _process_kcp_data(self, data: bytes):
        """
        Process KCP data
        @param data: KCP data to process
        @return:
            None
        """
        result = self.__kcp_parser.parse(data)
        if result is not None:
            for opcode, payload in result:
                if opcode == HexSocketOpcode.Binary:
                    api_up = public_api_up_pb2.APIUp()
                    api_up.ParseFromString(payload)
                    self._process_api_up(api_up)
                elif opcode == HexSocketOpcode.Text:
                    log_common(f"kcp text message: {payload}")
                else:
                    log_warn(f"unsupported opcode: {opcode} from kcp")

    def _process_api_up(self, api_up):
        """
        Process APIUp message (thread-safe with lock)
        @param api_up: APIUp message to process
        @return:
            None
        """
        # Acquire lock to ensure thread safety
        with self._process_lock:
            # # Track call frequency
            # self._process_api_up_call_count += 1
            # current_time = time.time()
            # elapsed_time = current_time - self._process_api_up_start_time
            
            # # Print frequency periodically
            # if current_time - self._process_api_up_last_print_time >= self._process_api_up_print_interval:
            #     if elapsed_time > 0:
            #         frequency = self._process_api_up_call_count / elapsed_time
            #         log_info(f"_process_api_up 调用频率: {frequency:.2f} Hz (总调用次数: {self._process_api_up_call_count}, 运行时间: {elapsed_time:.2f}秒)")
            #     self._process_api_up_last_print_time = current_time
            
            if len(self.__raw_data) >= RAW_DATA_LEN:
                self.__raw_data.pop(0)
            self.__raw_data.append(api_up)

            # Periodically check for orphaned tasks
            self._check_counter += 1
            if self._check_counter >= ORPHANED_TASK_CHECK_INTERVAL:
                self._check_counter = 0
                orphaned_count = self._check_and_cleanup_orphaned_tasks()
                if orphaned_count > 0:
                    log_debug(f"found {orphaned_count} orphaned tasks")

            if api_up.HasField('log'):
                log_info(f"Get log from server: {api_up.log}")

            if api_up.HasField('time_stamp'):
                if self.__use_ptp:
                    ptp_timestamp = api_up.time_stamp.ptp_time_stamp
                    if ptp_timestamp.calibrated:
                        timestamp = Timestamp.from_s_ns(ptp_timestamp.seconds, ptp_timestamp.nanoseconds)
                    else:
                        log_debug("PTP time is not calibrated.")
                        input_timestamp = Timestamp.from_s_ns(ptp_timestamp.seconds, ptp_timestamp.nanoseconds)
                        timestamp = self.__sync_monotonic_time(input_timestamp)
                else:
                    monotonic_timestamp = api_up.time_stamp.monotonic_time_stamp
                    input_timestamp = Timestamp.from_s_ns(monotonic_timestamp.seconds, monotonic_timestamp.nanoseconds)
                    timestamp = self.__sync_monotonic_time(input_timestamp)
            else:
                timestamp = Timestamp.from_ns(time.perf_counter_ns())

            # Get robot_type type information
            robot_type = api_up.robot_type
            robot_type_name = public_api_types_pb2.RobotType.Name(robot_type)
            # print(f"robot_type 类型: {type(robot_type)}, 值: {robot_type}, 名称: {robot_type_name}")

            # Check if robot_type is valid
            if isinstance(api_up.robot_type, int):
                device = self.find_device_by_robot_type(robot_type)

                if device:
                    device._update(api_up, timestamp)
                else:
                    log_debug(f"create new device: {robot_type_name}")

                    try:
                        device = self._create_and_register_device(
                            robot_type, api_up)
                    except Exception as e:
                        log_err(f"_create_and_register_device error: {e}")
                        return

                    if device:
                        device._update(api_up, timestamp)
                    else:
                        log_warn(f"unknown device type: {robot_type_name}")
            else:
                return

            # Process optional fields
            self._process_optional_fields(api_up, timestamp)

    def _process_optional_fields(self, api_up, timestamp: Timestamp):
        """
        Process SecondaryDeviceStatus array in APIUp message
        
        Args:
            api_up: APIUp message containing secondary_device_status array
        """
        # Process SecondaryDeviceStatus array
        if hasattr(api_up, 'secondary_device_status') and api_up.secondary_device_status:
            for secondary_device in api_up.secondary_device_status:
                try:
                    device_id = secondary_device.device_id
                    # Determine message type from the oneof status field
                    device_type = secondary_device.device_type
                    
                    if device_type:
                        # Find existing device by device_id
                        optional_device = self.find_optional_device_by_id(device_id)
                        
                        if optional_device:
                            # Update existing device
                            success = optional_device._update_optional_data(device_type, secondary_device, timestamp)
                            if not success:
                                log_err(f"Failed to update optional device data for device_id {device_id}, type {device_type}")
                        else:
                            log_debug(f"create new optional device: device_id={device_id}, type={device_type}")
                            
                            try:
                                # Create and register new optional device
                                optional_device = self._create_and_register_optional_device(device_id, device_type, secondary_device)
                            except Exception as e:
                                log_err(f"_create_and_register_optional_device_by_id error: {e}")
                                continue
                            
                            if optional_device:
                                # Update newly created device
                                success = optional_device._update_optional_data(device_type, secondary_device, timestamp)
                                if not success:
                                    log_warn(f"Failed to update new optional device data for device_id {device_id}, type {device_type}")
                            else:
                                log_debug(f"unknown optional device type: device_id={device_id}, type={device_type}")
                    
                except Exception as e:
                    log_err(f"Error processing secondary device {getattr(secondary_device, 'device_id', 'unknown')}: {e}")

    ## async function
    async def __async_close(self):
        """
        @brief: Close async thread and connection
        @return:
            None
        """
        self.__is_closing.set()
        if self.__shutdown_event is not None:
                self.__shutdown_event.set()

        try:
            # Close KCP connection using stop() method
            if self.__kcp_client is not None:
                self.__kcp_client.stop()
                self.__kcp_client = None
                log_info("KCP connection closed successfully")

            # Close WebSocket connection
            if self.__websocket:
                ws = self.__websocket
                self.__websocket = None
                await ws.close()
                log_info("WebSocket connection closed successfully")
            
        except Exception as e:
            log_err(f"Error closing connection: {e}")

    async def __main_loop(self):
        self.__shutdown_event = asyncio.Event()
        log_common("HexDevice Api started.")

        await self.__connect_ws()

        task1 = asyncio.create_task(self.__websocket_data_parser())
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

    async def __websocket_data_parser(self):
        """
        @brief: Periodic data parsing
        @return:
            None
        """
        # Check if the protocol version is supported
        try:
            api_up = await self.__capture_data_frame_from_websocket()
            if not self._is_support_version(api_up):
                self.close()
                return
        except Exception as e:
            log_err(f"__websocket_data_parser error: {e}")
            self.close()
            return

        # try to connect kcp connection
        if self.enable_kcp:
            kcp_client = KCPClient()
            client_port = kcp_client.get_local_port()
            msg = self._construct_enable_kcp_message(client_port)
            await self._send_down_message(msg)

            kcp_init = False
            while not kcp_init:
                try:
                    # Set raise_on_timeout=True to catch timeout and resend enable_kcp message
                    api_up = await self.__capture_data_frame_from_websocket(raise_on_timeout=True)

                    if api_up.HasField('kcp_server_status'):
                        server_port = api_up.kcp_server_status.server_port
                        session_id = api_up.session_id
                        kcp_client.config_kcp(self.parsed_url.hostname, server_port, session_id)
                        kcp_client.set_message_callback(self._process_kcp_data)
                        kcp_client.start()
                        # set report frequency to 1Hz
                        msg = self._construct_tcp_report_frequency_message(ReportFrequency.Rf1Hz)
                        await self._send_down_message(msg)
                        kcp_init = True
                    # If received other messages, just ignore and continue waiting
                    
                except asyncio.TimeoutError:
                    log_debug("Waiting for KCP server status, resending enable_kcp message...")
                    msg = self._construct_enable_kcp_message(client_port)
                    await self._send_down_message(msg)

            # kcp init finished
            self.__kcp_client = kcp_client
            log_debug(f"kcp client initialized, session_id={session_id}")
            # send a start message to kcp
            msg = self._construct_kcp_start_message()
            await self._send_down_message(msg)
            
        # set report frequency to target frequency
        msg = self._construct_tcp_report_frequency_message(self.__report_frequency)
        await self._send_down_message(msg)

        # Begin to parse the data
        while True:
            try:
                api_up = await self.__capture_data_frame_from_websocket()
                if api_up is None and self.__is_closing.is_set():
                    return
            except Exception as e:
                log_err(f"__websocket_data_parser error: {e}")
                continue
            if not self.enable_kcp:
                self._process_api_up(api_up)

    # User api
    async def reset_report_frequency(self, report_frequency: int):
        """
        Reset report frequency to target frequency
        """
        msg = self._construct_tcp_report_frequency_message(report_frequency)
        await self._send_down_message(msg)

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

    def find_optional_device_by_id(self, device_id: int) -> Optional[OptionalDeviceBase]:
        """
        Find optional device by device_id
        
        Args:
            device_id: Device ID from SecondaryDeviceStatus
            
        Returns:
            Matching optional device or None
        """
        for device in self._optional_device_list:
            if hasattr(device, 'device_id') and device.device_id == device_id:
                return device
        return None

    def find_optional_device_by_robot_type(self, robot_type) -> Optional[List[OptionalDeviceBase]]:
        """
        Find optional device by robot_type
        
        Args:
            robot_type: Robot type
            
        Returns:
            Matching optional device or None
        """
        devices = []
        for device in self._optional_device_list:
            if hasattr(device, 'device_type') and device.device_type == robot_type:
                devices.append(device)
        if len(devices) == 0:
            return None
        return devices

    def close(self):
        if self.__loop and self.__loop.is_running():
            try:
                # Close all devices
                for device in self._internal_device_list:
                    if hasattr(device, 'stop'):
                        device.stop()
                for device in self._optional_device_list:
                    if hasattr(device, 'stop'):
                        device.stop()
            except Exception as e:
                log_warn(f"Error closing devices: {e}")
            time.sleep(0.3)
            
            log_warn("HexDevice API is closing...")
            try:
                # Submit the async close task and wait for it to complete
                future = asyncio.run_coroutine_threadsafe(self.__async_close(), self.__loop)
                # Wait for the task to complete with a timeout
                try:
                    future.result(timeout=5.0)
                except TimeoutError:
                    log_warn("__async_close timed out after 5 seconds")
                except Exception as e:
                    log_err(f"Error waiting for __async_close: {e}")
            except Exception as e:
                log_err(f"Error submitting __async_close task: {e}")
                import traceback
                log_err(f"Traceback: {traceback.format_exc()}")

    def is_api_exit(self) -> bool:
        """
        @brief: Check if API is exiting
        @return:
            bool: True if API is exiting, False otherwise
        """
        if self.__loop is None:
            return False
        return self.__loop.is_closed()

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
