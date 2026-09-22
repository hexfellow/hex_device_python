#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
"""
KCP Client Core - Pure KCP Communication Implementation
Minimal implementation focusing on KCP protocol communication only
"""

import socket
import queue
import select
import time
import threading
import ipaddress
import errno
from typing import Optional, Callable
from dataclasses import dataclass
import logging as _logging
_kcp_fallback_logger = _logging.getLogger('hex_device')

from kcp.extension import KCP


@dataclass
class KCPConfig:
    """KCP configuration parameters"""
    update_interval: int = 10
    no_delay: bool = True
    resend_count: int = 2
    no_congestion_control: bool = True
    send_window_size: int = 128
    receive_window_size: int = 128
    max_message_length: int = 2048


# Default KCP configuration instance
DEFAULT_KCP_CONFIG = KCPConfig()


class KCPClient:
    """
    Pure KCP client implementation using low-level socket operations
    
    Features:
    - Direct UDP socket management
    - A single owner thread for every KCP operation
    - Thread-safe outbound queue for application messages
    - Configurable KCP parameters
    """

    def __init__(self, config: Optional[KCPConfig] = None, logger=None):
        """
        Initialize KCP client
        
        Args:
            config: KCP configuration (uses DEFAULT_KCP_CONFIG if None)
            logger: Per-instance logger or LoggerAdapter; falls back to the
                    shared 'hex_device' logger when not provided.
        """
        self._logger = logger if logger is not None else _kcp_fallback_logger
        self.server_address = None
        self.server_port = None
        self.filter_address = None
        self._kcp = None

        # Store configuration
        self._config = config if config is not None else DEFAULT_KCP_CONFIG

        # Create UDP socket with IPv6 support (dual-stack mode)
        # AF_INET6 with IPV6_V6ONLY=0 allows both IPv4 and IPv6
        self._sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, 0)
        try:
            # Enable dual-stack mode (allows IPv4 and IPv6)
            # On some systems, this may not be supported, so we catch the error
            self._sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        except (OSError, AttributeError):
            # If dual-stack is not supported, continue with IPv6-only
            self._logger.debug(
                "[KCP Client] Dual-stack mode not supported, using IPv6-only")
        self._sock.setblocking(False)
        # Bind to IPv6 wildcard address (::) which also accepts IPv4 in dual-stack mode
        self._sock.bind(('::', 0))

        # Get actual bound port
        self.local_port = self._sock.getsockname()[1]

        # Message callback
        self._message_callback: Optional[Callable[[bytes], None]] = None

        # The C extension is not thread-safe. The IO thread exclusively owns
        # every KCP call; callers only append to this queue and wake the owner.
        self._send_queue = queue.SimpleQueue()
        self._message_queue = queue.SimpleQueue()
        self._wakeup_reader, self._wakeup_writer = socket.socketpair()
        self._wakeup_reader.setblocking(False)
        self._wakeup_writer.setblocking(False)

        # Control flags
        self._running = False
        self._io_thread: Optional[threading.Thread] = None
        self._callback_thread: Optional[threading.Thread] = None

    def config_kcp(self, address: str, port: int, conv_id: int) -> None:
        """
        Configure KCP connection with server address and parameters
        
        Args:
            address: Server IP address (IPv4, IPv6, or IPv6 with zone identifier)
            port: Server port
            conv_id: KCP conversation ID
        """
        # Create KCP instance using stored configuration
        self._kcp = KCP(
            conv_id=conv_id,
            no_delay=self._config.no_delay,
            update_interval=self._config.update_interval,
            resend_count=self._config.resend_count,
            no_congestion_control=self._config.no_congestion_control,
            send_window_size=self._config.send_window_size,
            receive_window_size=self._config.receive_window_size,
        )

        # Set KCP outbound handler
        self._kcp.include_outbound_handler(self._on_kcp_output)

        # Store original address for filtering
        self.server_address = address
        self.server_port = port
        self._set_filter(address, port)
        self._logger.debug(
            f"[KCP Client] Configured: address={address}, port={port}, conv_id={conv_id}"
        )

    def get_local_port(self) -> int:
        """
        Get the local port
        """
        return self.local_port

    def set_message_callback(self, callback: Callable[[bytes], None]) -> None:
        """
        Set callback function for received messages
        
        Args:
            callback: Function to call when message is received
                     Signature: callback(data: bytes) -> None
        """
        self._message_callback = callback

    def _send_raw_data(self, data: bytes) -> None:
        """
        Send raw data directly to UDP socket
        
        Args:
            data: Raw bytes to send
        """
        try:
            if self.server_address is None or self.server_port is None:
                self._logger.warning(f"[KCP Client] Server not started, skip sending.")
                return
            if len(data) > self._config.max_message_length:
                self._logger.error(
                    f"[KCP Client] Message length exceeds {self._config.max_message_length} bytes: {len(data)}, skip"
                )
                return

            # Parse IPv6 zone identifier (scope_id) if present
            # Format: fe80::1%eth0 or fe80::1%11
            send_address = self.server_address
            scope_id = 0
            idx = send_address.find('%')
            if idx != -1:
                zone_str = send_address[idx+1:]
                send_address = send_address[:idx]
                # Try to parse as integer first (e.g., %11), otherwise try interface name
                try:
                    scope_id = int(zone_str)
                except ValueError:
                    # Try to convert interface name to index
                    try:
                        scope_id = socket.if_nametoindex(zone_str)
                    except (OSError, AttributeError):
                        self._logger.warning(f"[KCP Client] Invalid zone identifier: {zone_str}, ignoring")
                        scope_id = 0

            # For IPv6 socket (AF_INET6), IPv4 addresses need to be converted to IPv4-mapped IPv6 format
            # This is required for dual-stack sockets to properly send to IPv4 addresses
            try:
                # Try to parse as IP address to determine type
                ip_addr = ipaddress.ip_address(send_address)
                if isinstance(ip_addr, ipaddress.IPv4Address):
                    # Convert IPv4 to IPv4-mapped IPv6 format (::ffff:x.x.x.x)
                    send_address = f"::ffff:{send_address}"
                    # IPv4-mapped addresses don't use scope_id
                    scope_id = 0
            except ValueError:
                # If not a valid IP address (e.g., hostname), use getaddrinfo to resolve
                # This ensures proper address resolution for the IPv6 socket
                try:
                    addrinfo = socket.getaddrinfo(send_address,
                                                  self.server_port,
                                                  socket.AF_INET6,
                                                  socket.SOCK_DGRAM,
                                                  socket.IPPROTO_UDP)
                    if addrinfo:
                        # Use the first resolved address (already in correct format)
                        # addrinfo[0][4] is a tuple of (host, port, flowinfo, scope_id)
                        addr_tuple = addrinfo[0][4]
                        send_address = addr_tuple[0]
                        if len(addr_tuple) >= 4 and addr_tuple[3] != 0:
                            scope_id = addr_tuple[3]
                except (socket.gaierror, OSError) as e:
                    self._logger.debug(
                        f"[KCP Client] Address resolution for {send_address} failed: {e}"
                    )
                    # If resolution fails, the original sendto will raise an error
                    # which will be caught by the outer exception handler

            # Use 4-tuple format for IPv6 with scope_id if needed
            # Format: (host, port, flowinfo, scope_id)
            if scope_id != 0:
                self._sock.sendto(data, (send_address, self.server_port, 0, scope_id))
            else:
                self._sock.sendto(data, (send_address, self.server_port))
        except Exception as e:
            self._logger.error(f"[KCP Client] Socket send error: {e}")

    def _on_kcp_output(self, kcp: KCP, data: bytes) -> None:
        """
        KCP outbound handler - called when KCP needs to send data
        
        Args:
            kcp: KCP instance
            data: Data to send
        """
        self._send_raw_data(data)

    def _receive_from_socket(self):
        """
        Receive one UDP datagram.

        Returns:
            bytes for an accepted datagram, False for a filtered datagram,
            None when the socket has nothing left to read.
        """
        try:
            data, addr = self._sock.recvfrom(self._config.max_message_length)
            # For IPv6 socket, recvfrom returns (address, port, flowinfo, scope_id) - 4-tuple
            # For IPv4 (in dual-stack mode), it returns (address, port) - 2-tuple
            # filter_address is always (address, port) - 2-tuple
            if self.filter_address is not None:
                # Extract address and port (handle both 2-tuple and 4-tuple)
                recv_addr = addr[0]
                recv_port = addr[1]
                filter_addr, filter_port = self.filter_address

                # Fast path: direct comparison (most common case - exact match)
                if recv_addr == filter_addr and recv_port == filter_port:
                    return data

                # Fallback: handle IPv4-mapped IPv6 addresses (::ffff:x.x.x.x)
                if recv_port == filter_port and recv_addr.startswith('::ffff:'):
                    if recv_addr[7:] == filter_addr:
                        return data

                return False
            return data
        except (BlockingIOError, socket.timeout):
            return None
        except Exception as e:
            if self._running:  # Only log if we're still running
                self._logger.error(f"[KCP Client] Socket receive error: {e}")
            return None

    def _drain_socket(self) -> list[bytes]:
        """Read every currently queued datagram without blocking."""
        messages = []
        while True:
            raw_data = self._receive_from_socket()
            if raw_data is None:
                return messages
            if raw_data is False:
                continue
            messages.extend(self._process_received_data(raw_data))


    def _set_filter(self, address: str, port: int) -> None:
        """
        Set filter function for received messages
        
        Args:
            address: Server IP address (may contain zone identifier)
            port: Server port
        """
        # Remove zone identifier for filtering (recvfrom returns address without zone identifier in the host field)
        # Format: [2001:db8::1%eth0] -> 2001:db8::1 or fe80::1%11 -> fe80::1
        # Optimized: only process if zone identifier exists
        idx = address.find('%')
        filter_addr = address[:idx] if idx != -1 else address
        
        # Normalize address at filter setup time to avoid repeated normalization during receive
        # This ensures IPv4 addresses are stored in standard format for efficient comparison
        # Note: We store the normalized address, but recvfrom may return IPv4-mapped format
        # The receive logic handles both cases efficiently
        self.filter_address = (filter_addr, port)

    def _process_received_data(self, raw_data: bytes) -> list[bytes]:
        """Feed one UDP datagram to KCP and return complete messages."""
        self._kcp.receive(raw_data)
        return list(self._kcp.get_all_received())

    def send(self, data: bytes) -> bool:
        """
        Queue data for the KCP owner thread.

        Args:
            data: Data to send

        Returns:
            True if accepted by the outbound queue, False otherwise.
            Acceptance is not delivery: KCP enqueue and UDP errors are
            handled by the owner thread.
        """
        if not data:
            self._logger.error("[KCP Client] Cannot send empty data")
            return False
        if len(data) > self._config.max_message_length:
            self._logger.error(
                f"[KCP Client] Message length exceeds {self._config.max_message_length} bytes: {len(data)}, skip"
            )
            return False
        if not self._running:
            self._logger.warning("[KCP Client] Client is not running, skip sending")
            return False

        self._send_queue.put(data)
        self._wake()
        return True

    def _wake(self) -> None:
        """Wake the owner thread. A full buffer already has a pending wake."""
        try:
            self._wakeup_writer.send(b"\0")
        except BlockingIOError:
            pass
        except OSError as e:
            if e.errno not in (errno.EBADF, errno.ECONNRESET, errno.EPIPE, errno.ENOTCONN):
                raise

    def _callback_loop(self) -> None:
        """Run application callbacks without delaying KCP protocol service."""
        self._logger.debug("[KCP Client] Callback loop started")
        while True:
            data = self._message_queue.get()
            if data is None:
                break
            try:
                self._message_callback(data)
            except Exception as e:
                self._logger.error(f"[KCP Client] Message callback error: {e}")
        self._logger.debug("[KCP Client] Callback loop stopped")

    def _drain_send_queue(self) -> None:
        """Move every pending application message into KCP."""
        while True:
            try:
                data = self._send_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._kcp.enqueue(data)
            except Exception as e:
                self._logger.error(f"[KCP Client] Send error: {e}")

    def _drain_wakeup(self) -> None:
        """Clear queued wake bytes after select reports the socket readable."""
        while True:
            try:
                if not self._wakeup_reader.recv(4096):
                    return
            except BlockingIOError:
                return
            except OSError:
                return

    def _next_update_timeout(self, update_interval: float) -> float:
        """Seconds until KCP next needs ikcp_update, capped by its interval."""
        try:
            delay_ms = self._kcp.update_check()
        except Exception:
            delay_ms = 0
        if not isinstance(delay_ms, (int, float)) or delay_ms < 0:
            delay_ms = 0
        return min(delay_ms / 1000.0, update_interval)

    def _service_kcp(self, messages: list) -> None:
        """Enqueue, update, and flush on the owner thread.

        update() only calls ikcp_flush on its interval. An explicit flush is
        required so newly queued data leaves immediately, including the final
        service pass after stop().
        """
        self._drain_send_queue()
        self._kcp.update()
        self._kcp.flush()
        messages.extend(self._kcp.get_all_received())

    def _io_loop(self) -> None:
        """Own the KCP control block and service RX, TX, and timers."""
        self._logger.debug("[KCP Client] IO loop started")

        update_interval = max(self._config.update_interval / 1000.0, 0.001)
        while self._running:
            messages = []
            try:
                readable, _, _ = select.select(
                    [self._sock, self._wakeup_reader], [], [],
                    self._next_update_timeout(update_interval))
                if self._wakeup_reader in readable:
                    self._drain_wakeup()
                if self._sock in readable:
                    messages.extend(self._drain_socket())

                self._service_kcp(messages)
            except Exception as e:
                if self._running:
                    self._logger.error(f"[KCP Client] IO loop error: {e}")

            for data in messages:
                self._message_queue.put(data)

        try:
            messages = []
            self._service_kcp(messages)
            for data in messages:
                self._message_queue.put(data)
        except Exception as e:
            self._logger.error(f"[KCP Client] Final flush error: {e}")

        self._logger.debug("[KCP Client] IO loop stopped")


    def start(self) -> None:
        """
        Start the single KCP owner thread.
        """
        if self._kcp is None or self._message_callback is None:
            self._logger.error("[KCP Client] KCP not configured, can not start")
            return
        elif self._running:
            self._logger.debug("[KCP Client] Already running")
            return

        self._logger.debug(f"[KCP Client] Starting, local port: {self.local_port}")
        self._running = True

        self._io_thread = threading.Thread(target=self._io_loop,
                                           daemon=True,
                                           name="KCP-IO")
        self._io_thread.start()
        self._callback_thread = threading.Thread(target=self._callback_loop,
                                                 daemon=True,
                                                 name="KCP-Callback")
        self._callback_thread.start()

        self._logger.debug("[KCP Client] Started")

    def stop(self) -> None:
        """
        Stop the owner thread and close the socket.

        Data already accepted by send() is enqueued and flushed before the
        socket is closed. In-flight KCP segments are not retransmitted after
        stop returns.
        """
        if not self._running:
            return

        self._logger.debug("[KCP Client] Stopping...")
        self._running = False
        self._wake()

        if self._io_thread and self._io_thread.is_alive():
            self._io_thread.join(timeout=1.0)
            if self._io_thread.is_alive():
                self._logger.error("[KCP Client] IO thread did not stop")
                return

        self._message_queue.put(None)
        if self._callback_thread and self._callback_thread.is_alive():
            self._callback_thread.join(timeout=1.0)
            if self._callback_thread.is_alive():
                self._logger.error("[KCP Client] Callback thread did not stop")

        for sock in (self._sock, self._wakeup_reader, self._wakeup_writer):
            try:
                sock.close()
            except OSError:
                pass

        self._logger.debug("[KCP Client] Stopped")

    def is_running(self) -> bool:
        """Check if client is running"""
        return self._running

    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()


# Example usage
if __name__ == "__main__":
    # Configuration
    SERVER_HOST = "172.23.0.5"
    SERVER_PORT = 8000
    CONV_ID = 1

    # Message callback
    def on_message(data: bytes):
        """Handle received messages"""
        print(f"[Main] Received: {data}")

    # Create custom KCP configuration (optional)
    custom_config = KCPConfig(
        update_interval=10,
        no_delay=True,
        resend_count=2,
        no_congestion_control=True,
        send_window_size=128,
        receive_window_size=128,
        max_message_length=2048
    )

    # Create and configure client
    client = KCPClient(config=custom_config)  # Or use KCPClient() for default config
    client.config_kcp(SERVER_HOST, SERVER_PORT, CONV_ID)

    # Set message callback
    client.set_message_callback(on_message)

    # Start client
    client.start()

    try:
        # Send some test messages
        for i in range(5):
            message = f"Hello {i}".encode()
            if client.send(message):
                print(f"[Main] Sent: {message}")
            time.sleep(1)

        # Keep running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user")
    finally:
        client.stop()
