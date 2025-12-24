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
import time
import threading
import ipaddress
from typing import Optional, Callable
from dataclasses import dataclass
from .common_utils import log_err, log_info, log_debug, log_warn

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
    - KCP protocol handling
    - Separate threads for update and receive loops
    - Configurable KCP parameters
    """

    def __init__(self, config: Optional[KCPConfig] = None):
        """
        Initialize KCP client
        
        Args:
            config: KCP configuration (uses DEFAULT_KCP_CONFIG if None)
        """
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
            log_debug(
                "[KCP Client] Dual-stack mode not supported, using IPv6-only")

        self._sock.settimeout(0.5)
        # Bind to IPv6 wildcard address (::) which also accepts IPv4 in dual-stack mode
        self._sock.bind(('::', 0))

        # Get actual bound port
        self.local_port = self._sock.getsockname()[1]

        # Message callback
        self._message_callback: Optional[Callable[[bytes], None]] = None

        # Control flags
        self._running = False
        self._update_thread: Optional[threading.Thread] = None
        self._receive_thread: Optional[threading.Thread] = None

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
        log_debug(
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
                log_warn(f"[KCP Client] Server not started, skip sending.")
                return
            if len(data) > self._config.max_message_length:
                log_err(
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
                        log_warn(f"[KCP Client] Invalid zone identifier: {zone_str}, ignoring")
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
                    log_debug(
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
            log_err(f"[KCP Client] Socket send error: {e}")

    def _on_kcp_output(self, kcp: KCP, data: bytes) -> None:
        """
        KCP outbound handler - called when KCP needs to send data
        
        Args:
            kcp: KCP instance
            data: Data to send
        """
        self._send_raw_data(data)

    def _receive_from_socket(self) -> Optional[bytes]:
        """
        Receive raw data from UDP socket
        
        Returns:
            Received bytes or None if error
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
                if recv_addr == filter_addr:
                    if recv_port == filter_port:
                        return data
                    return None
                
                # Fallback: handle IPv4-mapped IPv6 addresses (::ffff:x.x.x.x)
                # Only check if port matches and recv_addr is IPv4-mapped format
                if recv_port == filter_port and recv_addr.startswith('::ffff:'):
                    # Compare IPv4 part (skip '::ffff:' prefix, 7 chars)
                    if recv_addr[7:] == filter_addr:
                        return data
                
                return None
            return data
        except socket.timeout:
            return None
        except Exception as e:
            if self._running:  # Only log if we're still running
                log_err(f"[KCP Client] Socket receive error: {e}")
            return None

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

    def _process_received_data(self, raw_data: bytes) -> None:
        """
        Process received raw data through KCP protocol
        
        Args:
            raw_data: Raw bytes received from socket
        """
        # Feed data to KCP
        self._kcp.receive(raw_data)

        # Get all processed packets from KCP
        for data in self._kcp.get_all_received():
            # Call user callback if set
            if self._message_callback:
                try:
                    self._message_callback(data)
                except Exception as e:
                    log_err(f"[KCP Client] Message callback error: {e}")

    def send(self, data: bytes) -> bool:
        """
        Send data through KCP
        
        Args:
            data: Data to send
            
        Returns:
            True if successfully enqueued, False otherwise
        """
        try:
            # Enqueue to KCP
            self._kcp.enqueue(data)

            # Flush to trigger immediate send
            self._kcp.flush()

            return True
        except Exception as e:
            log_err(f"[KCP Client] Send error: {e}")
            return False

    def _update_loop(self) -> None:
        """
        KCP update loop - runs in separate thread
        Handles KCP protocol state updates and retransmissions
        """
        log_debug("[KCP Client] Update loop started")

        while self._running:
            try:
                # Update KCP state
                self._kcp.update()

                # Calculate next update time
                sleep_time = self._kcp.update_check() / 1000.0

                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Minimum sleep to prevent CPU spinning
                    time.sleep(0.0001)

            except Exception as e:
                if self._running:
                    log_err(f"[KCP Client] Update loop error: {e}")

        log_debug("[KCP Client] Update loop stopped")

    def _receive_loop(self) -> None:
        """
        Socket receive loop - runs in separate thread
        Continuously receives data from socket and processes through KCP
        """
        log_debug("[KCP Client] Receive loop started")

        while self._running:
            try:
                # Receive raw data from socket, timeout after 0.5 seconds
                raw_data = self._receive_from_socket()

                if raw_data:
                    # Process through KCP
                    self._process_received_data(raw_data)

            except Exception as e:
                if self._running:
                    log_err(f"[KCP Client] Receive loop error: {e}")

        log_debug("[KCP Client] Receive loop stopped")

    def start(self) -> None:
        """
        Start KCP client
        Launches update and receive threads
        """
        if self._kcp is None or self._message_callback is None:
            log_err("[KCP Client] KCP not configured, can not start")
            return
        elif self._running:
            log_debug("[KCP Client] Already running")
            return

        log_debug(f"[KCP Client] Starting, local port: {self.local_port}")
        self._running = True

        # Start update thread
        self._update_thread = threading.Thread(target=self._update_loop,
                                               daemon=True,
                                               name="KCP-Update")
        self._update_thread.start()

        # Start receive thread
        self._receive_thread = threading.Thread(target=self._receive_loop,
                                                daemon=True,
                                                name="KCP-Receive")
        self._receive_thread.start()

        log_debug("[KCP Client] Started")

    def stop(self) -> None:
        """
        Stop KCP client
        Stops all threads and closes socket
        """
        if not self._running:
            return

        log_debug("[KCP Client] Stopping...")
        self._running = False

        # Wait for threads to finish
        if self._update_thread:
            self._update_thread.join(timeout=1.0)
        if self._receive_thread:
            self._receive_thread.join(timeout=1.0)

        # Close socket
        try:
            self._sock.close()
        except:
            pass

        log_debug("[KCP Client] Stopped")

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
        log_info(f"[Main] Received: {data}")

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
                log_info(f"[Main] Sent: {message}")
            time.sleep(1)

        # Keep running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        log_info("\n[Main] Interrupted by user")
    finally:
        client.stop()
