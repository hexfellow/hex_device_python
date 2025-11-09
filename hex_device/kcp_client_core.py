#!/usr/bin/env python3
"""
KCP Client Core - Pure KCP Communication Implementation
Minimal implementation focusing on KCP protocol communication only
"""


## todo：检查代发送消息长度

import socket
import time
import threading
import struct
from typing import Optional, Callable

from kcp.extension import KCP

# Default KCP configuration
DEFAULT_UPDATE_INTERVAL = 10
DEFAULT_NO_DELAY = True
DEFAULT_RESEND_COUNT = 2
DEFAULT_NO_CONGESTION_CONTROL = True
DEFAULT_SEND_WINDOW_SIZE = 128
DEFAULT_RECEIVE_WINDOW_SIZE = 128


class KCPClient:
    """
    Pure KCP client implementation using low-level socket operations
    
    Features:
    - Direct UDP socket management
    - KCP protocol handling
    - Separate threads for update and receive loops
    - Configurable KCP parameters
    """
    
    def __init__(
        self,
        server_address: str,
        server_port: int,
        conv_id: int,
        local_port: int = 0,
        # KCP parameters
        update_interval: int = DEFAULT_UPDATE_INTERVAL,
        no_delay: bool = DEFAULT_NO_DELAY,
        resend_count: int = DEFAULT_RESEND_COUNT,
        no_congestion_control: bool = DEFAULT_NO_CONGESTION_CONTROL,
        send_window_size: int = DEFAULT_SEND_WINDOW_SIZE,
        receive_window_size: int = DEFAULT_RECEIVE_WINDOW_SIZE,
    ):
        """
        Initialize KCP client
        
        Args:
            server_address: Server IP address
            server_port: Server port
            conv_id: KCP conversation ID (must match server)
            local_port: Local port to bind (0 for auto-assign)
            update_interval: KCP update interval in ms
            no_delay: Enable no-delay mode for low latency
            resend_count: Fast retransmission trigger count
            no_congestion_control: Disable congestion control
            send_window_size: Send window size
            receive_window_size: Receive window size
        """
        self.server_address = server_address
        self.server_port = server_port
        self.conv_id = conv_id
        
        # Create UDP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self._sock.settimeout(0.5)
        self._sock.bind(('0.0.0.0', local_port))
        
        # Get actual bound port
        self.local_port = self._sock.getsockname()[1]
        
        # Create KCP instance
        self._kcp = KCP(
            conv_id=conv_id,
            no_delay=no_delay,
            update_interval=update_interval,
            resend_count=resend_count,
            no_congestion_control=no_congestion_control,
            send_window_size=send_window_size,
            receive_window_size=receive_window_size,
        )
        
        # Set KCP outbound handler
        self._kcp.include_outbound_handler(self._on_kcp_output)
        
        # Message callback
        self._message_callback: Optional[Callable[[bytes], None]] = None
        
        # Control flags
        self._running = False
        self._update_thread: Optional[threading.Thread] = None
        self._receive_thread: Optional[threading.Thread] = None
        
        print(f"[KCP Client] Initialized")
        print(f"  Server: {server_address}:{server_port}")
        print(f"  Local port: {self.local_port}")
        print(f"  Conv ID: {conv_id}")
        print(f"  Update interval: {update_interval}ms")
    
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
            self._sock.sendto(data, (self.server_address, self.server_port))
        except Exception as e:
            print(f"[KCP Client] Socket send error: {e}")
    
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
            data, addr = self._sock.recvfrom(2048)
            return data
        except socket.timeout:
            return None
        except Exception as e:
            if self._running:  # Only log if we're still running
                print(f"[KCP Client] Socket receive error: {e}")
            return None
    
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
                    print(f"[KCP Client] Message callback error: {e}")
    
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
            print(f"[KCP Client] Send error: {e}")
            return False
    
    def _update_loop(self) -> None:
        """
        KCP update loop - runs in separate thread
        Handles KCP protocol state updates and retransmissions
        """
        print("[KCP Client] Update loop started")
        
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
                    print(f"[KCP Client] Update loop error: {e}")
        
        print("[KCP Client] Update loop stopped")
    
    def _receive_loop(self) -> None:
        """
        Socket receive loop - runs in separate thread
        Continuously receives data from socket and processes through KCP
        """
        print("[KCP Client] Receive loop started")
        
        while self._running:
            try:
                # Receive raw data from socket, timeout after 0.5 seconds
                raw_data = self._receive_from_socket()
                
                if raw_data:
                    # Process through KCP
                    self._process_received_data(raw_data)
                    
            except Exception as e:
                if self._running:
                    print(f"[KCP Client] Receive loop error: {e}")
        
        print("[KCP Client] Receive loop stopped")
    
    def start(self) -> None:
        """
        Start KCP client
        Launches update and receive threads
        """
        if self._running:
            print("[KCP Client] Already running")
            return
        
        self._running = True
        
        # Start update thread
        self._update_thread = threading.Thread(
            target=self._update_loop,
            daemon=True,
            name="KCP-Update"
        )
        self._update_thread.start()
        
        # Start receive thread
        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True,
            name="KCP-Receive"
        )
        self._receive_thread.start()
        
        print("[KCP Client] Started")
    
    def stop(self) -> None:
        """
        Stop KCP client
        Stops all threads and closes socket
        """
        if not self._running:
            return
        
        print("[KCP Client] Stopping...")
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
        
        print("[KCP Client] Stopped")
    
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
    
    # Create and start client
    print("=" * 60)
    print("KCP Client Core - Example Usage")
    print("=" * 60)
    
    client = KCPClient(
        server_address=SERVER_HOST,
        server_port=SERVER_PORT,
        conv_id=CONV_ID,
        local_port=52323,  # Specific port or 0 for auto
    )
    
    # Set message callback
    client.set_message_callback(on_message)
    
    # Start client
    client.start()
    
    try:
        # Send some test messages
        print("\n[Main] Sending test messages...")
        for i in range(5):
            message = f"Hello {i}".encode()
            if client.send(message):
                print(f"[Main] Sent: {message}")
            time.sleep(1)
        
        # Keep running
        print("\n[Main] Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user")
    finally:
        client.stop()
        print("[Main] Done")

