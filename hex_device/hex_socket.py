#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

import enum
from typing import Optional, List, Tuple
import struct

U16_MAX = 65535

class HexSocketOpcode(enum.Enum):
    """
    HexSocket Opcode Definition
    Lower 4 bits of Byte[0]
    """
    Continuation = 0x0  # Not used for now
    Text = 0x1
    Binary = 0x2
    ConnectionClose = 0x8  # Not used for now
    Ping = 0x9
    Pong = 0xA


class HexSocketParser:
    """
    HexSocket Protocol Parser
    
    Header Format (Little Endian, 4 Bytes):
    - Byte[0]: 
      - Higher 4 bits: 0b1000 (reserved, always 0x8)
      - Lower 4 bits: opcode (0x0-0xF)
    - Byte[1]: Reserved (always 0x00)
    - Byte[2-3]: Data length in u16 (little endian)
    
    Usage:
        # Stateful parser that accumulates data
        parser = HexSocketParser()
        result = parser.parse(incoming_data)
        if result is not None:
            for opcode, payload in result:
                print(f"Received {opcode.name}: {payload}")
                
        # Static methods for single frame operations
        frame = HexSocketParser.create_header(b"Hello", HexSocketOpcode.Binary)
    """
    
    RESERVED_HIGH_BITS = 0x80  # 0b1000_0000
    HEADER_SIZE = 4
    
    def __init__(self):
        """Initialize parser with empty buffer"""
        self.data = bytearray()

    @staticmethod
    def create_header(data: bytes, opcode: HexSocketOpcode) -> bytes:
        """
        Create HexSocket header and pack with data
        
        Args:
            data: Payload data
            opcode: HexSocket opcode
            
        Returns:
            Header + data
            
        Raises:
            ValueError: If data length exceeds U16_MAX
            
        Example:
            >>> frame = HexSocketParser.create_header(b"Hello", HexSocketOpcode.Binary)
            >>> frame[:4].hex()
            '82000500'
        """
        length = len(data)
        if length > U16_MAX:
            raise ValueError(f"Data length {length} exceeds U16_MAX ({U16_MAX}) bytes")
        
        # Byte[0]: Higher 4 bits = 0b1000, Lower 4 bits = opcode
        byte0 = HexSocketParser.RESERVED_HIGH_BITS | (opcode.value & 0x0F)
        
        # Byte[1]: Reserved, always 0
        byte1 = 0x00
        
        # Byte[2-3]: Length in u16 little endian
        # '<' means little endian, 'BB' for two bytes, 'H' for u16
        header = struct.pack('<BBH', byte0, byte1, length)
        
        return header + data
    
    @staticmethod
    def parse_header(header: bytes) -> Tuple[HexSocketOpcode, int]:
        """
        Parse HexSocket header
        
        Args:
            header: 4-byte header
            
        Returns:
            (opcode, data_length)
            
        Raises:
            ValueError: If header format is invalid
            
        Example:
            >>> header = b'\\x82\\x00\\x05\\x00'
            >>> opcode, length = HexSocketParser.parse_header(header)
            >>> opcode.name, length
            ('Binary', 5)
        """
        if len(header) < HexSocketParser.HEADER_SIZE:
            raise ValueError(f"Header too short: {len(header)} < {HexSocketParser.HEADER_SIZE}")
        
        # Unpack little endian: byte0, byte1, length
        byte0, byte1, length = struct.unpack('<BBH', header[:HexSocketParser.HEADER_SIZE])
        
        # Check reserved high bits
        high_bits = (byte0 & 0xF0)
        if high_bits != HexSocketParser.RESERVED_HIGH_BITS:
            raise ValueError(f"Invalid reserved bits: 0x{high_bits:02X}, expected 0x{HexSocketParser.RESERVED_HIGH_BITS:02X}")
        
        # Extract opcode (lower 4 bits)
        opcode_value = byte0 & 0x0F
        
        # Convert to enum
        try:
            opcode = HexSocketOpcode(opcode_value)
        except ValueError:
            raise ValueError(f"Unknown opcode: 0x{opcode_value:X}")
        
        # Check byte1 is reserved (should be 0)
        if byte1 != 0x00:
            raise ValueError(f"Byte[1] should be 0x00, got 0x{byte1:02X}")
        
        return opcode, length
    
    def parse(self, incoming: bytes) -> Optional[List[Tuple[HexSocketOpcode, bytes]]]:
        """
        Parse incoming data and return all complete frames
        
        This method accumulates incoming data in an internal buffer and parses
        all complete frames. Partial data is kept for the next call.
        
        Args:
            incoming: New incoming data to parse
            
        Returns:
            None if no complete frames are available yet
            List of (opcode, payload) tuples if one or more frames were parsed
            
        Raises:
            ValueError: If frame format is invalid
            
        Example:
            >>> parser = HexSocketParser()
            >>> # Feed partial data
            >>> result = parser.parse(b'\\x82\\x00')
            >>> result is None
            True
            >>> # Feed rest of frame
            >>> result = parser.parse(b'\\x05\\x00Hello')
            >>> result
            [(HexSocketOpcode.Binary, b'Hello')]
            >>> # Feed multiple frames at once
            >>> result = parser.parse(b'\\x82\\x00\\x02\\x00Hi\\x81\\x00\\x03\\x00Bye')
            >>> len(result)
            2
        """
        ret = []
        self.data.extend(incoming)
        
        while True:
            result = self._inner_parse()
            if result is not None:
                opcode, payload = result
                ret.append((opcode, payload))
            else:
                # No more complete frames
                if len(ret) > 0:
                    return ret
                else:
                    return None
    
    def _inner_parse(self) -> Optional[Tuple[HexSocketOpcode, bytes]]:
        """
        Internal method to parse a single frame from the buffer
        
        Returns:
            (opcode, payload) if a complete frame was parsed, None otherwise
            
        Raises:
            ValueError: If frame format is invalid
        """
        # Check if header is complete
        if len(self.data) < HexSocketParser.HEADER_SIZE:
            return None
        
        # Validate header and get length
        if len(self.data) >= HexSocketParser.HEADER_SIZE:
            # Check reserved high bits
            if self.data[0] & 0xF0 != HexSocketParser.RESERVED_HIGH_BITS:
                raise ValueError(f"Invalid header: {bytes(self.data[:HexSocketParser.HEADER_SIZE]).hex()}")
            
            # Extract opcode
            opcode_value = self.data[0] & 0x0F
            try:
                opcode = HexSocketOpcode(opcode_value)
            except ValueError:
                raise ValueError(f"Unknown opcode: 0x{opcode_value:X}")
            
            # Extract length (little endian u16 at bytes 2-3)
            length = struct.unpack('<H', self.data[2:4])[0]
        else:
            return None
        
        # Check if we have complete frame data
        if len(self.data) >= HexSocketParser.HEADER_SIZE + length:
            # Extract the complete frame
            frame_end = HexSocketParser.HEADER_SIZE + length
            payload = bytes(self.data[HexSocketParser.HEADER_SIZE:frame_end])
            
            # Remove processed frame from buffer
            self.data = self.data[frame_end:]
            
            return (opcode, payload)
        else:
            return None