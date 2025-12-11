#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
import re
import time
import asyncio
import logging
import ipaddress
from typing import Tuple

from .error_type import InvalidWSURLException


def is_valid_ws_url(url: str) -> Tuple[str, bool]:
    # Pattern to match:
    # - IPv4/domain: ws://host:port or ws://host
    # - IPv6: ws://[2001:db8::1]:port or ws://[2001:db8::1]
    # - IPv6 with zone identifier: ws://[fe80::1%3]:port or ws://[fe80::1%eth0]:port
    # - IPv4-mapped IPv6: ws://[::ffff:192.0.2.1%eth0]:port (contains dots)
    ws_url_pattern = re.compile(
        r'^(ws|wss)://'
        r'('
        r'\[([0-9a-fA-F:.]+)(%([0-9a-zA-Z_]+))?\]'  # IPv6 in brackets with optional zone identifier (allows dots for IPv4-mapped format)
        r'|'
        r'([a-zA-Z0-9.-]+)'     # IPv4 or domain
        r')'
        r'(?::(\d+))?$'
    )

    match = ws_url_pattern.match(url)
    if not match:
        raise InvalidWSURLException(f"Invalid WebSocket URL: {url}")

    protocol, host_part, ipv6_addr, zone_part, zone_id, ipv4_or_domain, port_str = match.groups()
    
    # Determine if this is an IPv6 address
    is_ipv6 = ipv6_addr is not None
    
    # Determine the actual host (either IPv6 or IPv4/domain)
    if ipv6_addr:
        # Validate IPv6 address using ipaddress module (without zone identifier)
        try:
            ipaddress.IPv6Address(ipv6_addr)
        except ValueError:
            raise InvalidWSURLException(f"Invalid IPv6 address format in URL: {url}")
        
        # Validate zone identifier - must be present for IPv6 addresses
        if not zone_id:
            # IPv6 address must have zone identifier (e.g., %3 or %eth0)
            raise InvalidWSURLException(f"IPv6 address must include zone identifier (e.g., %3 or %eth0) in URL: {url}")
        
        # Zone identifier must be numeric (like %3) or alphanumeric with underscores (like %eth0)
        if not re.match(r'^[0-9a-zA-Z_]+$', zone_id):
            raise InvalidWSURLException(f"Invalid zone identifier format in URL: {url}. Zone identifier must be like %3 or %eth0")
        
        # Keep brackets and zone identifier for IPv6 in the returned URL
        host = f"[{ipv6_addr}%{zone_id}]"
    else:
        host = ipv4_or_domain

    # Set default port to 8439
    if not port_str:
        port_str = '8439'

    try:
        port = int(port_str)
        # port must be 0 ~ 65535
        if not (0 <= port <= 65535):
            raise InvalidWSURLException(f"Invalid port number in URL: {url}")
    except ValueError:
        raise InvalidWSURLException(f"Invalid port number in URL: {url}")

    validated_url = f"{protocol}://{host}:{port_str}"
    return validated_url, is_ipv6


async def delay(start_time, ms):
    end_time = start_time + ms / 1000
    now = time.perf_counter()
    sleep_time = end_time - now
    # Handle negative delay (when we're already past the target time)
    if sleep_time <= 0:
        # Log warning if delay is significantly negative (> 1ms)
        if sleep_time < -0.001:
            log_debug(f"HexDevice: Negative delay detected: {sleep_time*1000:.2f}ms - cycle overrun")
        return  # Don't sleep if we're already late
    
    await asyncio.sleep(sleep_time)


# Create a logger for the hex_device package
_logger = logging.getLogger(__name__.split('.')[0])  # Use 'hex_device' as logger name

def log_warn(message):
    """Log warning message"""
    _logger.warning(message)

def log_err(message):
    """Log error message"""
    _logger.error(message)

def log_info(message):
    """Log info message"""
    _logger.info(message)

def log_common(message):
    """Log common message (info level)"""
    _logger.info(message)

def log_debug(message):
    """Log debug message"""
    _logger.debug(message)
