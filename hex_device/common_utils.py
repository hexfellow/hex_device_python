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

from .error_type import InvalidWSURLException


def is_valid_ws_url(url: str) -> str:
    ws_url_pattern = re.compile(r'^(ws|wss)://([a-zA-Z0-9.-]+)(?::(\d+))?$')

    match = ws_url_pattern.match(url)
    if not match:
        raise InvalidWSURLException(f"Invalid WebSocket URL: {url}")

    protocol, host, port_str = match.groups()

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

    return f"{protocol}://{host}:{port_str}"


async def delay(start_time, ms):
    end_time = start_time + ms / 1000
    now = time.perf_counter()
    await asyncio.sleep(end_time - now)


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
