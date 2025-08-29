#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
from enum import Enum


class WsError(Exception):
    pass


class ProtocolError(WsError):
    pass


class ConnectionClosedError(WsError):
    pass


class InvalidWSURLException(Exception):
    """Custom exception, used to indicate invalid WebSocket URL"""
    pass
