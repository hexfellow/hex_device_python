#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

# A Simple Test for HexDeviceApi
# Quick Start: python3 tests/hello_test.py --url ws://<Your controller ip>:8439 [--device arm|hello|both] [--visuable]

import argparse
import numpy as np
import time
import colorsys

import hex_device
from hex_device import HexDeviceApi, public_api_types_pb2
from hex_device import Arm, SdtHello

import socket
import json
from copy import deepcopy

def send_plotjuggle_data(Plotjuggle_socket: socket.socket, data, IP:str, port:int):
    Plotjuggle_socket.sendto(json.dumps(data).encode(), (IP, port))

def transform_plotjuggle_data(device: Arm) -> dict:
    dic = {}
    device_motor_status = device.get_simple_motor_status(False)
    dic["position"] = device_motor_status["pos"].tolist()
    dic["velocity"] = device_motor_status["vel"].tolist()
    return deepcopy(dic)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Hexapod robotic arm trajectory planning and execution test',
        formatter_class=argparse.RawTextHelpFormatter,
        usage="python hello_test.py --url ws://<device_url>:8439 [options]"
    )
    parser.add_argument(
        '--url', 
        metavar='URL',
        required=True,
        help='WebSocket URL for HEX device connection, example: ws://0.0.0.0:8439 or ws://[::1%%eth0]:8439'
    )
    parser.add_argument(
        '--log-level',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        default='INFO',
        help='Set logging level for hex_device package, example: DEBUG, INFO, WARNING, ERROR'
    )
    parser.add_argument(
        '--device',
        choices=['arm', 'sdt-hello', 'both', None],
        default=None,
        help='Which device to print status for: arm (motor positions), sdt-hello (simple motor pos), or both.'
    )
    
    parser.add_argument(
        "--visuable",
        action="store_true",
        help="Enable visualization of the arm trajectory."
    )
    
    args = parser.parse_args()
    
    check_arm = args.device in ('arm', 'both')
    check_sdt_hello = args.device in ('hello', 'both')
    
    # Set log level
    hex_device.set_log_level(args.log_level)
    print(f"Log level set to: {args.log_level}")
    
    # Check if visualization is enabled
    check_visuable = args.visuable
    visuableIp = "127.0.0.1"
    visuablePort = 9870
    
    # Init Plotjuggle
    if check_visuable:
        Plotjuggle_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        Plotjuggle_data = {}
        Plotjuggle_data["Hello"] = {}
        Plotjuggle_data["Arm"] = {}
        Plotjuggle_last_send_time = time.time()
        print(f"============= visuable Server url:{visuableIp} port:{visuablePort} =================")
        
    # Init HexDeviceApi
    api = HexDeviceApi(ws_url=args.url, control_hz=500, enable_kcp=True, local_port=0)
    first_time = True
    
    # RGB stripe color cycling variables
    rgb_hue = 0.0  # hue value 0.0 ~ 1.0
    rgb_last_update_time = time.time()

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                for device in api.device_list:
                    # for Arm
                    if isinstance(device, Arm):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                # Must start device before using it.
                                device.start()
                               
                            if check_arm:
                                print(f"arm position: {device.get_motor_positions(False).tolist()}")
                            # print(f"arm simple motor status: {device.get_simple_motor_status(False)}")
                            
                            if check_visuable:
                                Plotjuggle_data['Arm'] = transform_plotjuggle_data(device)

                optional_devices = api.find_optional_device_by_robot_type(public_api_types_pb2.SecondaryDeviceType.SdtHello1J1T4BV1)
                if optional_devices is not None:
                    device:SdtHello = optional_devices[0]
                    if device.has_new_data():
                        current_time = time.time()
                        if current_time - rgb_last_update_time >= 0.01:  # 10ms
                            rgb_last_update_time = current_time
                            rgb_hue = (rgb_hue + 0.005) % 1.0  # increase 0.5% hue value
                            r_list, g_list, b_list = [], [], []
                            for i in range(6):
                                hue = (rgb_hue + i / 6.0) % 1.0  # each light offset 60°
                                r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                                r_list.append(int(r * 255))
                                g_list.append(int(g * 255))
                                b_list.append(int(b * 255))
                            
                            # set rgb stripe command
                            device.set_rgb_stripe_command(r_list, g_list, b_list)
                            
                        if check_sdt_hello:
                            print(f"sdt hello position: {device.get_simple_motor_status()['pos']}")
                            
                        if check_visuable:
                            pos = device.get_simple_motor_status()
                            if pos:
                                Plotjuggle_data['Hello']["motor_position"] = pos["pos"]
            if check_visuable:
                Plotjuggle_current_send_time = time.time()
                if Plotjuggle_current_send_time - Plotjuggle_last_send_time >= 0.01:  # 10ms
                    send_plotjuggle_data(Plotjuggle_socket, Plotjuggle_data, visuableIp, visuablePort)
                    Plotjuggle_last_send_time = Plotjuggle_current_send_time
                        
            time.sleep(0.0001)

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == "__main__":
    main()
