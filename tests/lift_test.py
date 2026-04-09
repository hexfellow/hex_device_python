#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

# A Simple Test for HexDeviceApi
# Quick Start: python3 tests/lift_test.py --url ws://<Your controller ip>:8439 or ws://[::1%eth0]:8439

import sys
import argparse
import numpy as np
import time

import hex_device
from hex_device import HexDeviceApi, public_api_types_pb2
from hex_device import LinearLift, ZetaLift
from hex_device.motor_base import CommandType, MitMotorCommand

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
    args = parser.parse_args()
    
    # Set log level
    hex_device.set_log_level(args.log_level)
    print(f"Log level set to: {args.log_level}")
    
    # Init HexDeviceApi
    api = HexDeviceApi(ws_url=args.url, control_hz=500, enable_kcp=True, local_port=0)
    first_time = True

    # Position list for linear lift, send one every 5 seconds in cycle
    lift_position_list = [0.1, 0.4]
    lift_last_command_time = 0.0
    lift_position_index = 0

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                for device in api.device_list:
                    if isinstance(device, LinearLift):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                ## You can change speed if you want
                                # device.set_move_speed(75000)

                                # The lift must be calibrated before moving when powered on. 
                                # It is strictly forbidden to send the calibrate command continuously!!!
                                device.calibrate()

                            print(f"linear lift position: {device.get_motor_positions()}, range: {device.get_pos_range()}")
                            # print(f"linear lift pulse_per_meter: {device.get_pulse_per_meter()}")
                            # print(f"linear lift speed: {device.get_move_speed()}")
                            # print(f"linear lift max motor speed: {device.get_max_move_speed()}")
                            # print(f"linear lift state: {device.get_state()}")
                            # print(f"linear lift parking stop detail: {device.get_parking_stop_detail()}")

                            # command: send next position from list every 5 seconds, cycle through list
                            if time.time() - lift_last_command_time >= 5.0:
                                target_pos = lift_position_list[lift_position_index]
                                device.motor_command(CommandType.POSITION, target_pos)
                                print(f"linear lift command sent: position={target_pos} (index {lift_position_index})")
                                lift_position_index = (lift_position_index + 1) % len(lift_position_list)
                                lift_last_command_time = time.time()
                    
                    elif isinstance(device, ZetaLift):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                ## setting the max move speed of pos mode
                                # device.set_move_speed([0.1, 0.1, 0.1])

                            ## please select one of the following commands.
                            # device.motor_command(
                            #     CommandType.SPEED,
                            #     [0.0, -0.72, 0.0])

                            # device.motor_command(
                            #     CommandType.POSITION,
                            #     [0.0, 0.2, 0.0])

                            print(f"zeta lift position: {device.get_motor_positions()}")

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
