#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

# A Simple Test for HexDeviceApi
# Quick Start: python3 arm_release.py --url ws://<Your controller ip>:8439 or ws://[::1%eth0]:8439

import argparse
import numpy as np
import time

import hex_device
from hex_device import HexDeviceApi, public_api_types_pb2
from hex_device import Arm, Hands
from hex_device.motor_base import CommandType, MitMotorCommand

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Hexapod robotic arm trajectory planning and execution test',
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        '--url', 
        metavar='URL',
        default="ws://0.0.0.0:8439",
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
                                # 6-axis config
                                config_dict = {
                                    'name':'Archer_d6y',
                                    'dof_num': 'six_axis',
                                    'motor_model': [0x80] * 6,
                                    'joints': [{
                                        'joint_name': 'joint_1',
                                        'joint_limit': [-2.7, 3.1, -0.3, 0.3, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_2',
                                        'joint_limit': [-1.57, 2.094, -0.3, 0.3, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_3',
                                        'joint_limit': [0.0, 3.14159265359, -0.3, 0.3, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_4',
                                        'joint_limit': [-1.56, 1.56, -0.3, 0.3, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_5',
                                        'joint_limit': [-1.56, 1.56, -0.3, 0.3, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_6',
                                        'joint_limit': [-1.57, 1.57, -0.3, 0.3, -0.0, 0.0]
                                    }]
                                }
                                if not device.reload_arm_config_from_dict(config_dict):
                                    exit(1)

                            print(f"arm position: {device.get_motor_positions(False).tolist()}")
                            # print(f"arm simple motor status: {device.get_simple_motor_status(False)}")
                            
                            device.motor_command(
                                CommandType.TORQUE,
                                [0.0] * device.motor_count)

                optional_devices = api.find_optional_device_by_robot_type(public_api_types_pb2.SecondaryDeviceType.SdtHandGp100)
                if optional_devices is not None:
                    device: Hands = optional_devices[0]
                    if device.has_new_data():
                        print(f"hands position: {device.get_motor_positions()}")
                        device.motor_command(
                            CommandType.TORQUE,
                            [0.0] * device.motor_count
                        )
                
                optional_devices = api.find_optional_device_by_robot_type(public_api_types_pb2.SecondaryDeviceType.SdtHandGp80G1)
                if optional_devices is not None:
                    device: Hands = optional_devices[0]
                    if device.has_new_data():
                        print(f"hands position: {device.get_motor_positions()}")

                        device.motor_command(
                            CommandType.TORQUE,
                            [0.0] * device.motor_count
                        )

                # optional_devices = api.find_optional_device_by_robot_type(public_api_types_pb2.SecondaryDeviceType.SdtHandGr100)
                # if optional_devices is not None:
                #     device: Hands = optional_devices[0]
                #     if device.has_new_data():
                #         print(f"hands position: {device.get_motor_positions()}")

                #         device.motor_command(
                #             CommandType.TORQUE,
                #             [0.0] * device.motor_count
                #         )

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
