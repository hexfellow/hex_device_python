#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

# A Simple Test for HexDeviceApi

import sys
import argparse

sys.path.insert(1, '<your project path>/hex_device_python')
sys.path.insert(
    1,
    '<your project path>/hex_device_python/hex_device/generated')

from hex_device import HexDeviceApi
import time
from hex_device.chassis_maver import ChassisMaver
from hex_device.motor_base import CommandType
from hex_device.arm_archer import ArmArcher
from hex_device.motor_base import MitMotorCommand
from hex_device.chassis_mark2 import ChassisMark2

from hex_device.motor_base import public_api_types_pb2

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Hexapod robotic arm trajectory planning and execution test',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--url', 
        metavar='URL',
        default="ws://0.0.0.0:8439",
        help='WebSocket URL for HEX device connection'
    )
    args = parser.parse_args()
    
    # Init HexDeviceApi
    api = HexDeviceApi(ws_url=args.url, control_hz=250)
    first_time = True

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                for device in api.device_list:
                    # for ChassisMaver
                    if isinstance(device, ChassisMaver):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                device.clear_odom_bias()

                            # device.get_device_summary()
                            print(
                                f"vehicle position: {device.get_vehicle_position()}"
                            )
                            device.enable()
                            
                            ## command, Please select one of the following commands.
                            # device.set_vehicle_speed(0.0, 0.0, 0.0)
                            # device.motor_command(CommandType.SPEED, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

                    # for ChassisMark2
                    if isinstance(device, ChassisMark2):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                device.clear_odom_bias()

                            # device.get_device_summary()
                            print(
                                f"vehicle position: {device.get_vehicle_position()}"
                            )
                            device.enable()
                            # device.disable()

                            ## command, Please select one of the following commands.
                            # device.set_vehicle_speed(0.1, 0.0, 0.0)
                            # device.motor_command(CommandType.SPEED, [0.4, 0.4])

                    # for ArmArcher
                    elif isinstance(device, ArmArcher):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                config_dict = {
                                    'name':'Archer_d6y',
                                    'dof_num': 'six_axis',
                                    'motor_model': [0x80] * 6,
                                    'joints': [{
                                        'joint_name': 'joint_1',
                                        'joint_limit': [-2.7, 3.1, -0.03, 0.03, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_2',
                                        'joint_limit': [-1.57, 2.094, -0.03, 0.03, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_3',
                                        'joint_limit': [0.0, 3.14159265359, -0.03, 0.03, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_4',
                                        'joint_limit': [-1.56, 1.56, -0.03, 0.03, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_5',
                                        'joint_limit': [-1.56, 1.56, -0.03, 0.03, -0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_6',
                                        'joint_limit': [-1.57, 1.57, -0.03, 0.03, -0.0, 0.0]
                                    }]
                                }
                                if not device.reload_arm_config_from_dict(config_dict):
                                    exit(1)

                                # device.command_timeout_check(False)
                                # device.motor_command(
                                #     CommandType.SPEED,
                                #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                                

                            # print(device.get_device_summary())
                            # print(device.get_motor_summary())

                            print(f"arm position: {device.get_motor_positions()}")

                            ## command, Please select one of the following commands.
                            ## warning!!!! Only position command is limit position & speed.Only speed command is limit speed & acc.
                            # device.motor_command(
                            #     CommandType.POSITION,
                            #     [-0.3, -1.48, 2.86, 0.0, 0.0, 0.0])

                            # device.motor_command(
                            #     CommandType.TORQUE,
                            #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                            # device.motor_command(
                            #     CommandType.SPEED,
                            #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                            # device.motor_command(
                            #     CommandType.BRAKE,
                            #     [True, True, True, True, True, True])

                            # device.motor_command(
                            #     CommandType.MIT,[
                            #     MitMotorCommand(torque=0.0, speed=0.0, position=-0.3, kp=150.0, kd=12.0),
                            #     MitMotorCommand(torque=0.0, speed=0.0, position=-1.48, kp=150.0, kd=12.0),
                            #     MitMotorCommand(torque=0.0, speed=0.0, position=2.86, kp=150.0, kd=12.0),
                            #     MitMotorCommand(torque=0.0, speed=0.0, position=0.0, kp=150.0, kd=12.0),
                            #     MitMotorCommand(torque=0.0, speed=0.0, position=0.0, kp=39.0, kd=0.8),
                            #     MitMotorCommand(torque=0.0, speed=0.0, position=0.0, kp=39.0, kd=0.8)])

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == "__main__":
    main()
