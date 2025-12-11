#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################

# A Simple Test for HexDeviceApi
# Quick Start: python3 tests/main.py --url ws://<Your controller ip>:8439 or ws://[::1%eth0]:8439

import sys
import argparse
import numpy as np
import time

## If you want to use the local version of the library, you can uncomment the following lines.
# PROJECT_PATH = '<your project path>'
# sys.path.insert(1, f'{PROJECT_PATH}')
# sys.path.insert(
#     1,
#     f'{PROJECT_PATH}/hex_device/generated')

import hex_device
from hex_device import HexDeviceApi, public_api_types_pb2
from hex_device import Chassis, LinearLift, Arm, Hands
from hex_device.motor_base import CommandType, MitMotorCommand

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
        help='WebSocket URL for HEX device connection, example: ws://0.0.0.0:8439 or ws://[::1%eth0]:8439'
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
    
    # Enable/Disable loop test variable
    enable_disable_start_time = time.time()
    is_enabled = True

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                # Enable/Disable loop test logic
                elapsed_time = time.time() - enable_disable_start_time
                if elapsed_time >= 5.0:
                    is_enabled = not is_enabled
                    enable_disable_start_time = time.time()
                
                for device in api.device_list:
                    # for ChassisMaver
                    if isinstance(device, Chassis):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                # Must start device before using it.
                                device.start()
                                device.clear_odom_bias()

                            # device.get_device_summary()
                            print(
                                f"vehicle position: {device.get_vehicle_position()}"
                            )
                            
                            # # Enable/Disable loop test
                            # if is_enabled:
                            #     device.enable()
                            #     print(f"Chassis enabled")
                            # else:
                            #     device.disable()
                            #     print(f"Chassis disabled")
                            
                            ## command, Please select one of the following commands.
                            device.set_vehicle_speed(0.0, 0.0, 0.1)
                            # device.motor_command(CommandType.SPEED, [0.5] * len(device))

                    # for Arm
                    elif isinstance(device, Arm):
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
                                # # 7-axis config
                                # config_dict = {
                                #     'name':'saber_d7x',
                                #     'dof_num': 'seven_axis',
                                #     'motor_model': [0x80] * 7,
                                #     'joints': [{
                                #         'joint_name': 'joint_1',
                                #         'joint_limit': [-2.967, 2.967, -0.3, 0.3, -0.0, 0.0]
                                #     }, {
                                #         'joint_name': 'joint_2',
                                #         'joint_limit': [-1.57, 1.57, -0.3, 0.3, -0.0, 0.0]
                                #     }, {
                                #         'joint_name': 'joint_3',
                                #         'joint_limit': [-2.967, 2.967, -0.3, 0.3, -0.0, 0.0]
                                #     }, {
                                #         'joint_name': 'joint_4',
                                #         'joint_limit': [-0.393, 3.14159265359, -0.3, 0.3, -0.0, 0.0]
                                #     }, {
                                #         'joint_name': 'joint_5',
                                #         'joint_limit': [-1.6, 1.6, -0.3, 0.3, -0.0, 0.0]
                                #     }, {
                                #         'joint_name': 'joint_6',
                                #         'joint_limit': [-1.57, 1.57, -0.3, 0.3, -0.0, 0.0]
                                #     }, {
                                #         'joint_name': 'joint_7',
                                #         'joint_limit': [-1.57, 1.57, -0.3, 0.3, -0.0, 0.0]
                                #     }]
                                # }
                                if not device.reload_arm_config_from_dict(config_dict):
                                    exit(1)

                                device.command_timeout_check(False)
                                # device.motor_command(
                                #     CommandType.SPEED,
                                #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                                
                            # print(f"arm device summary: {device.get_device_summary()}")
                            # print(f"arm motor summary: {device.get_motor_summary()}")
                            # print(f"arm pulse per rotation: {device.get_motor_pulse_per_rotations()}")

                            print(f"arm position: {device.get_motor_positions(False)}")
                            # print(f"arm simple motor status: {device.get_simple_motor_status(False)}")
                            # # print the encoders to zero, you can use this to set the encoders to zero.
                            # print(f"arm encoders from zero: {device.get_encoders_to_zero(False)}")

                            # # command, Please select one of the following commands.
                            # # warning!!!! Only position command is limit position & speed.Only speed command is limit speed & acc.
                            # device.motor_command(
                            #     CommandType.POSITION,
                            #     [-0.3, -1.48, 2.86, 0.0, 0.0, 0.0])

                            # device.motor_command(
                            #     CommandType.TORQUE,
                            #     [0.0] * device.motor_count)

                            # device.motor_command(
                            #     CommandType.SPEED,
                            #     [0.0] * device.motor_count)

                            # device.motor_command(
                            #     CommandType.BRAKE,
                            #     [True] * device.motor_count)

                            ## If you need to use torque control or mit control, be sure to start with small parameters
                            # device.motor_command(
                            #     CommandType.TORQUE,
                            #     [0.0] * device.motor_count)

                            # device.motor_command(
                            #     CommandType.MIT,[
                            #     MitMotorCommand(position=-0.3, speed=0.0, torque=0.0, kp=150.0, kd=12.0),
                            #     MitMotorCommand(position=-1.48, speed=0.0, torque=0.0, kp=150.0, kd=12.0),
                            #     MitMotorCommand(position=2.86, speed=0.0, torque=0.0, kp=150.0, kd=12.0),
                            #     MitMotorCommand(position=0.0, speed=0.0, torque=0.0, kp=150.0, kd=12.0),
                            #     MitMotorCommand(position=0.0, speed=0.0, torque=0.0, kp=39.0, kd=0.8),
                            #     MitMotorCommand(position=0.0, speed=0.0, torque=0.0, kp=39.0, kd=0.8),
                            #     ])

                            # mit_commands = device.construct_mit_command(
                            #     np.array([-0.3, -1.48, 2.86, 0.0, 0.0, 0.0]), 
                            #     np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
                            #     np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
                            #     np.array([150.0, 150.0, 150.0, 150.0, 39.0, 39.0]), 
                            #     np.array([12.0, 12.0, 12.0, 12.0, 0.8, 0.8])
                            # )
                            # mit_commands = device.construct_mit_command(
                            #     [0.3, -1.48, 2.86, 0.0, 0.0, 0.0], 
                            #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            #     [150.0, 150.0, 150.0, 150.0, 39.0, 39.0], 
                            #     [12.0, 12.0, 12.0, 12.0, 0.8, 0.8]
                            # )
                            # device.motor_command(
                            #     CommandType.MIT,
                            #     mit_commands)

                    elif isinstance(device, LinearLift):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                ## You can change speed if you want
                                # device.set_move_speed(75000)

                                # The lift must be calibrated before moving when powered on. 
                                # It is strictly forbidden to send the calibrate command continuously!!!
                                # device.calibrate()

                            print(f"linear lift position: {device.get_motor_positions()}")
                            # print(f"linear lift pulse_per_meter: {device.get_pulse_per_meter()}")
                            # print(f"linear lift speed: {device.get_move_speed()}")
                            # print(f"linear lift range: {device.get_pos_range()}")
                            # print(f"linear lift max motor speed: {device.get_max_move_speed()}")
                            # print(f"linear lift state: {device.get_state()}")
                            # print(f"linear lift parking stop detail: {device.get_parking_stop_detail()}")

                            ## command, support POSITION and BRAKE command type.
                            # device.motor_command(
                            #     CommandType.POSITION,
                            #     0.0)

                optional_devices = api.find_optional_device_by_robot_type(public_api_types_pb2.SecondaryDeviceType.SdtHandGp100)
                if optional_devices is not None:
                    device:Hands = optional_devices[0]
                    if device.has_new_data():
                        # print(f"hands position: {device.get_motor_positions()}")
                        device.motor_command(
                            CommandType.TORQUE,
                            [0.0] * device.motor_count
                        )

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
