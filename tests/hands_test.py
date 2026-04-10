#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Hexfellow. All rights reserved.
# Author: Hexfellow
# Date  : 2026-4-9
################################################################
# A Simple Test for HexDeviceApi
# Quick Start: python3 tests/hands_test.py --url ws://<Your controller ip>:8439 [--move]
import argparse
import time
import math

from hex_device import HexDeviceApi
from hex_device import Hands,Arm
from hex_device.motor_base import CommandType

def main():
    
    parser = argparse.ArgumentParser(
        description='Hexapod robotic arm trajectory planning and execution test',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        usage="python hello_test.py --url ws://<device_url>:8439 [options]"
    )
    parser.add_argument(
        '--url', 
        metavar='URL',
        required=True,
        help='WebSocket URL for HEX device connection'
    )
    parser.add_argument(
        "--move",
        action="store_true",
        help="Enable visualization of the arm trajectory."
    )
    
    args = parser.parse_args()
    check_move = args.move
    
    api = HexDeviceApi(ws_url=args.url, control_hz=500, enable_kcp=True, local_port=0)
    first_time = True
    
    try:
        while True:
            if api.is_api_exit():
                break
            else:
                
                for device in api.device_list:
                    if isinstance(device,Arm) and first_time:
                        first_time = False
                        device.start()
                
                for device in api.optional_device_list:
                    if isinstance(device, Hands):
                        if device.has_new_data():
                            if check_move:
                                pos_range = device.get_joint_limits()
                                min_pos = pos_range[0]
                                max_pos = pos_range[1]
                                
                                # Create sinusoidal motion for smooth interpolation
                                t = time.time()
                                interpolation_factor = (math.sin(t * 0.5) + 1.0) / 2.0  # 0 to 1
                                target_position = min_pos + interpolation_factor * (max_pos - min_pos)
                                # Apply to first motor (or all motors if desired)
                                target_positions = [target_position] + [0.0] * (device.motor_count - 1)

                                device.motor_command(
                                        CommandType.POSITION,
                                        target_positions
                                    )
                            print(f"Position: {device.get_motor_positions(pop=False)[0]:.2f} velocities: {device.get_motor_velocities(pop=False)[0]:.2f} torques: {device.get_motor_torques(pop=False)[0]:.2f}")
                            
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

