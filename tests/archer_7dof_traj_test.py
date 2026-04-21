#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-11-1
################################################################

# Warning!!! The test will move your robotic arm. Please ensure that there is enough space in front of and around the robotic arm before performing this test ！！！
# You can use this test like this command:  python3 tests/saber7dof_traj_test.py --url ws://<Your controller ip>:8439

# For saber, when you disconnect from the robotic arm, the joints will automatically lock. 
# You can modify the example in main.py to send a torque 0 command to unlock the motors (please ensure proper safety measures to prevent the robotic arm from falling directly).

import sys
import math
import numpy as np
import argparse

PROJECT_PATH = '/Users/jecjune/Downloads/python/hex_device/hex_device_python'
sys.path.insert(1, f'{PROJECT_PATH}')
sys.path.insert(
    1,
    f'{PROJECT_PATH}/hex_device/generated')


from hex_device import HexDeviceApi
import time
from hex_device import Arm, Hands
from hex_device import CommandType, public_api_types_pb2

TRAJ_TIME = 7
HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RETURN_HOME_DURATION = 10.0  # Duration for returning to home position (seconds)

class TrajectoryPlanner:
    """Trajectory planner that supports smooth acceleration and deceleration planning"""
    
    def __init__(self, waypoints, segment_durations, dwell_time=1.0):
        """
        Initialize trajectory planner
        waypoints: List of N waypoints
        segment_durations: List of N durations (seconds), one per segment.
                           segment_durations[i] is the time to move from waypoint[i] to waypoint[(i+1) % N].
        dwell_time: Duration to hold at the end of each segment before moving to next (seconds)
        """
        if len(segment_durations) != len(waypoints):
            raise ValueError(
                f"segment_durations length ({len(segment_durations)}) must match "
                f"waypoints length ({len(waypoints)})"
            )
        self.waypoints = waypoints
        self.segment_durations = segment_durations
        self.dwell_time = dwell_time
        # Cumulative start time of each segment within one full loop
        self._cycle_offsets = []
        cumulative = 0.0
        for d in segment_durations:
            self._cycle_offsets.append(cumulative)
            cumulative += d + dwell_time
        self._total_cycle = cumulative  # Duration of one full loop
        
        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.start_time = None
        self.last_target_position = None  # Store last commanded position

    def _find_segment(self, loop_elapsed):
        """Given elapsed time within one loop, return (segment_index, cycle_elapsed)."""
        n = len(self.waypoints)
        for i in range(n - 1, -1, -1):
            if loop_elapsed >= self._cycle_offsets[i]:
                return i, loop_elapsed - self._cycle_offsets[i]
        return 0, loop_elapsed

    def start_trajectory(self):
        """Start trajectory execution"""
        if not self.waypoints:
            return False
            
        self.trajectory_started = True
        self.start_time = time.time()
        self.current_waypoint_index = 0
        return True
        
    def get_current_target(self):
        """Get the target position at the current moment"""
        if not self.trajectory_started or not self.waypoints:
            return None
            
        elapsed_time = time.time() - self.start_time
        loop_elapsed = elapsed_time % self._total_cycle
        segment_index, cycle_elapsed = self._find_segment(loop_elapsed)

        seg_duration = self.segment_durations[segment_index]
        start_waypoint = self.waypoints[segment_index]
        end_waypoint = self.waypoints[(segment_index + 1) % len(self.waypoints)]
        end_pos = np.array(end_waypoint)
        
        if cycle_elapsed >= seg_duration:
            # Dwell phase: hold at end waypoint
            target_position = end_pos
        else:
            # Motion phase: S-curve interpolation
            normalized_time = cycle_elapsed / seg_duration
            s = self._smooth_step(normalized_time)
            start_pos = np.array(start_waypoint)
            target_position = start_pos + s * (end_pos - start_pos)
        
        self.current_waypoint_index = segment_index
        self.last_target_position = target_position  # Store for potential return home
        
        return target_position
    
    def get_last_position(self):
        """Get the last commanded position"""
        return self.last_target_position
        
    def _smooth_step(self, t):
        """S-curve interpolation function that provides smooth acceleration and deceleration"""
        # Limit t to [0,1] range
        t = max(0.0, min(1.0, t))
        
        # Use 5th degree polynomial for smoother interpolation: 6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3
        
    def get_current_segment_info(self):
        """Get information about the current segment"""
        if not self.trajectory_started:
            return None
            
        elapsed_time = time.time() - self.start_time
        loop_elapsed = elapsed_time % self._total_cycle
        segment_index, cycle_elapsed = self._find_segment(loop_elapsed)

        seg_duration = self.segment_durations[segment_index]
        is_dwelling = cycle_elapsed >= seg_duration
        segment_progress = min(cycle_elapsed / seg_duration, 1.0)
        
        return {
            'segment_index': segment_index,
            'segment_progress': segment_progress,
            'is_dwelling': is_dwelling,
            'total_elapsed': elapsed_time
        }


class ReturnHomeController:
    """Controller for smooth return to home position"""
    
    def __init__(self, start_position, home_position, duration):
        """
        Initialize return home controller
        start_position: Starting position (current position when Ctrl+C is pressed)
        home_position: Target home position
        duration: Duration to reach home position (seconds)
        """
        self.start_position = np.array(start_position)
        self.home_position = np.array(home_position)
        self.duration = duration
        self.start_time = time.time()
        
    def get_target_position(self):
        """Get the current target position during return home"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time >= self.duration:
            return self.home_position, True  # Reached home
        
        # Calculate normalized time [0, 1]
        t = elapsed_time / self.duration
        
        # Use S-curve interpolation for smooth motion
        s = self._smooth_step(t)
        
        # Interpolate between start and home position
        target_position = self.start_position + s * (self.home_position - self.start_position)
        
        return target_position, False  # Not yet reached home
    
    def _smooth_step(self, t):
        """S-curve interpolation function"""
        t = max(0.0, min(1.0, t))
        return 6 * t**5 - 15 * t**4 + 10 * t**3


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
    api = HexDeviceApi(ws_url=args.url, control_hz=250, enable_kcp=True, local_port=0)
    first_time = True
    hands_first_time = True

    arm_position = [
        [0.68, 1.17, 1.25, -1.0, -0.94, 0.8, 0.7],
        [-0.68, -1.17, -1.25, 1.0, 1.0, -0.8, -0.7],
    ]
    # One duration per waypoint: segment_durations[i] is the time to move from
    # arm_position[i] to arm_position[(i+1) % N]. Length must equal len(arm_position).
    segment_durations = [
        TRAJ_TIME,  # arm_position[0] -> arm_position[1]
        TRAJ_TIME,  # arm_position[1] -> arm_position[0]
    ]

    trajectory_planner = TrajectoryPlanner(
        waypoints=arm_position,
        segment_durations=segment_durations,
    )
    
    trajectory_initialized = False
    loop_counter = 0
    arm_device = None  # Store reference to arm device
    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                arm_devices = api.find_device_by_robot_type(public_api_types_pb2.RobotType.RtArmArcherX7h1)
                if arm_devices is not None:
                    device = arm_devices  # Use the found device
                    arm_device = device  # Store reference for return home
                    if device.has_new_data():
                        if first_time:
                            first_time = False
                            # Must start device before using it.
                            device.start()
                            # 7-axis config
                            config_dict = {
                                'name':'archer_x7_h1',
                                'dof_num': 'seven_axis',
                                'motor_model': [0x87, 0x87, 0x85, 0x85, 0x84, 0x84, 0x84],
                                'joints': [{
                                    'joint_name': 'joint_1',
                                    'joint_limit': [-2.7, 2.8, -0.5, 0.5, -0.0, 0.0]
                                }, {
                                    'joint_name': 'joint_2',
                                    'joint_limit': [-2.0, 1.4, -0.4, 0.4, -0.0, 0.0]
                                }, {
                                    'joint_name': 'joint_3',
                                    'joint_limit': [-2.7, 2.8, -0.9, 0.9, -0.0, 0.0]
                                }, {
                                    'joint_name': 'joint_4',
                                    'joint_limit': [-2.1, 1.0, -0.4, 0.4, -0.0, 0.0]
                                }, {
                                    'joint_name': 'joint_5',
                                    'joint_limit': [-2.7, 2.7, -0.9, 0.9, -0.0, 0.0]
                                }, {
                                    'joint_name': 'joint_6',
                                    'joint_limit': [-0.85, 0.89, -0.4, 0.4, -0.0, 0.0]
                                }, {
                                    'joint_name': 'joint_7',
                                    'joint_limit': [-0.78, 0.78, -0.4, 0.4, -0.0, 0.0]
                                }]
                            }
                            if not device.reload_arm_config_from_dict(config_dict):
                                exit(1)

                        if not trajectory_initialized:
                            if trajectory_planner.start_trajectory():
                                trajectory_initialized = True
                                print("Trajectory planner initialized, starting waypoint planning")
                        
                        if trajectory_initialized:
                            target_positions = trajectory_planner.get_current_target()
                            
                            if target_positions is not None:
                                # Send position command
                                device.motor_command(CommandType.POSITION, target_positions.tolist())
                                
                                segment_info = trajectory_planner.get_current_segment_info()
                                if int(segment_info['total_elapsed'] * 10) % 5 == 0:  # Print every 0.5 seconds
                                    # print(f"Path segment: {segment_info['segment_index']} -> {(segment_info['segment_index'] + 1) % len(arm_position)}")
                                    # if segment_info['is_dwelling']:
                                    #     print(f"Status: dwelling (holding end position)")
                                    # else:
                                    #     print(f"Segment progress: {segment_info['segment_progress']:.2f}")
                                    # print(f"Target position: {[f'{x:.3f}' for x in target_positions]}")
                                    
                                    # Check if a complete loop has been completed
                                    if segment_info['segment_index'] == 0 and segment_info['segment_progress'] < 0.1:
                                        loop_counter += 1
                                        # print(f"--- Completed trajectory loop {loop_counter} ---")

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\nReceived Ctrl-C. Planning return to home position...")
        
        # Get the last commanded position
        last_position = trajectory_planner.get_last_position()
        if last_position is None:
            print("No trajectory data available, using default start position")
            last_position = arm_position[0]
        
        print(f"Current position: {[f'{x:.3f}' for x in last_position]}")
        print(f"Home position: {HOME_POSITION}")
        print(f"Return duration: {RETURN_HOME_DURATION} seconds")
        
        # Create return home controller
        return_home = ReturnHomeController(last_position, HOME_POSITION, RETURN_HOME_DURATION)
        
        # Execute return home trajectory
        reached_home = False
        while not reached_home and arm_device is not None:
            if api.is_api_exit():
                print("Public API has exited during return home.")
                break
            
            if arm_device.has_new_data():
                target_position, reached_home = return_home.get_target_position()
                arm_device.motor_command(CommandType.POSITION, target_position.tolist())
                
                # Print progress
                current_time = time.time()
                elapsed = current_time - return_home.start_time
                if int(elapsed * 10) % 5 == 0:  # Print every 0.5 seconds
                    progress = min(100, (elapsed / RETURN_HOME_DURATION) * 100)
                    print(f"Return progress: {progress:.1f}% - Position: {[f'{x:.3f}' for x in target_position]}")
            
            time.sleep(0.002)
        
        if reached_home:
            print("Successfully reached home position!")
        
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == "__main__":
    main()
