import sys
import math
import numpy as np
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
from hex_device.hands import Hands

TRAJ_TIME = 3
SPEED = 0.5

class TrajectoryPlanner:
    """Trajectory planner that supports smooth acceleration and deceleration planning"""
    
    def __init__(self, waypoints, segment_duration=3.0):
        """
        Initialize trajectory planner
        waypoints: List of waypoints
        segment_duration: Duration of each trajectory segment (seconds)
        """
        self.waypoints = waypoints
        self.segment_duration = segment_duration
        
        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.start_time = None
        
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
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        total_segments = len(self.waypoints)
        segment_index = int(elapsed_time / self.segment_duration) % total_segments
        
        segment_elapsed = elapsed_time % self.segment_duration
        normalized_time = segment_elapsed / self.segment_duration
        
        start_waypoint = self.waypoints[segment_index]
        end_waypoint = self.waypoints[(segment_index + 1) % total_segments]
        
        # Use S-curve interpolation to calculate current position
        s = self._smooth_step(normalized_time)
        
        start_pos = np.array(start_waypoint)
        end_pos = np.array(end_waypoint)
        target_position = start_pos + s * (end_pos - start_pos)
        
        self.current_waypoint_index = segment_index
        
        return target_position
        
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
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        segment_index = int(elapsed_time / self.segment_duration) % len(self.waypoints)
        segment_elapsed = elapsed_time % self.segment_duration
        segment_progress = segment_elapsed / self.segment_duration
        
        return {
            'segment_index': segment_index,
            'segment_progress': segment_progress,
            'total_elapsed': elapsed_time
        }

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
    hands_first_time = True

    arm_position = [
        [0.0, 0.223598775598, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.623598775598, 1.59439265359, 1.57, -1.0472, 0.0],
        [0.0, 0.223598775598, 0.0, 0.0, 0.0, 0.0],
        [-0.5, 0.623598775598, 1.59439265359, -1.57, 1.0472, 0.0]
    ]

    # arm_position = [
    #     [0.0, 0.79, 1.81, -1.14, 0.0, 0.0],
    #     [0.0, -0.91, 2.44, 0.0, 0.0, 0.0],
    #     [0.0, 0.79, 1.81, -1.14, 0.0, 0.0],
    #     [0.0, -0.91, 2.44, 0.0, 0.0, 0.0],
    # ]
    
    trajectory_planner = TrajectoryPlanner(
        waypoints=arm_position,
        segment_duration=TRAJ_TIME
    )
    
    trajectory_initialized = False
    loop_counter = 0
    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                for device in api.device_list:
                    if isinstance(device, ArmArcher):
                        if device.has_new_data():
                            if first_time:
                                first_time = False
                                config_dict = {
                                    'name':'Archer_d6y',
                                    'dof_num': 'six_axis',
                                    'motor_model': [0x80] * 6,
                                    'joints': [{
                                        'joint_name': 'joint_1',
                                        'joint_limit': [-2.7, 3.1, -0.1, 0.1, 0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_2',
                                        'joint_limit': [-1.57, 2.094, -SPEED, SPEED, 0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_3',
                                        'joint_limit': [0.0, 3.14159265359, -SPEED, SPEED, 0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_4',
                                        'joint_limit': [-1.5, 1.5, -SPEED, SPEED, 0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_5',
                                        'joint_limit': [-1.56, 1.56, -SPEED, SPEED, 0.0, 0.0]
                                    }, {
                                        'joint_name': 'joint_6',
                                        'joint_limit': [-1.57, 1.57, -SPEED, SPEED, 0.0, 0.0]
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
                                        print(f"Path segment: {segment_info['segment_index']} -> {(segment_info['segment_index'] + 1) % len(arm_position)}")
                                        print(f"Segment progress: {segment_info['segment_progress']:.2f}")
                                        print(f"Target position: {[f'{x:.3f}' for x in target_positions]}")
                                        
                                        # Check if a complete loop has been completed
                                        if segment_info['segment_index'] == 0 and segment_info['segment_progress'] < 0.1:
                                            loop_counter += 1
                                            print(f"--- Completed trajectory loop {loop_counter} ---")

                for device in api.optional_device_list:
                    if isinstance(device, Hands):
                        if device.has_new_data():
                            if hands_first_time:
                                hands_first_time = False
                                device.set_positon_step(0.02)
                                device.set_pos_torque(3.0)
                                
                            current_positions = device.get_motor_positions()
                            print(f"hands position: {current_positions}")
                            
                            # Create sinusoidal motion for smooth interpolation
                            t = time.time()
                            interpolation_factor = (math.sin(t * 0.5) + 1.0) / 2.0  # 0 to 1
                            # eventhough the range is [-1.56, 1.57], the command will limit the position to the range [0.0, 1.335]
                            min_pos = -1.56
                            target_position = min_pos + interpolation_factor * (1.57 - min_pos)
                            # Apply to first motor (or all motors if desired)
                            target_positions = [target_position] + [0.0] * (device.motor_count - 1)
                            
                            # For hands, only support the position mode or mit mode, also mit mode only read the position data.
                            # example 1:
                            mit_commands = device.construct_mit_command(
                                target_positions, 
                                [0.0], 
                                [0.0], 
                                [0.0], 
                                [0.0]
                            )
                            device.motor_command(
                                CommandType.MIT,
                                mit_commands)

                            # example 2:
                            # device.motor_command(
                            #     CommandType.POSITION,
                            #     target_positions
                            # )

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
