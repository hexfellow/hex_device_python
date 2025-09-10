import sys
import math
import numpy as np

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

TRAJ_TIME = 3
SPEED = 0.5
HEX_DEVICE_URL = "ws://192.168.1.100:8439"

class TrajectoryPlanner:
    """轨迹规划器，支持平滑的加减速规划"""
    
    def __init__(self, waypoints, segment_duration=3.0):
        """
        初始化轨迹规划器
        waypoints: 路径点列表
        segment_duration: 每段轨迹的持续时间（秒）
        """
        self.waypoints = waypoints
        self.segment_duration = segment_duration
        
        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.start_time = None
        
    def start_trajectory(self):
        """开始轨迹执行"""
        if not self.waypoints:
            return False
            
        self.trajectory_started = True
        self.start_time = time.time()
        self.current_waypoint_index = 0
        return True
        
    def get_current_target(self):
        """获取当前时刻的目标位置"""
        if not self.trajectory_started or not self.waypoints:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # 计算当前应该在哪个轨迹段
        total_segments = len(self.waypoints)
        segment_index = int(elapsed_time / self.segment_duration) % total_segments
        
        # 计算在当前段内的时间进度
        segment_elapsed = elapsed_time % self.segment_duration
        normalized_time = segment_elapsed / self.segment_duration
        
        # 获取当前段的起点和终点
        start_waypoint = self.waypoints[segment_index]
        end_waypoint = self.waypoints[(segment_index + 1) % total_segments]
        
        # 使用S曲线插值计算当前位置
        s = self._smooth_step(normalized_time)
        
        # 计算插值位置
        start_pos = np.array(start_waypoint)
        end_pos = np.array(end_waypoint)
        target_position = start_pos + s * (end_pos - start_pos)
        
        # 更新当前路径点索引（用于状态显示）
        self.current_waypoint_index = segment_index
        
        return target_position
        
    def _smooth_step(self, t):
        """S曲线插值函数，提供平滑的加减速"""
        # 限制t在[0,1]范围内
        t = max(0.0, min(1.0, t))
        
        # 使用5次多项式实现更平滑的插值：6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3
        
    def get_current_segment_info(self):
        """获取当前段的信息"""
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
    # Init HexDeviceApi
    api = HexDeviceApi(ws_url=HEX_DEVICE_URL, control_hz=250)
    first_time = True

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
                                    print("轨迹规划器已初始化，开始路径点规划")
                            
                            if trajectory_initialized:
                                target_positions = trajectory_planner.get_current_target()
                                
                                if target_positions is not None:
                                    # 发送位置命令
                                    device.motor_command(CommandType.POSITION, target_positions.tolist())
                                    
                                    segment_info = trajectory_planner.get_current_segment_info()
                                    if int(segment_info['total_elapsed'] * 10) % 5 == 0:  # 每0.5秒打印一次
                                        print(f"路径段: {segment_info['segment_index']} -> {(segment_info['segment_index'] + 1) % len(arm_position)}")
                                        print(f"段进度: {segment_info['segment_progress']:.2f}")
                                        print(f"目标位置: {[f'{x:.3f}' for x in target_positions]}")
                                        
                                        # 检查是否完成了一个完整的循环
                                        if segment_info['segment_index'] == 0 and segment_info['segment_progress'] < 0.1:
                                            loop_counter += 1
                                            print(f"--- 完成轨迹循环 {loop_counter} ---")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == "__main__":
    main()
