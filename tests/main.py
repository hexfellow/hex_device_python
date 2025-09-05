import sys

sys.path.insert(1, '/Users/jecjune/Downloads/python/hex_device_python')
sys.path.insert(
    1,
    '/Users/jecjune/Downloads/python/hex_device_python/hex_device/generated')

from hex_device import HexDeviceApi
import time
from hex_device.chassis_maver import ChassisMaver
from hex_device.motor_base import CommandType
from hex_device.arm_archer import ArmArcher


def main():
    # Init HexDeviceApi
    api = HexDeviceApi(ws_url="ws://172.18.8.161:8439", control_hz=200)

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                # Get raw data
                for device in api.device_list:
                    if isinstance(device, ChassisMaver):
                        # device.get_device_summary()
                        print(
                            f"vehicle position: {device.get_vehicle_position()}"
                        )
                        device.enable()
                        # device.set_vehicle_speed(0.0, 0.0, 0.0)
                        # device.motor_command(CommandType.SPEED, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
                    elif isinstance(device, ArmArcher):
                        config_dict = {
                            'name':'Archer_d6y',
                            'dof_num': 'six_axis',
                            'motor_model': [0x80] * 6,
                            'joints': [{
                                'joint_name': 'joint_1',
                                'joint_limit': [-3.0, 3.0, -4.0, 4.0, -0.0, 0.0]
                            }, {
                                'joint_name': 'joint_2',
                                'joint_limit': [-1.5, 2.0, -4.5, 4.5, -0.0, 0.0]
                            }, {
                                'joint_name': 'joint_3',
                                'joint_limit': [-0.5, 3.0, -4.5, 4.5, -0.0, 0.0]
                            }, {
                                'joint_name': 'joint_4',
                                'joint_limit': [-3.0, 3.0, -4.5, 4.5, -0.0, 0.0]
                            }, {
                                'joint_name': 'joint_5',
                                'joint_limit': [-1.5, 1.5, -4.5, 4.5, -0.0, 0.0]
                            }, {
                                'joint_name': 'joint_6',
                                'joint_limit': [-1.5, 1.5, -4.5, 4.5, -0.0, 0.0]
                            }]
                        }
                        if device.reload_arm_config_from_dict(config_dict):
                            exit(1)

                        # print(device.get_device_summary())
                        # print(device.get_motor_summary())

                        # print(f"arm position: {device.get_motor_positions()}")
                        device.motor_command(
                            CommandType.POSITION,
                            [-0.3, -1.48, 2.86, 0.0, -0.0, 0.0])

            time.sleep(0.0005)

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == "__main__":
    main()
