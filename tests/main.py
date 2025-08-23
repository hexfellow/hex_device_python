import sys
sys.path.insert(1, '/Users/jecjune/Downloads/python/hex_device_python')
sys.path.insert(1, '/Users/jecjune/Downloads/python/hex_device_python/hex_device/generated')

from hex_device import HexDeviceApi
import time
from hex_device.chassis_maver import ChassisMaver
from hex_device.motor_base import CommandType

def main():
    # Init HexDeviceApi
    api = HexDeviceApi(ws_url = "ws://172.18.8.139:8439", control_hz = 200)

    odom_clear = False
    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                # Get raw data
                data, count = api._get_raw_data()
                if data != None:
                    for device in api.device_list:
                        if isinstance(device, ChassisMaver):
                            # device.get_status_summary()

                            if not odom_clear:
                                device.clear_odom_bias()
                                odom_clear = True

                            print(f"vehicle position: {device.get_vehicle_position()}")
                            device.enable()
                            # device.set_vehicle_speed(0.0, 0.0, 0.0)
                            # device.motor_command(CommandType.SPEED, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

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