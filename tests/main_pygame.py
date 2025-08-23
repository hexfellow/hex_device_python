import sys
sys.path.insert(1, '<your project path>/hex_device_python_lib')
sys.path.insert(1, '<your project path>/hex_device_python_lib/hex_device/generated')

from hex_device import PublicAPI as VehicleAPI
import time
import pygame

def init_pygame_joystick():
    """Initialize pygame and controller"""
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected.")
        exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected handle: {joystick.get_name()}")
    print()
    return joystick

def main():
    # init pygame
    joystick = init_pygame_joystick()
    
    # init PublicAPI
    api = VehicleAPI(ws_url = "ws://172.18.2.66:8439", control_hz = 200, control_mode = "speed")

    # get velocity interface
    velocity_interface = api.vehicle

    cmd_x = 0.0
    cmd_y = 0.0
    angle = 0.0
    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                data, count = api._get_raw_data()
                if data != None:
                    pass
                    # print("count = ", count)

                if velocity_interface.has_new_data():
                    velocity = velocity_interface.get_motor_velocity()
                    # print("velocity:", velocity)
                    tor = velocity_interface.get_motor_torque()
                    # print("tor:", tor)
                    position = velocity_interface.get_motor_position()
                    # print("position:", position)

                # get handle input
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        axis_0 = joystick.get_axis(0)
                        axis_1 = -joystick.get_axis(1)
                        axis_2 = joystick.get_axis(2)
                        axis_3 = -joystick.get_axis(3)

                        print(f"左摇杆 X: {axis_0:.2f}, Y: {axis_1:.2f}，右摇杆 X: {axis_2:.2f}, Y: {axis_3:.2f}")

                        cmd_x = axis_0 * 0.5
                        cmd_y = axis_1 * 0.5
                        angle = axis_3 * 1.0
                
                
                if abs(cmd_x) > 0.01 or abs(cmd_y) > 0.01 or abs(angle) > 0.01:
                    print(f"target speed: X={cmd_x:.2f}, Y={cmd_y:.2f}, Z={angle:.2f}")
                
                velocity_interface.set_target_vehicle_speed(cmd_y, cmd_x, angle)
                # velocity_interface.set_motor_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            time.sleep(0.0005)
            
    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        if joystick:
            joystick.quit()
        pygame.quit()

    print("Resources have been cleaned up.")
    exit(0)

if __name__ == "__main__":
    main()