# Hex Device Python Library

<p align="center">
	<a href="https://github.com/hexfellow/hex_device_python/stargazers"><img src="https://img.shields.io/github/stars/hexfellow/hex_device_python?colorA=363a4f&colorB=b7bdf8&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/issues"><img src="https://img.shields.io/github/issues/hexfellow/hex_device_python?colorA=363a4f&colorB=f5a97f&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/contributors"><img src="https://img.shields.io/github/contributors/hexfellow/hex_device_python?colorA=363a4f&colorB=a6da95&style=for-the-badge"></a>
</p>

## <a name="overview"></a> **Overview**

This library provides a simple interface for communicating with and controlling Hex devices. It uses Protocol Buffers for message serialization and WebSocket for real-time communication. The supported hardware list is as follows:
- [✅] **[ChassisMaver](#chassis_maver)**
- [✅] **[ChassisMark2](#chassis_mark2)**
- [✅] **[ArmArcher](#arm_archer)**
- [✅] **[HandsHtGp100](#hands)**
- [-] **[hex_lift](#hex_lift)**


## Installation

### Install from PyPI (Recommended)
```bash
pip install hex_device
```

### Clone the Repository (Development)
```bash
git clone --recurse-submodules https://github.com/hexfellow/hex_device_python.git
```

## Prerequisites

- **Python 3.8.10 or higher**
- Anaconda Distribution (recommended for beginners) - includes Python, NumPy, and commonly used scientific computing packages

## Quickstart

### Install `protoc`

We highly recommend you to use **`protoc-27.1`** since we have fully tested it in both `x86_64` and `arm64` archs.

You can use the binary installation method below to install **`protoc-27.1`**.

```bash
# For Linux x86_64
wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-x86_64.zip
sudo unzip protoc-27.1-linux-x86_64.zip -d /usr/local
rm protoc-27.1-linux-x86_64.zip
   
# For Linux arm64
wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-aarch_64.zip
sudo unzip protoc-27.1-linux-aarch_64.zip -d /usr/local
rm protoc-27.1-linux-aarch_64.zip
   
# Verify installation
protoc --version  # Should display libprotoc 27.1
```

### Install `hex_device`

#### Option 1: Package Installation

To install the library in your Python environment:

```bash
python3 -m pip install .
```

#### Option 2: Direct Usage (No Installation)

If you prefer to run the library without installing it in your Python environment:

1. **Compile Protocol Buffer messages:**

   ```bash
   mkdir ./hex_device/generated
   protoc --proto_path=proto-public-api --python_out=hex_device/generated proto-public-api/*.proto
   ```

2. **Install dependencies:**

    ```bash
    python3 -m pip install -r requirements.txt
    ```

3. **Add the library path to your script:**

    ```python
    import sys
    sys.path.insert(1, '<your project path>/hex_device_python')
    sys.path.insert(1, '<your project path>/hex_device_python/hex_device/generated')
    ```

## Usage

> **The detailed function interfaces can be found in our [wiki](https://github.com/hexfellow/hex_device_python/wiki/API-List).**

### <a name="chassis_maver"></a> For chassis_maver <small><sup>[overview ▲](#overview)</sup></small>
```python
api = HexDeviceApi(ws_url="ws://<device ip>:8439", control_hz=500)
try:
    first_time = True
    while True:
        if api.is_api_exit():
            print("Public API has exited.")
            break
        else:
            for device in api.device_list:
                if isinstance(device, ChassisMaver):
                    if device.has_new_data():
                        if first_time:
                            first_time = False
                            device.clear_odom_bias()
                        print(device.get_device_summary())
                        print(f"vehicle position: {device.get_vehicle_position()}")
                        device.set_vehicle_speed(0.0, 0.0, 0.0)
        time.sleep(0.004)
except KeyboardInterrupt:
    print("Received Ctrl-C.")
    api.close()
finally:
    print("Resources have been cleaned up.")
exit(0)
```

### <a name="chassis_mark2"></a> For chassis_mark2 <small><sup>[overview ▲](#overview)</sup></small>
```python
api = HexDeviceApi(ws_url="ws://<device ip>:8439", control_hz=500)
try:
    first_time = True
    while True:
        if api.is_api_exit():
            print("Public API has exited.")
            break
        else:
            for device in api.device_list:
                if isinstance(device, ChassisMark2):
                    if device.has_new_data():
                        if first_time:
                            first_time = False
                            device.clear_odom_bias()

                        print(device.get_device_summary())
                        print(f"vehicle position: {device.get_vehicle_position()}")
                        ## command, Please select one of the following commands.
                        device.set_vehicle_speed(0.0, 0.0, 0.0)
                        # device.motor_command(CommandType.SPEED, [0.4, 0.4])
        time.sleep(0.004)
except KeyboardInterrupt:
    print("Received Ctrl-C.")
    api.close()
finally:
    print("Resources have been cleaned up.")
exit(0)
```

### <a name="arm_archer"></a> For arm_archer <small><sup>[overview ▲](#overview)</sup></small>

A path trajectory example is provided:
```bash
python3 tests/archer_traj_test.py  --url <ip>:<8439 or 9439>
```

A simple usage example is shown below:

```python
api = HexDeviceApi(ws_url="ws://<device ip>:8439", control_hz=250)
try:
    while True:
        if api.is_api_exit():
            print("Public API has exited.")
            break
        else:
            for device in api.device_list:
                if isinstance(device, ArmArcher):
                    print(device.get_device_summary())
                    print(f"motor position: {device.get_motor_positions()}")
                    device.motor_command(
                        CommandType.SPEED,
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(0.004)
except KeyboardInterrupt:
    print("Received Ctrl-C.")
    api.close()
finally:
    print("Resources have been cleaned up.")
exit(0)
```

### <a name="hands"></a> For hands <small><sup>[overview ▲](#overview)</sup></small>
```python
api = HexDeviceApi(ws_url="ws://<device ip>:8439", control_hz=250)
hands = None
while hands == None:
    hands = api.find_optional_device('hand_status')
    time.sleep(0.1)
try:
    while True:
        if api.is_api_exit():
            print("Public API has exited.")
            break
        else:
            if hands:
                if hands.has_new_data():
                    print(device.get_device_summary())
                    print(f"motor position: {device.get_motor_positions()}")
                    device.motor_command(
                        CommandType.POSITION,
                        [0.0])
        time.sleep(0.004)
except KeyboardInterrupt:
    print("Received Ctrl-C.")
    api.close()
finally:
    print("Resources have been cleaned up.")
exit(0)
```

### <a name="hex_lift"></a> For hex_lift <small><sup>[overview ▲](#overview)</sup></small>
Coming soon...

--- 

<p align="center">
	Copyright &copy; 2025-present <a href="https://github.com/hexfellow" target="_blank">Hexfellow Org</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/robot_hardware_interface/blob/main/LICENSE"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=Apache&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>