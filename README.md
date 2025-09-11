# Hex Device Python Library

<p align="center">
	<a href="https://github.com/hexfellow/hex_device_python/stargazers"><img src="https://img.shields.io/github/stars/hexfellow/hex_device_python?colorA=363a4f&colorB=b7bdf8&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/issues"><img src="https://img.shields.io/github/issues/hexfellow/hex_device_python?colorA=363a4f&colorB=f5a97f&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/contributors"><img src="https://img.shields.io/github/contributors/hexfellow/hex_device_python?colorA=363a4f&colorB=a6da95&style=for-the-badge"></a>
</p>

## <a name="overview"></a> **Overview**

This library provides a simple interface for communicating with and controlling hex device. It uses Protocol Buffers for message serialization and WebSocket for real-time communication.The supported hardware list is as follows:
- [✅] **[ChassisMaver](#chassis_maver)**
- [✅] **[ArmArcher](#arm_archer)**
- [-] **[hex_lift](#hex_lift)**


## Clone
```
git clone --recurse-submodules https://github.com/hexfellow/hex_device_python.git
```

## Prerequisites

- **Python 3.8.10 or higher**
- Anaconda Distribution (recommended for beginners) - includes Python, NumPy, and commonly used scientific computing packages

## Quickstart

### Option 1: Package Installation

To install the library in your Python environment:

```
python3 -m pip install .
```

### Option 2: Direct Usage (No Installation)

If you prefer to run the library without installing it in your Python environment:

1. **Compile Protocol Buffer messages:**
   ```bash
   mkdir ./hex_device/generated
   protoc --proto_path=proto-public-api --python_out=hex_device/generated proto-public-api/*.proto
   ```
   
   **Note:** This library requires newer protoc. If compilation fails, please try to install protoc-27.1 using the binary installation method below.

   **Installing protoc-27.1 through binary:**
   ```bash
   # For Linux x86_64
   wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-x86_64.zip
   sudo unzip protoc-27.1-linux-x86_64.zip -d /usr/local
   rm protoc-27.1-linux-x86_64.zip
   
   # For Linux arm64
   wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-aarch_64.zip
   sudo unzip protoc-27.1-linux-aarch_64.zip -d /usr/local
   rm protoc-27.1-linux-aarch_64.zip
   
   #  Verify installation
   protoc --version  # Should show libprotoc 27.1
   ```

2. **Install dependencies:**
```bash
python3 -m pip install -r requirements.txt
```

3. **Add the library path to your script:**
Add the library path to your script:
```python
import sys
sys.path.insert(1, '<your project path>/hex_device_python')
sys.path.insert(1, '<your project path>/hex_device_python/hex_device/generated')
```

## Usage

> **The detailed function interfaces can be found in our [wiki](https://github.com/hexfellow/hex_device_python/wiki).**

### <a name="chassis_maver"></a> For chassis_maver <small><sup>[overview ▲](#overview)</sup></small>
```python
api = HexDeviceApi(ws_url="ws://<device ip>:8439", control_hz=250)
try:
    while True:
        if api.is_api_exit():
            print("Public API has exited.")
            break
        else:
            for device in api.device_list:
                # for ChassisMaver
                if isinstance(device, ChassisMaver):
                    print(device.get_device_summary())
                    print(
                        f"vehicle position: {device.get_vehicle_position()}"
                    )
                    device.set_vehicle_speed(0.0, 0.0, 0.0)
        time.sleep(0.004)
except KeyboardInterrupt:
    print("Received Ctrl-C.")
    api.close()
finally:
    pass

print("Resources have been cleaned up.")
exit(0)
```

### <a name="arm_archer"></a> For arm_archer <small><sup>[overview ▲](#overview)</sup></small>
```python
api = HexDeviceApi(ws_url="ws://<device ip>:8439", control_hz=250)
try:
    while True:
        if api.is_api_exit():
            print("Public API has exited.")
            break
        else:
            for device in api.device_list:
                # for ArmArcher
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
    pass

print("Resources have been cleaned up.")
exit(0)
```

### <a name="hex_lift"></a> For lift <small><sup>[overview ▲](#overview)</sup></small>
waiting...

--- 

<p align="center">
	Copyright &copy; 2025-present <a href="https://github.com/hexfellow" target="_blank">Hexfellow Org</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/robot_hardware_interface/blob/main/LICENSE"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=Apache&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>