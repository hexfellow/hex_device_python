# Hex Vehicle Python Library

A Python library for controlling hex vehicles through a WebSocket-based API.

## Overview

This library provides a simple interface for communicating with and controlling hex vehicles. It uses Protocol Buffers for message serialization and WebSocket for real-time communication.

## Clone
```
git clone --recurse-submodules https://github.com/hexfellow/hex_device_python_lib.git
```

## Prerequisites

- **Python 3.8.10 or higher**
- Anaconda Distribution (recommended for beginners) - includes Python, NumPy, and commonly used scientific computing packages

## Quickstart

### Option 1: Direct Usage (No Installation)

If you prefer to run the library without installing it in your Python environment:

1. **Compile Protocol Buffer messages:**
   ```bash
   mkdir ./hex_device/generated
   protoc --proto_path=proto-public-api --python_out=hex_device/generated proto-public-api/*.proto
   ```
   
   **Note:** This library requires newer protoc. If compilation fails, please try to install protoc-27.1 using the binary installation method below.
   **Installing protoc-27.1:**
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

2. **Add the library path to your script:**
   ```python
   import sys
   sys.path.insert(1, '<your project path>/hex_device_python_lib')
   sys.path.insert(1, '<your project path>/hex_device_python_lib/hex_device/generated')
   ```

3. **Install dependencies**
   ```bash
   pip install websockets
   pip install pygame
   ```

4. **Run your test script:**
   ```bash
   python3 tests/<your_script>.py
   ```

### Option 2: Package Installation

To install the library in your Python environment:

1. **Build the package:**
   ```bash
   python3 -m pip install .
   ```

2. **Run your test script:**
   ```bash
   python3 tests/<your_script>.py
   ```

## Usage

All vehicle control interfaces are provided through the `VehicleAPI` class:

```python
from hex_device import PublicAPI as VehicleAPI

# Initialize the API
api = VehicleAPI(ws_url="ws://your_vehicle_ip:8439", control_hz=200, control_mode="speed")

# Get vehicle interface
vehicle = api.vehicle

# Control the vehicle
vehicle.set_target_vehicle_speed(x_speed, y_speed, rotation_speed)
```

## Architecture

The library consists of three main modules:

### 1. public_api
- **Network Manager**: Handles Protocol Buffer message construction and WebSocket communication
- Manages data transmission to and from the vehicle

### 2. vehicle  
- **Vehicle Data Manager**: Continuously updates vehicle data in a loop
- Provides interfaces for:
  - Obtaining chassis status
  - Controlling vehicle movement
  - Reading motor data (velocity, torque, position)

### 3. utils
- **General Tools**: Parameter management and common utility functions
- Provides helper functions for various operations

## Examples

Please refer to the `tests/` directory for example usage:
- `main.py` - Basic vehicle control example
- `main_pygame.py` - Joystick control example using pygame.
- `vehicle_calc.py` - A simple kinematics solution.

Additional application projects can be found here:
[tidybot2](https://github.com/hexfellow/tidybot2/blob/main/base_controller_pygame.py)

## Requirements

- numpy>=1.17.4
- protobuf>=5.29.4,<6.0.0
- websockets
- pygame (for joystick control examples)