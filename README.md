# Hex Device Python Library

<p align="center">
	<a href="README_CN.md">中文</a> | <a href="README.md">English</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/hex_device_python/stargazers"><img src="https://img.shields.io/github/stars/hexfellow/hex_device_python?colorA=363a4f&colorB=b7bdf8&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/issues"><img src="https://img.shields.io/github/issues/hexfellow/hex_device_python?colorA=363a4f&colorB=f5a97f&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/contributors"><img src="https://img.shields.io/github/contributors/hexfellow/hex_device_python?colorA=363a4f&colorB=a6da95&style=for-the-badge"></a>
</p>

## <a name="overview"></a> **Overview**

This library provides a simple interface for communicating with and controlling Hex devices. It uses Protocol Buffers for message serialization and WebSocket for real-time communication. The supported hardware list is as follows:
- [✅] **[ChassisMaver](#chassis_maver)**
- [✅] **[ChassisMark2](#chassis_mark2)**
- [✅] **[ChassisTriggerA3](#ChassisTriggerA3)**
- [✅] **[ArmArcher](#arm_archer)**
- [✅] **[ArmSaber](#arm_saber)**
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

- **Python 3.9 or higher**
- Anaconda Distribution (recommended for beginners) - includes Python, NumPy, and commonly used scientific computing packages

## Quickstart

### Install `protoc`

1. Install protoc from package manager (Recommended only for Debian13/Ubuntu24.04)
    ```bash
    sudo apt install protobuf-compiler
    ```

2. Install protoc from Github Releases (Recommended Ubuntu22.04 and below)
    
    Just choose a suitable version and install it. Here below is an example of installing `protoc-27.1`. 

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
    protoc --version # Should be or more than 3.21.12
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

> **The complete function interfaces can be found in our [wiki](https://github.com/hexfellow/hex_device_python/wiki/API-List).**

- A simple demo for all device: [test/main.py](https://github.com/hexfellow/hex_device_python/blob/main/tests/main.py)

- Example of robotic arm trajectory tracking: [test/archer_traj_test.py](https://github.com/hexfellow/hex_device_python/blob/main/tests/archer_traj_test.py)


## Q&A
Why did a previously working software package stop working after a redeployment?
- Please check the [Change log](https://github.com/hexfellow/hex_device_python/wiki/Change-Log) to see if you are using cross-version software packages

If I want to use newer software packages, how do I perform hardware upgrades?
- Please contact our after-sales service, and we will provide hardware upgrade instructions based on the equipment you purchased
--- 

<p align="center">
	Copyright &copy; 2025-present <a href="https://github.com/hexfellow" target="_blank">Hexfellow Org</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/robot_hardware_interface/blob/main/LICENSE"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=Apache&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>