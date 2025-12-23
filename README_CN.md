# Hex Device Python 库

<p align="center">
	<a href="README_CN.md">中文</a> | <a href="README.md">English</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/hex_device_python/stargazers"><img src="https://img.shields.io/github/stars/hexfellow/hex_device_python?colorA=363a4f&colorB=b7bdf8&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/issues"><img src="https://img.shields.io/github/issues/hexfellow/hex_device_python?colorA=363a4f&colorB=f5a97f&style=for-the-badge"></a>
	<a href="https://github.com/hexfellow/hex_device_python/contributors"><img src="https://img.shields.io/github/contributors/hexfellow/hex_device_python?colorA=363a4f&colorB=a6da95&style=for-the-badge"></a>
</p>

## <a name="overview"></a> **概述**

本库提供了与 Hex 设备通信和控制的简单接口。它使用 Protocol Buffers 进行消息序列化，使用 WebSocket 进行实时通信。支持的硬件列表如下：
- [✅] **[ChassisMaver](#chassis_maver)**
- [✅] **[ChassisMark2](#chassis_mark2)**
- [✅] **[ChassisTriggerA3](#ChassisTriggerA3)**
- [✅] **[ArmArcher](#arm_archer)**
- [✅] **[ArmSaber](#arm_saber)**
- [✅] **[HandsHtGp100](#hands)**
- [✅] **[LiftLotaP1](#LiftLotaP1)**

## 前置要求

- **Python 3.9 或更高版本**
- Anaconda 发行版（推荐初学者使用）- 包含 Python、NumPy 和常用的科学计算包

## 安装

### 从 PyPI 安装（推荐）
```bash
pip install hex_device
```

### 克隆仓库（开发版本）
```bash
git clone --recurse-submodules https://github.com/hexfellow/hex_device_python.git
```

## 快速开始
如果你已经通过Pypi安装了该库，请直接查阅[使用方法](#使用方法)
### 安装 `protoc`
1. 从包管理器安装 protoc (仅适用于 Debian13/Ubuntu24.04)
    ```bash
    sudo apt install protobuf-compiler
    ```

2. 从 Github Releases 安装 protoc (适用于 Ubuntu22.04 及以下版本)
    
    选择一个合适的版本并安装。以下是一个安装 `protoc-27.1` 的示例。

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

### 安装 `hex_device`

**编译 Protocol Buffer 消息：**

   ```bash
   mkdir ./hex_device/generated
   protoc --proto_path=proto-public-api --python_out=hex_device/generated proto-public-api/*.proto
   ```

#### 选项 1：包安装

在您的 Python 环境中安装库：

```bash
python3 -m pip install .
```

#### 选项 2：直接使用（无需安装）

如果您更喜欢在不安装到 Python 环境的情况下运行库：

1. **安装依赖：**

    ```bash
    python3 -m pip install -r requirements.txt
    ```

2. **将库路径添加到您的脚本中：**

    ```python
    import sys
    sys.path.insert(1, '<your project path>/hex_device_python')
    sys.path.insert(1, '<your project path>/hex_device_python/hex_device/generated')
    ```

## 使用方法

> - **完整的函数说明可以在我们的 [wiki](https://github.com/hexfellow/hex_device_python/wiki/API-List) 中找到。**
> - **如果您使用的是机械臂，相关控制器的端口说明可以查看[docs](https://docs.hexfellow.com/hex-arm/controller.common_en/)。**

### 示例

- **所有设备的简单演示**：[tests/main.py](https://github.com/hexfellow/hex_device_python/blob/main/tests/main.py)
- **机械臂轨迹跟踪示例**：[tests/archer_traj_test.py](https://github.com/hexfellow/hex_device_python/blob/main/tests/archer_traj_test.py) 或 [tests/saber7dof_traj_test.py](https://github.com/hexfellow/hex_device_python/blob/main/tests/saber7dof_traj_test.py)

### 基本用法

**IPv4 连接：**
```bash
python3 tests/main.py --url ws://0.0.0.0:8439
```

**IPv6 连接：**
```bash
python3 tests/main.py --url ws://[fe80::500d:96ff:fee1:d60b%3]:8439
```

## 常见问题

### 如何使用 IPv6 进行连接？

您可以使用 IPv6 连接我们的设备，这样可以实现无需路由器的直接连接（例如，使用单根网线连接机器人和 PC）。

**注意：** 我们假设您对 IPv6 有基本了解。如果您不了解，请使用 IPv4。我们不会详细解释 IPv6。

**要点：**
- 即使没有 DHCP6，设备仍然可以拥有链路本地地址
- 要使用链路本地地址，必须使用 `%` 符号指定接口的区域 ID
- 您可以通过运行 `ip a` 来查找接口的区域 ID

**示例：**
```bash
# 查找接口和区域 ID
ip a

# 在连接 URL 中使用区域 ID
ws://[fe80::500d:96ff:fee1:d60b%3]:8439
```

### 为什么之前一直能使用的软件包，在某次重新部署后就无法正常使用了？

请查看 [Change log](https://github.com/hexfellow/hex_device_python/wiki/Change-Log)，检查是否使用了跨版本的软件包。同时，我们建议您在成功部署之后就使用固定的软件版本，以避免由于非兼容性更新导致您的代码失效。

### 如果我想使用更新的软件包，如何进行硬件升级？

请联系我们的售后服务，我们将根据您所购买的设备提供硬件升级说明。

--- 

<p align="center">
	Copyright &copy; 2025-present <a href="https://github.com/hexfellow" target="_blank">Hexfellow Org</a>
</p>

<p align="center">
	<a href="https://github.com/hexfellow/robot_hardware_interface/blob/main/LICENSE"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=Apache&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>
