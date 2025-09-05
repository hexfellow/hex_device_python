#!/usr/bin/env python3
# -*- coding: utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
"""
机械臂配置系统使用示例

这个模块展示了如何使用类似于 Rust 的 load_default_arm_config 功能
来管理不同系列的机械臂参数配置。

主要功能：
1. 预定义机械臂配置（类似于 Rust 代码中的 HashMap<i64, ArmConfig>）
2. 根据机械臂系列获取配置
3. 关节限制验证
4. 动态添加新配置
5. 在 ArmArcher 类中集成使用
"""

from .arm_config import (load_default_arm_config, get_arm_config,
                         add_arm_config, ArmConfig, DofType, JointParam,
                         JointParams)


def example_basic_usage():
    """基本使用示例"""
    print("=== 基本使用示例 ===")

    # 1. 加载所有默认配置
    configs = load_default_arm_config()
    print(f"加载了 {len(configs)} 个默认配置")

    # 2. 获取特定系列的配置
    saber_d6x_config = get_arm_config(0x0E)  # saber_d6x
    if saber_d6x_config:
        print(f"找到 saber_d6x 配置: {saber_d6x_config.name}")
        print(f"  自由度: {saber_d6x_config.dof_num.value}")
        print(f"  电机数量: {len(saber_d6x_config.motor_model)}")
        print(f"  关节数量: {len(saber_d6x_config.robot_config.joints)}")

    # 3. 获取关节限制
    joint_limits = saber_d6x_config.robot_config.joints[0].joint_limit
    print(f"  第一个关节限制: {joint_limits}")
    print()


def example_joint_validation():
    """关节验证示例"""
    print("=== 关节验证示例 ===")

    config = get_arm_config(0x0E)  # saber_d6x
    if not config:
        return

    # 测试位置超出限制
    print("1. 测试位置超出限制:")
    positions = [5.0, 0.0, 1.57, 0.0, 0.0, 0.0]  # 第一个关节超出范围
    print(f"   原始位置: {positions}")

    validated_positions = arm_config_manager.validate_joint_positions(
        0x0E, positions)
    print(f"   验证后位置: {validated_positions}")

    # 检查第一个关节是否被修正到边界
    joint_1_limit = config.robot_config.joints[0].joint_limit
    expected_pos_1 = joint_1_limit[1]  # max_pos
    print(f"   第一个关节限制: [{joint_1_limit[0]:.3f}, {joint_1_limit[1]:.3f}]")
    print(f"   期望位置: {expected_pos_1:.3f}")
    print(f"   实际位置: {validated_positions[0]:.3f}")
    print(
        f"   验证结果: {'通过' if abs(validated_positions[0] - expected_pos_1) < 0.001 else '失败'}"
    )
    print()

    # 测试位置在限制范围内
    print("2. 测试位置在限制范围内:")
    positions = [0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
    print(f"   原始位置: {positions}")

    validated_positions = arm_config_manager.validate_joint_positions(
        0x0E, positions)
    print(f"   验证后位置: {validated_positions}")

    # 检查位置是否保持不变
    unchanged = all(
        abs(orig - val) < 0.001
        for orig, val in zip(positions, validated_positions))
    print(f"   验证结果: {'通过' if unchanged else '失败'}")
    print()

    # 测试速度超出限制
    print("3. 测试速度超出限制:")
    positions = [0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
    velocities = [0.0, 0.0, 6.0, 0.0, 0.0, 0.0]  # 第三个关节速度超出限制 (max_vel = 4.5)
    dt = 0.001  # 1ms

    print(f"   原始位置: {positions}")
    print(f"   当前速度: {velocities}")
    print(f"   时间步长: {dt}s")

    validated_positions = arm_config_manager.validate_joint_positions(
        0x0E, positions, velocities, dt)
    print(f"   验证后位置: {validated_positions}")

    # 检查第三个关节是否根据速度限制进行了调整
    joint_3_limit = config.robot_config.joints[2].joint_limit
    max_vel = joint_3_limit[3]  # max_vel
    expected_displacement = max_vel * dt
    expected_pos_3 = positions[2] + expected_displacement

    print(
        f"   第三个关节速度限制: [{joint_3_limit[2]:.1f}, {joint_3_limit[3]:.1f}] rad/s"
    )
    print(f"   期望位移: {expected_displacement:.6f} rad")
    print(f"   期望位置: {expected_pos_3:.6f} rad")
    print(f"   实际位置: {validated_positions[2]:.6f} rad")
    print(
        f"   验证结果: {'通过' if abs(validated_positions[2] - expected_pos_3) < 0.0001 else '失败'}"
    )
    print()

    # 测试位置和速度都超出限制
    print("4. 测试位置和速度都超出限制:")
    positions = [5.0, 0.0, 1.57, 0.0, 0.0, 0.0]  # 第一个关节位置超出
    velocities = [0.0, 0.0, 6.0, 0.0, 0.0, 0.0]  # 第三个关节速度超出
    dt = 0.001

    print(f"   原始位置: {positions}")
    print(f"   当前速度: {velocities}")

    validated_positions = arm_config_manager.validate_joint_positions(
        0x0E, positions, velocities, dt)
    print(f"   验证后位置: {validated_positions}")

    # 检查第一个关节是否被限制在边界
    joint_1_limit = config.robot_config.joints[0].joint_limit
    expected_pos_1 = joint_1_limit[1]  # max_pos

    # 检查第三个关节是否根据速度限制进行了调整
    joint_3_limit = config.robot_config.joints[2].joint_limit
    max_vel = joint_3_limit[3]  # max_vel
    expected_displacement = max_vel * dt
    expected_pos_3 = positions[2] + expected_displacement

    print(f"   第一个关节期望位置: {expected_pos_1:.3f} rad")
    print(f"   第一个关节实际位置: {validated_positions[0]:.3f} rad")
    print(f"   第三个关节期望位置: {expected_pos_3:.6f} rad")
    print(f"   第三个关节实际位置: {validated_positions[2]:.6f} rad")

    pos_1_ok = abs(validated_positions[0] - expected_pos_1) < 0.001
    pos_3_ok = abs(validated_positions[2] - expected_pos_3) < 0.0001
    print(f"   验证结果: {'通过' if pos_1_ok and pos_3_ok else '失败'}")
    print()


def example_add_custom_config():
    """添加自定义配置示例"""
    print("=== 添加自定义配置示例 ===")

    # 创建自定义配置
    custom_config = ArmConfig(
        name="my_custom_arm",
        dof_num=DofType.SIX_AXIS,
        arm_series=0x99,
        motor_model=[0x80] * 6,
        robot_config=JointParams(joints=[
            JointParam(joint_name="joint_1",
                       joint_limit=[-3.14, 3.14, -4.5, 4.5, -0.0, 0.0]),
            JointParam(joint_name="joint_2",
                       joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
            JointParam(joint_name="joint_3",
                       joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
            JointParam(joint_name="joint_4",
                       joint_limit=[-3.14, 3.14, -4.5, 4.5, -0.0, 0.0]),
            JointParam(joint_name="joint_5",
                       joint_limit=[-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]),
            JointParam(joint_name="joint_6",
                       joint_limit=[-3.14, 3.14, -4.5, 4.5, -0.0, 0.0])
        ]))

    # 添加配置
    add_arm_config(0x99, custom_config)
    print(f"添加了自定义配置: 系列 0x99 - {custom_config.name}")

    # 验证添加成功
    retrieved_config = get_arm_config(0x99)
    if retrieved_config:
        print(f"验证成功: {retrieved_config.name}")
    print()


def example_reload_config():
    """重载配置示例"""
    print("=== 重载配置示例 ===")

    # 获取原始配置
    original_config = get_arm_config(0x0E)
    if not original_config:
        print("未找到原始配置")
        return

    print(f"原始配置: {original_config.name}")
    print(f"第一个关节限制: {original_config.robot_config.joints[0].joint_limit}")

    # 从字典重载配置
    print("从字典重载配置:")
    config_dict = {
        'name':
        'saber_d6x_from_dict',
        'dof_num':
        'six_axis',
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

    success = arm_config_manager.reload_from_dict(0x0E, config_dict)
    print(f"从字典重载结果: {'成功' if success else '失败'}")

    # 验证重载结果
    dict_reloaded_config = get_arm_config(0x0E)
    if dict_reloaded_config:
        print(f"字典重载后配置: {dict_reloaded_config.name}")
        print(
            f"第一个关节限制: {dict_reloaded_config.robot_config.joints[0].joint_limit}"
        )
        print(
            f"验证结果: {'通过' if dict_reloaded_config.name == 'saber_d6x_from_dict' else '失败'}"
        )
    print()

    # 再次重载，修改关节限制
    print("再次重载，修改关节限制:")
    config_dict2 = {
        'name':
        'saber_d6x_modified',
        'dof_num':
        'six_axis',
        'motor_model': [0x80] * 6,
        'joints': [
            {
                'joint_name': 'joint_1',
                'joint_limit': [-3.14, 3.14, -5.0, 5.0, -0.0, 0.0]  # 修改限制
            },
            {
                'joint_name': 'joint_2',
                'joint_limit': [-1.57, 2.094, -4.5, 4.5, -0.0, 0.0]
            },
            {
                'joint_name': 'joint_3',
                'joint_limit': [-0.393, 3.14159265359, -4.5, 4.5, -0.0, 0.0]
            },
            {
                'joint_name': 'joint_4',
                'joint_limit': [-2.967, 2.967, -4.5, 4.5, -0.0, 0.0]
            },
            {
                'joint_name': 'joint_5',
                'joint_limit': [-1.6, 1.6, -4.5, 4.5, -0.0, 0.0]
            },
            {
                'joint_name': 'joint_6',
                'joint_limit': [-1.57, 1.57, -4.5, 4.5, -0.0, 0.0]
            }
        ]
    }

    success = arm_config_manager.reload_from_dict(0x0E, config_dict2)
    print(f"再次重载结果: {'成功' if success else '失败'}")

    # 验证重载结果
    final_config = get_arm_config(0x0E)
    if final_config:
        print(f"最终配置: {final_config.name}")
        print(f"第一个关节限制: {final_config.robot_config.joints[0].joint_limit}")
        print(
            f"验证结果: {'通过' if final_config.name == 'saber_d6x_modified' else '失败'}"
        )
    print()


def example_arm_archer_integration():
    """ArmArcher 集成示例"""
    print("=== ArmArcher 集成示例 ===")

    try:
        from .arm_archer import ArmArcher

        # 创建 ArmArcher 实例
        arm = ArmArcher(
            motor_count=6,
            arm_series=0x0E,  # saber_d6x
            name="TestArm")

        print(f"机械臂名称: {arm.get_arm_name()}")
        print(f"机械臂系列: 0x{arm.get_arm_series():02X}")
        print(f"期望电机数量: {arm.get_expected_motor_count()}")
        print(f"电机数量匹配: {arm.check_motor_count_match()}")
        print(f"关节名称: {arm.get_joint_names()}")

        # 测试关节限制验证
        test_positions = [5.0, 0.0, 1.57, 0.0, 0.0, 0.0]  # 第一个关节超出范围

        print(f"原始位置: {test_positions}")

        # 设置初始位置
        initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        arm.set_initial_positions(initial_positions)
        print(f"设置初始位置: {initial_positions}")

        validated_positions = arm.validate_joint_positions(test_positions)
        print(f"验证后位置: {validated_positions}")

        # 检查结果是否与直接调用配置管理器一致
        direct_result = arm_config_manager.validate_joint_positions(
            0x0E, test_positions)
        consistent = all(
            abs(a - b) < 0.0001
            for a, b in zip(validated_positions, direct_result))
        print(f"与直接调用结果一致: {'是' if consistent else '否'}")

        # 测试连续位置验证（速度限制）
        print("\n测试连续位置验证（速度限制）:")
        second_positions = [0.5, 0.1, 0.2, 0.0, 0.0, 0.0]
        print(f"第二次位置: {second_positions}")

        validated_second = arm.validate_joint_positions(second_positions)
        print(f"验证后第二次位置: {validated_second}")

        # 获取位置历史
        last_positions = arm.get_last_positions()
        print(f"位置历史记录: {last_positions}")

        # 测试重载配置
        print("测试重载配置:")
        config_dict = {
            'name':
            'test_arm_reloaded',
            'dof_num':
            'six_axis',
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

        success = arm.reload_arm_config_from_dict(config_dict)
        print(f"重载结果: {'成功' if success else '失败'}")

        # 验证重载后的配置
        reloaded_config = arm.get_arm_config()
        if reloaded_config:
            print(f"重载后配置名称: {reloaded_config.name}")
            print(
                f"重载后第一个关节限制: {reloaded_config.robot_config.joints[0].joint_limit}"
            )

    except ImportError as e:
        print(f"无法导入 ArmArcher: {e}")
    print()


def run_all_examples():
    """运行所有示例"""
    print("机械臂配置系统使用示例")
    print("=" * 50)

    example_basic_usage()
    example_joint_validation()
    example_add_custom_config()
    example_reload_config()
    example_arm_archer_integration()

    print("所有示例运行完成！")
    print("\n这个配置系统提供了类似于 Rust 代码的功能:")
    print("- 预定义的机械臂配置")
    print("- 根据机械臂系列获取配置")
    print("- 智能关节限制验证（返回修正后的位置列表）")
    print("- 基于位置历史的自动速度限制检查和位置插值")
    print("- 动态添加新配置")
    print("- 从字典运行时重载配置参数")
    print("- 在 ArmArcher 类中集成使用")


if __name__ == "__main__":
    run_all_examples()
