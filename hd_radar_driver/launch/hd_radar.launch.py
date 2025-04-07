#!/usr/bin/env python3

import os
import yaml
import argparse

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

SENSOR_MODEL = 'hd_radar'
PACKAGE_NAME = f'{SENSOR_MODEL}_driver'

def str2bool(arg):
    return arg.lower() in ('yes', 'true', 'True', 't', '1')

def generate_nodes(context, *args, **kwargs):
    # Read launch args
    rviz_arg = str2bool(LaunchConfiguration('rviz').perform(context))
    rqt_arg = str2bool(LaunchConfiguration('rqt').perform(context))

    print('rviz_arg:', rviz_arg)
    print('rqt_arg:', rqt_arg)

    nodes = []

    # Setup config paths
    hd_radar_config_path = os.path.join(
    get_package_share_directory(PACKAGE_NAME),'config/hd_radar_config.yaml'
    )
    rviz_config_path = os.path.join(
    get_package_share_directory(PACKAGE_NAME),'rviz/hd_radar_single.rviz'
    )
    rqt_config_path = os.path.join(
    get_package_share_directory(PACKAGE_NAME),'rqt/hd_radar_service_caller.perspective'
    )

    # Read YAML file
    with open(hd_radar_config_path, 'r') as stream:
        data = yaml.safe_load(stream)

    sens_num = len(data['sensors'].keys())

    print("Sensors total: ", sens_num)

    message = LogInfo(msg=f'Starting {sens_num} {SENSOR_MODEL} nodes.')

    for i in range(0, sens_num):
        # Read params for curent sensor
        params = data['sensors'][f'sensor_{i}']
        params.update({'topic': f'{SENSOR_MODEL}_{i}'})
        params.update({'frame_id': f'{SENSOR_MODEL}_{i}'})
        print(params)
        # Launch node for current sensor
        nodes.append(
            Node(
                package = PACKAGE_NAME, 
                executable = f'{SENSOR_MODEL}_node',
                name = f'{SENSOR_MODEL}_{i}',
                parameters=[params]
            )
        )
    # Launch rviz node if arg was setted
    if (rviz_arg):
        nodes.append(
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config_path)]
            )
        )
    # Launch rqt node if arg was setted
    if (rqt_arg):
        nodes.append(
        Node(
            package='rqt_gui',
            namespace='',
            executable='rqt_gui',
            name='rqt_gui',
            arguments=['--perspective-file', str(rqt_config_path)]
            )
        )
    return nodes + [message]

def generate_launch_description():
    declared_args = []
    declared_args.append(
    	DeclareLaunchArgument(
    	'rviz',
    	default_value='False',
    	)
    )
    declared_args.append(
    	DeclareLaunchArgument(
    	'rqt',
    	default_value='False',
    	)
    )
    return LaunchDescription(
	    declared_args +
	    [
		OpaqueFunction(function=generate_nodes)
    ])
