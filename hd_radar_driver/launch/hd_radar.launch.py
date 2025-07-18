#!/usr/bin/env python3

import os
import yaml
import numpy as np

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
    rqt_config_path = os.path.join(
    get_package_share_directory(PACKAGE_NAME),'rqt/hd_radar_service_caller.perspective'
    )

    # Read YAML file
    with open(hd_radar_config_path, 'r') as stream:
        data = yaml.safe_load(stream)

    sens_num = len(data['sensors'].keys())

    # Setup rviz config path
    rviz_config_path = os.path.join(
    get_package_share_directory(PACKAGE_NAME),
    'rviz/hd_radar_multiple.rviz' if sens_num >= 2 else 'rviz/hd_radar_single.rviz'
    )

    print("Sensors total: ", sens_num)

    message = LogInfo(msg=f'Starting {sens_num} {SENSOR_MODEL} nodes.')

    for i in range(0, sens_num):

        # Read params for curent sensor
        sensor_key = f'sensor_{i}'
        sensor_data = data['sensors'][sensor_key]
        params = sensor_data
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

        # Check if transform parameters exist
        if 'transform' not in sensor_data:
            print(f"Warning: No transform parameters for {sensor_key}, using defaults")
            tf_params = {
                'parent_frame': 'base_link',
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
            }
        else:
            tf_params = sensor_data['transform']

        # Create TF publisher node
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{sensor_key}_tf_publisher',
                arguments=[
                    str(tf_params['x']), str(tf_params['y']), str(tf_params['z']),
                    str(np.deg2rad(tf_params['yaw'])),
                    str(np.deg2rad(tf_params['pitch'])),
                    str(np.deg2rad(tf_params['roll'])),
                    tf_params['parent_frame'], f'{SENSOR_MODEL}_{i}'
                ]
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
