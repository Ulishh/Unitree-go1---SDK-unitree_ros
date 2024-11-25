from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from glob import glob
# from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Incluir el archivo de lanzamiento de unitree go1
    unitree_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('unitree_ros'), 'launch', 'unitree_driver_launch.py']
            )
        ),
        launch_arguments={'wifi': 'true'}.items()
    )

    # Incluir el archivo de lanzamiento de lidar a1
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rplidar_ros'), 'launch', 'rplidar_a1_launch.py']
            )
        )
    )
    
    # Incluir el archivo de lanzamiento de slam_toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
            )
        ),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'config', 'mapper_params_online_async.yaml']
            ),
            'use_sim_time': 'false'
        }.items()
    )

    # # Incluir el archivo de lanzamiento de Nav2 -------> Se debe correr en otra terminal para que genere el costmap
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    #         )
    #     )
    # )

    # Nodo para transformacion estatica de base_link a laser
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser',
        arguments=[
            '0', '0', '0',  # Traslación (x, y, z)
            '0', '0', '0', '1',  # Rotación (cuaternión: x, y, z, w)
            'base_link',  # Frame padre
            'laser'  # Frame hijo
        ]
    )

    # Nodo para transformacion estatica de base_link a base_footprint
    tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint',
        arguments=[
            '0', '0', '0',  # Traslación (x, y, z)
            '0', '0', '0', '1',  # Rotación (cuaternión: x, y, z, w)
            'base_link',  # Frame padre
            'base_footprint'  # Frame hijo
        ]
    )

    # # Abrir rviz2 con configuracion especifica
    # rviz_config_path = PathJoinSubstitution(
    #     [FindPackageShare('unitree_slam'), 'rviz', 'default.rviz']
    # )
    # rviz2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path]
    # )

    return LaunchDescription([
        unitree_launch,
        rplidar_launch,
        slam_launch,
        tf_laser,
        tf_base_footprint
    ])


