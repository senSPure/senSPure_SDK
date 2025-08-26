import os
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tof_param = os.path.join(
        get_package_share_directory('tof_camera_param'),
        'param',
        'tof_param.yaml'
    )
    postfilter_param = os.path.join(
        get_package_share_directory('tof_camera_param'),
        'param',
        'postfilter_param.yaml'
    )
    lens_param = os.path.join(
        get_package_share_directory('tof_camera_param'),
        'param',
        'lensconv_param.yaml'
    )
    rviz_param = os.path.join(
        get_package_share_directory('tof_camera_param'),
        'param',
        'rviz2_setting.rviz'
    )

    container = ComposableNodeContainer(
            name='tof_camera_node',
            namespace='tof_camera_node',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='tof_camera_node',
                    plugin='tof_camera_node::RecordNode',
                    name='Record',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='tof_camera_node',
                    plugin='tof_camera_node::StdMsgNode',
                    name='StdMsg',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='tof_camera_node',
                    plugin='tof_camera_node::LensConvNode',
                    name='LensConv',
                    parameters=[lens_param],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='tof_camera_node',
                    plugin='tof_camera_node::PostFilterNode',
                    name='PostFilter',
                    parameters=[postfilter_param],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='tof_camera_node',
                    plugin='tof_camera_node::TofCtrlNode',
                    name='TofCtrl',
                    parameters=[tof_param],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output='both',
            arguments=['--ros-args', '--log-level', 'error']
    )

    rviz = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=['-d', rviz_param]
    )

    return launch.LaunchDescription([rviz, container])
