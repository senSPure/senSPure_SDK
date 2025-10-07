import os
import launch
from launch_ros.actions import ComposableNodeContainer
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
    viewer_config = os.path.join(
        get_package_share_directory('tof_camera_example'),
        'param'
    )
    font_path = os.path.join(
        get_package_share_directory('tof_camera_example'),
        '3rd_pty'
    )

    container = ComposableNodeContainer(
            name='tof_camera_node',
            namespace='tof_camera_node',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='tof_camera_node',
                    plugin='tof_camera_node::RecordNode',
                    name='Record',
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
                    parameters=[tof_param, {'open_dev': False}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='tof_camera_example',
                    plugin='tof_camera_example::SampleViewerNode',
                    name='SampleViewer',
                    parameters=[{'config_path': viewer_config}, {'font_path': font_path}],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output='both',
            arguments=['--ros-args', '--log-level', 'error']
    )

    return launch.LaunchDescription([container])
