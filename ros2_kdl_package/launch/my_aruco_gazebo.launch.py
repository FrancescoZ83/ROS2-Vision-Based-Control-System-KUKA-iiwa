from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for manual configuration
    use_sim = LaunchConfiguration('use_sim', default='true')
    command_interface = LaunchConfiguration('command_interface', default='velocity')
    robot_controller = LaunchConfiguration('robot_controller', default='velocity_controller')
    marker_size = LaunchConfiguration('marker_size', default='0.1')
    marker_id = LaunchConfiguration('marker_id', default='201')
    initial_positions_file = LaunchConfiguration('initial_positions_file', default='init_pos_vis_cont.yaml')

    # Path to the aruco_ros single.launch.py file
    aruco_launch_file = FindPackageShare('aruco_ros').find('aruco_ros') + '/launch/single.launch.py'

    # Path to the iiwa_bringup iiwa.launch.py file
    iiwa_bringup_launch_file = FindPackageShare('iiwa_bringup').find('iiwa_bringup') + '/launch/iiwa.launch.py'

    # Arguments for the iiwa launch file
    iiwa_launch_arguments = [
        ('use_sim', use_sim),
        ('command_interface', command_interface),
        ('robot_controller', robot_controller),
        ('use_vision', 'true'),
        ('initial_positions_file', initial_positions_file),
    ]

    # Include the iiwa_bringup launch file
    iiwa_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(iiwa_bringup_launch_file),
        launch_arguments=iiwa_launch_arguments
    )

    # Include the aruco_ros single.launch.py file with arguments
    aruco_single_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_launch_file),
        launch_arguments={
            'marker_size': marker_size,
            'marker_id': marker_id,
        }.items(),
    )

    # Static transform publisher Node
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '1.57', '3.14', '1.57', 'camera_link', 'stereo_gazebo_left_camera_optical_frame'],
        output='screen',
    )

    # rqt_image_view Node
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
    )

    return LaunchDescription([
        # Declare arguments so that they can be passed from the command line
        DeclareLaunchArgument('use_sim', default_value='true', description='Whether to use simulation'),
        DeclareLaunchArgument('command_interface', default_value='velocity', description='Command interface type'),
        DeclareLaunchArgument('robot_controller', default_value='velocity_controller', description='Name of the robot controller'),
        DeclareLaunchArgument('initial_positions_file', default_value='init_pos_vis_cont.yaml', description='Initial positions file'),

        iiwa_bringup_launch,
        aruco_single_launch,
        static_transform_publisher,
        rqt_image_view,
    ])

