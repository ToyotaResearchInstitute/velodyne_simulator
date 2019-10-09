import os

from ament_index_python.packages import get_package_share_directory
import ament_index_python.packages
import launch
import launch_ros.actions


def read_file(path):
    with open(path, 'r') as f:
        contents = f.read()
    return contents

def generate_launch_description():
    if os.environ['ROS_DISTRO'] == 'crystal':
        output_mode = 'screen'
    else:
        output_mode = 'both'

    gazebo_dir = os.path.dirname(get_package_share_directory('velodyne_description'))
    world = os.path.join(get_package_share_directory('velodyne_description'),
        'world', 'example.world')

    # NOT USING xacro until it gets ported to ROS2
    robot_urdf = os.path.join(
        get_package_share_directory('velodyne_description'), 'urdf', 'example.urdf')
    urdf_contents = read_file(robot_urdf)


    rviz_config = os.path.join(
        ament_index_python.packages.get_package_share_directory('velodyne_description'),
        'rviz', 'example.rviz')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                              node_executable='robot_state_publisher',
                              output='both',
                              arguments=[robot_urdf])

    rviz = launch_ros.actions.Node(package='rviz2',
                              node_executable='rviz2',
                              output='both',
                              arguments=['-d', rviz_config])


    print("rviz_config : " + rviz_config)

    spawn_entity_message_contents = "'{initial_pose:{ position: {x: 0, y: 0, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}},  name: \"velodyne_description\", xml: \"" + urdf_contents.replace('"', '\\"') + "\"}'"
    spawn_entity = launch.actions.ExecuteProcess(
        name='spawn_entity', cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_entity_message_contents], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)

    # PUBLSHIES THE URDF TO /robot_description.
    # This is a workasround to make rviz get the urdf.
    urdf_pub_data = urdf_contents.replace('"', '\\"')
    launch_urdf = launch.actions.ExecuteProcess(
        name='launch_urdf', cmd=['ros2', 'topic', 'pub', '-r', '0.1', '/robot_description', 'std_msgs/String', '\'data: "' + urdf_pub_data + '"\''], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)


    my_env = os.environ.copy()
    my_env["GAZEBO_MODEL_PATH"] = gazebo_dir
    gazebo = launch.actions.ExecuteProcess(
        name='gazebo', cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world], env=my_env, output=output_mode, shell=True, log_cmd=False)

    return launch.LaunchDescription([rsp,
                                     rviz,
                                     gazebo,
                                     spawn_entity,
                                     launch_urdf,
                                     launch.actions.RegisterEventHandler(
                                      event_handler=launch.event_handlers.OnProcessExit(
                                         target_action=gazebo,
                                         on_exit=[launch.actions.EmitEvent(
                                             event=launch.events.Shutdown())],
                                      )),
                                   ])
