import os

import ament_index_python.packages
import launch
import launch_ros.actions

    world = get_package_share_directory('velodyne_description'),
    'world', 'example.world')

    robot_urdf = os.path.join(
        get_package_share_directory('velodyne_description'), 'urdf', 'example.urdf')
    urdf_contents = read_file(robot_urdf)


    # rviz_config = os.path.join(
    #     ament_index_python.packages.get_package_share_directory('bulba_description'),
    #     'config', 'bulba.rviz')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  node_executable='robot_state_publisher',
                                  output='both',
                                  arguments=[robot_urdf])

    rviz = launch_ros.actions.Node(package='rviz2',
                                  node_executable='rviz2',
                                  output='both',
                                  )#arguments=['--display-config', rviz_config])



    spawn_entity_message_contents = "'{initial_pose:{ position: {x: 0, y: 0, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}},  name: \"example\", xml: \"" + urdf_contents.replace('"', '\\"') + "\"}'"
    spawn_entity = launch.actions.ExecuteProcess(
        name='spawn_entity', cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_entity_message_contents], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)

    my_env = os.environ.copy()
    my_env["GAZEBO_MODEL_PATH"] = gazebo_dir
    gazebo = launch.actions.ExecuteProcess(
        name='gazebo', cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world], env=my_env, output=output_mode, shell=True, log_cmd=False)


    return launch.LaunchDescription([rsp,
                                     rviz,
                                     gazebo,
                                     spawn_enity,
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
