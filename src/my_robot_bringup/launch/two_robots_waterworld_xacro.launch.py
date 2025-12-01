# ~/ros2_ws/src/my_robot_bringup/launch/two_robots_waterworld_xacro.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch Gazebo (gz-sim) with the waterworld arena and spawn two robots:

      - emiliobot  (from emiliobot.urdf.xacro)
      - my_robot   (from my_robot.urdf.xacro)

    Both robots are driven directly from their XACRO files using the xacro
    executable at runtime. ros_gz_bridge is started with a YAML config and
    RViz2 is launched with a URDF visualization config.
    """

    # ------------------------------------------------------------------
    # Package locations
    # ------------------------------------------------------------------
    bringup_pkg = FindPackageShare("my_robot_bringup")
    description_pkg = FindPackageShare("my_robot_description")

    # World file: ~/ros2_ws/src/my_robot_bringup/worlds/waterworld.sdf
    world_path = PathJoinSubstitution(
        [bringup_pkg, "worlds", "waterworld.sdf"]
    )

    # Bridge config: ~/ros2_ws/src/my_robot_bringup/config/gazebo_bridge.yaml
    gazebo_config = PathJoinSubstitution(
        [bringup_pkg, "config", "gazebo_bridge.yaml"]
    )

    # RViz config: ~/ros2_ws/src/my_robot_description/rviz/urdf_config.rviz
    rviz_config = PathJoinSubstitution(
        [description_pkg, "rviz", "urdf_config.rviz"]
    )

    # Xacro executable
    xacro_exe = FindExecutable(name="xacro")

    # Xacro files for each robot
    #   ~/ros2_ws/src/my_robot_description/urdf/emiliobot.urdf.xacro
    #   ~/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
    emiliobot_xacro = PathJoinSubstitution(
        [description_pkg, "urdf", "emiliobot.urdf.xacro"]
    )
    my_robot_xacro = PathJoinSubstitution(
        [description_pkg, "urdf", "my_robot.urdf.xacro"]
    )

    # Run xacro at launch time to produce URDF XML strings
    emiliobot_description = Command([xacro_exe, " ", emiliobot_xacro])
    my_robot_description = Command([xacro_exe, " ", my_robot_xacro])

    # ------------------------------------------------------------------
    # Robot State Publishers (one per robot)
    # ------------------------------------------------------------------

    # Publishes /emiliobot/** TF and robot_description from emiliobot.xacro
    emiliobot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="emiliobot",
        name="emiliobot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": emiliobot_description,
             "use_sim_time": True},
        ],
    )

    # Publishes /my_robot/** TF and robot_description from my_robot.xacro
    my_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="my_robot",
        name="my_robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": my_robot_description,
             "use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # Gazebo (gz-sim) with the waterworld arena
    # ------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": world_path}.items(),
    )

    # ------------------------------------------------------------------
    # Spawn both robots directly from their URDF strings
    # ------------------------------------------------------------------

    # Spawn emiliobot into Gazebo using its URDF string
    spawn_emiliobot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_emiliobot",
        output="screen",
        arguments=[
            "-name", "emiliobot",
            "-x", "0.0", "-y", "0.0", "-z", "5.5",
            "-Y", "0.0",
            "-string", emiliobot_description,
        ],
    )

    # Spawn my_robot into Gazebo using its URDF string
    spawn_my_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_my_robot",
        output="screen",
        arguments=[
            "-name", "my_robot",
            "-x", "2.0", "-y", "0.0", "-z", "5.5",
            "-Y", "1.57",
            "-string", my_robot_description,
        ],
    )

    # ------------------------------------------------------------------
    # ROS ↔ Gazebo bridge using gazebo_bridge.yaml
    #   ~/ros2_ws/src/my_robot_bringup/config/gazebo_bridge.yaml
    # ------------------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": gazebo_config}],
    )

    # ------------------------------------------------------------------
    # RViz2 with URDF visualization
    # ------------------------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # ------------------------------------------------------------------
    # Return the full launch description
    # ------------------------------------------------------------------
    return LaunchDescription([
        gz_sim,
        emiliobot_state_publisher,
        my_robot_state_publisher,
        spawn_emiliobot,
        spawn_my_robot,
        bridge,
        rviz,
    ])
