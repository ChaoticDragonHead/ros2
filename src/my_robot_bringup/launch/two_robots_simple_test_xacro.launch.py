# ~/ros2_ws/src/my_robot_bringup/launch/two_robots_simple_test_xacro.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """
    Launch Gazebo (gz-sim) with the simple_test arena and spawn two robots:

      - emiliobot  (from emiliobot.urdf.xacro)
      - potatobot   (from potatobot.urdf.xacro)

    Both robots are driven directly from their XACRO files using the xacro
    executable at runtime. ros_gz_bridge is started with a YAML config and
    RViz2 is launched with a URDF visualization config.
    """

    # ------------------------------------------------------------------
    # Package locations
    # ------------------------------------------------------------------
    bringup_pkg = FindPackageShare("my_robot_bringup")
    description_pkg = FindPackageShare("my_robot_description")

    # World file: ~/ros2_ws/src/my_robot_bringup/worlds/simple_test.sdf
    world_path = PathJoinSubstitution(
        [bringup_pkg, "worlds", "simple_test.sdf"]
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
    #   ~/ros2_ws/src/my_robot_description/urdf/potatobot.urdf.xacro
    emiliobot_xacro = PathJoinSubstitution(
        [description_pkg, "urdf", "emiliobot.urdf.xacro"]
    )
    potatobot_xacro = PathJoinSubstitution(
        [description_pkg, "urdf", "potatobot.urdf.xacro"]
    )

    # Run xacro at launch time to produce URDF XML strings
    emiliobot_description_cmd = Command([xacro_exe, " ", emiliobot_xacro])
    potatobot_description_cmd = Command([xacro_exe, " ", potatobot_xacro])

    # ------------------------------------------------------------------
    # Robot State Publishers (one per robot)
    # ------------------------------------------------------------------

    # Publishes /emiliobot/** TF and robot_description from emiliobot.xacro
    #In Jazzy we must wraap Command(...) in ParemeterValue(value_type=str) so robot_description is treated as a plain string, not YAML
    emiliobot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="emiliobot",
        name="emiliobot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(emiliobot_description_cmd, value_type=str),
             "use_sim_time": True},
        ],
    )

    # Publishes /potatobot/** TF and robot_description from potatobot.xacro
    potatobot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="potatobot",
        name="potatobot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(potatobot_description_cmd, value_type = str),
             "use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # Gazebo (gz-sim) with the simple_test arena
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
            "-x", "0.0", 
            "-y", "0.0", 
            "-z", "5.5",
            "-Y", "0.0",
            "-string", emiliobot_description_cmd,
        ],
    )

    # Spawn potatobot into Gazebo using its URDF string
    spawn_potatobot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_potatobot",
        output="screen",
        arguments=[
            "-name", "potatobot",
            "-x", "2.0", 
            "-y", "0.0", 
            "-z", "5.5",
            "-Y", "1.57",
            "-string", potatobot_description_cmd,
        ],
    )

    # ------------------------------------------------------------------
    # ROS ↔ Gazebo bridge using gazebo_bridge.yaml
    #   ~/ros2_ws/src/potatobot_bringup/config/gazebo_bridge.yaml
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
        potatobot_state_publisher,
        spawn_emiliobot,
        spawn_potatobot,
        bridge,
        rviz,
    ])
