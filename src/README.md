# URDF-Demo
Demo of URDF, Gazebo, RVIZ and Keyboard Teleop Application


-- ~/.bashrc file requirements to source

source /opt/ros/jazzy/setup.bash


source ~/ros2_ws/install/setup.bash


export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"


export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/my_robot_bringup/models:$GZ_SIM_RESOURCE_PATH




-- Launch Gazebo and URDF

ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 


-- Launch Keyboard Teleoperation Package

ros2 run teleop_twist_keyboard teleop_twist_keyboard


Queries for tutorials:
how to publish a message to JOintStatePublisher ros2 jazzy gz 13.5.0
how to publish velocity to joint ros2
how to use jointstate publisher python ros2 jazzy (watch videos)
how to control specific joints velocity ros2 jazzy python teleop script
how to publish a velocity to a joint using joint state publisher ros2 python

Tutorials to reference:
https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
https://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher
https://answers.ros.org/question/335691/
https://robotics.stackexchange.com/questions/112425/ros2-control-joint-state-broadcaster-publish-wrong-joint-velocity
https://docs.ros.org/en/jazzy/p/joint_state_publisher/
https://gazebosim.org/api/sim/9/jointcontrollers.html


Launch script for testing world and potato python script

ros2 launch my_robot_bringup two_robots_simple_test_xacro.launch.py
ros2 run robot_legion_teleop_python potato_legion_teleop

Troubleshooting:
ros2 topic echo /arm_base_joint/cmd_vel < checks that ros is recieving outputs
gz topic -e -t /arm_base_joint/cmd_vel < checks that gazebo is recieving outputs
