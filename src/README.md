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


IN ORDER TO GET FUNCTIONALITY TO JOINTS BESIDES THE REGULAR MOVEMENT SCRIPT (contained in potato_teleop_legion_key):
- Make sure that under the mobile_base code you have a joint state controller block (see mobile_base_potatobot_gazebp.xacro):
    <!-- <gazebo>
        <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
            <joint_name>arm_base_joint</joint_name>
            <initial_velocity>0</initial_velocity>
            <topic>/arm_base_joint/cmd_vel</topic>
        </plugin>
    </gazebo> --> < commented to avoid code issues in readme>
- Make sure that you define the proper bridge in the gazebo_bridge.yaml file to ensure that ros2 and gazebo can communicate (critical for simulation):
# -------------------------
# POTATOBOT bridges
# -------------------------

- ros_topic_name: "/potatobot/joint_states"
  gz_topic_name: "/world/simple_test/model/my_robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/potatobot/tf"
  gz_topic_name: "/model/potatobot/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/potatobot/cmd_vel"
  gz_topic_name: "/model/potatobot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/arm_base_joint/cmd_vel"
  gz_topic_name: "/arm_base_joint/cmd_vel"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- In the controller code:
    from std_msgs.msg import Float64 < this imports the proper library for publishing messages

        self.armBaseJointPublisher_ = self.create_publisher(Float64, '/arm_base_joint/cmd_vel', 10) < creates publisher to send messages to robot joints, specifically the arm base joint

        self.velocity_ = 1.0 <  velocity must be a float


        def _pusher_left(self):
        #Set velocity to -.1 unless joint position is greater than .25
        print("Pusher left triggered")
        msg = Float64()
        msg.data = self.velocity_

        print("Publishing to joint" + "\n ~~~ \n ~~~ \n Publishing velocity " + str(msg.data))
        self.armBaseJointPublisher_.publish(msg)
        print("Joint triggered")

    def _pusher_right(self):
        #Set velocity to -.1 unless joint position is greater than .25
        print("Pusher right triggered")
        msg = Float64()
        msg.data = -self.velocity_

        print("Publishing to joint " + "\n ~~~ \n ~~~ \n Publishing velocity " + str(msg.data))
        self.armBaseJointPublisher_.publish(msg)
        print("Joint triggered")
    
    def _pusher_center(self):
        #Set velocity to -.1 unless joint position is greater than .25
        print("Pusher center triggered")
        msg = Float64()
        msg.data = 0.0

        print("Publishing to joint " + "\n ~~~ \n ~~~ \n Publishing velocity " + str(msg.data))
        self.armBaseJointPublisher_.publish(msg)
        print("Joint triggered")

        ^^^^^^
        Joint movement functions


        Within main loop:
                        elif key in ('j', 'k', 'l'):
                    print("pusherarm triggered")
                    if key == 'j':
                        self._pusher_left()
                    elif key == 'l':
                        self._pusher_right()
                    elif key == 'k':
                        self._pusher_center()
                    else:
                        print("error - unexpected result. Key not in j, k, or l.")

