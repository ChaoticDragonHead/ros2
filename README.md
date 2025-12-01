# SETUP INSTRUCTIONS

**Note:** If your **workspace directory is not called ros2_ws**, then **replace ros2_ws** with **the name of your workspace directory** throughout these code segments and commands. 

**Note:** A file called **_waterworld.sdf_** is referenced throughout this document.  If the world that you are trying to open is not named **_waterworld.sdf_**, then replace **_waterworld.sdf_** with **the name of your world file** when entering these commands.

git clone https://github.com/AEmilioDiStefano/ros2_ws.git


## 1.  Create a GZ_SIM_SYSTEM_PLUGIN_PATH system variable

### This will enable the use of our physics plugins.

Open the .bashrc file again by typing the following into your terminal from your home directory:

```shell
sudo gedit ~/.bashrc
```

Now paste the following at the end of the .bashrc file:
```shell
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

Save changes to the .bashrc file and close the editor.


## 2. Create a GZ_SIM_RESOURCE_PATH system variable

### This will allow you to reference models in your /models directory via a <uri></uri> in our SDF world files.  


Open your .bashrc file by typing the following into your terminal from your home directory:

```shell
sudo gedit ~/.bashrc
```

Now paste the following at the end of the .bashrc file:
```shell
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/my_robot_bringup/models:$GZ_SIM_RESOURCE_PATH
```

Save changes to the .bashrc file and close the editor.

Now add the code depicted below:

```shell
<include>
  <name>water_tower</name>
  <uri>model://water_tower/model.sdf</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

in your SDF world file will load the water_towel model from the /models directory into your world.

## 3. Make sure that the following lines are included in your .bashrc file:

```shell
source /opt/ros/jazzy/setup.bash
```

```shell
source ~/ros2_ws/install/setup.bash
```

```shell
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

```shell
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/my_robot_bringup/models:$GZ_SIM_RESOURCE_PATH
```





# Important Commands

## Reload the Shell:

```shell
source ~/.bashrc
```

## Reload the Shell:

```shell
source ~/.bashrc
```

## Build and Execute Your Environment:
### 1. Make sure you are in the correct folder (ros2_ws):
```shell
cd ~/ros2_ws
```
### 2. Build
```shell
colcon build
```
### 3. Execute
```shell
source ~/ros2_ws/install/setup.bash
```



## Open RViz to show a visialization of your robot:
### 1. Colcon Build and source to make sure any changes are reflected accurately: 
```shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### 2. Open RViz with a visualization of your robot
```shell
ros2 launch urdf_tutorial display.launch.py model:=/home/aemilio/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
```
**Note**: This command will open a XACRO file called my_robot.urdf.xacro which sould already be in your repository.



## Open a Gazebo world with your robot within:
### 1. Colcon Build and source to make sure any changes are reflected accurately: 
```shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### 2. Run the launch file that opens our Gazebo world and spawns our robot within:
```shell
ros2 launch my_robot_bringup my_robot.launch.xml
```
**Note**: This command will run a **launch file** called my_robot.urdf.xacro which sould already be in your repository.

### 3. Run a control node to control a robot:
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Note:** In order to conmtrol the robot, **the terminal window running the teleop control node must be selected** rather than the Gazebo simulation window.



## Open a Gazebo world with the my_robot.urdf.xacro and another robot within (2 robots total):
### 1. Colcon Build and source to make sure any changes are reflected accurately: 
```shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### 2. Run our Python launch script that opens our Gazebo combat arena while spawning two robots within: one called my_robot and another called emiliobot: 
```shell
ros2 launch my_robot_bringup two_robots_waterworld_xacro.launch.py
```
**Note**: This command runs a **launch file** that opens a world called **waterworld.sdf** with **both** the robot defined in **my_robot.urdf.xacro** and the robot defined in **emiliobot.urdf.xacro**.

### 3. Run a control node to control a robot:
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r cmd_vel:=/emiliobot/cmd_vel
```
**Note:** The above command will run a teleop node t control a robot called **emiliobit**.  In order to **run the teleop node to control another robot**, replace the name **emiliobot** in the above command with the name of your robot of chooice (for example, **my_robot**).





# Common Issues

## If Gazebo doesn't open when attempting to open a world from my_robot_gazebo.launch.xml:

### 1. Try opening your world from ROS2 in only Gazebo.

```shell
gz sim "$(ros2 pkg prefix my_robot_bringup)/share/my_robot_bringup/worlds/waterworld.sdf" -v4
```
### 2. If it doesn't open, then build and source your environment again:
**Build:**
```shell
colcon build
```
**Execute:**
```shell
source ~/ros2_ws/install/setup.bash
```
### 3. Now try launching again:
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
**Note:** A file called **_waterworld.sdf_** is referenced in the above commands.  If the world that you are trying to open is not named **_waterworld.sdf_**, then replace **_waterworld.sdf_** with **the name of youe world file** when entering these commands.



## If Gazebo opens but does not load your robot:

### 1. Open CMakeLists.txt in ~/ros2_ws/src/

**Make sure** that under the **__install (__** line of the **__CMakeLists.txt__** document, you see **the names of each directory** in the **__my_robot_descrioption__** directory.  If each directory name is not listed, **add it.** 



## If opening your XACRO file in RViz results in the following error or something similar:
```console
[INFO] [launch]: All log files can be found below /home/aemilio/.ros/log/2025-11-30-11-20-40-852518-mr-roboto-13290 [INFO] [launch]: Default logging verbosity is set to INFO [ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'joint_state_publisher_gui' not found, searching: ['/home/aemilio/ros2_ws/install/urdf_tutorial', '/home/aemilio/ros2_ws/install/urdf_launch', '/home/aemilio/ros2_ws/install/my_robot_description', '/home/aemilio/ros2_ws/install/my_robot_bringup', '/opt/ros/jazzy']"

```
This means that you have a **missing dependancy** (**'joint_state_publisher_gui'** in this example) which needs to be installed.  

### 1. Try installing the package with apt:
```shell
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui
```

### 2. Open a new terminal and source your setup files:
```shell
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 3. Enter your urdf directory:
**From your home directory** enter:
```shell
cd ros2_ws/src/my_robot_description/urdf
```

### 4. Try opening your XACRO file in RViz
```shell
ros2 launch urdf_tutorial display.launch.py 

model:=/home/aemilio/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
```

## If adding missing dependencies results in the following error or something similar:
```console
E: Unable to locate package ros-jazzy-joint-state-publisher-gui
```
This means that your **ROS2 apt repository** is **not configured**.

### 1. Install the software-properties-common package and the universe repository:
```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 2. Install ros2-apt-source helper to sets up the ROS 2 apt repo automatically
```shell
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 3. Refresh your apt indexes:
```shell
sudo apt update
```

### 4. Install the missing package:
(**ros-jazzy-joint-state-publisher-gui** in this example, replace with the appropriate package name if another package is missing frm your workspace)
```shell
sudo apt install ros-jazzy-joint-state-publisher-gui
```

### 5. Install the non-GUI publisher to be thorough:
```shell
sudo apt install ros-jazzy-joint-state-publisher
```

### 6. Open a new terminal and source your setup files:
```shell
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 7. Enter your urdf directory:
**From your home directory** enter:
```shell
cd ros2_ws/src/my_robot_description/urdf
```

### 8. You should now be able to open your XACRO file in RViz
```shell
ros2 launch urdf_tutorial display.launch.py 

model:=/home/aemilio/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
```





## Sources:

All downloaded objects were created by OpenRobotics and downloaded from <a href="https://app.gazebosim.org/OpenRobotics" target="_blank">https://app.gazebosim.org/OpenRobotics</a>
