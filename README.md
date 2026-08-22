# 🛠️ Cooperation landing package
This package is for heterogeneous robotic system with aerial and ground robots. 
## ⚙️ **Setup**

### 🖥️ Supported Environments
- **Ubuntu 20.04**, **Ubuntu 22.04**
- **ROS1** (Noetic)
- **ROSO** (one)

### 📦 **Build**
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash # please replace ${ROS_DISTRO} with your specific env variable, e.g., noetic
sudo apt install -y python3-wstool python3-catkin-tools
mkdir -p ~/ros/cooperation_landing_ros_ws/src
cd ~/ros/cooperation_landing_ros_ws
rosdep update
wstool init src
git clone git@github.com:Liyunong20000/cooperation_landing.git src/cooperation_landing
git -C src/cooperation_landing fetch origin
git -C src/cooperation_landing switch -t origin/develop/xuanwu_bricks
wstool merge -t src src/cooperation_landing/${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```
 **Full documentation is available in the [Wiki](https://github.com/Liyunong20000/cooperation_landing/wiki).**

## Spot + Kortex

The combined model mounts a Kinova Gen3 7-DOF arm and an optional Robotiq
2F-85 gripper at the center of Spot's upper body surface. Kortex links and
joints use the `arm_` prefix. The stock Spot model's centered Hokuyo/ASUS
accessories are omitted to leave the mounting area clear; the Gen3 Vision
RGB/depth camera and Spot IMU remain available.

Display the model and manipulate its joints with the RViz joint-state GUI:

```bash
source devel/setup.bash
roslaunch cooperation_landing spot_kortex_rviz.launch
```

Run the combined CHAMP/Gazebo simulation:

```bash
source devel/setup.bash
roslaunch cooperation_landing spot_kortex_gazebo.launch
```

Gazebo starts paused at a default 0.20 m foot clearance, applies the standing
and arm pose, starts the Spot, arm, and gripper controllers in zero gravity,
then restores gravity to begin the controlled drop test.
Disable this behavior for debugging with `auto_unpause:=false`. The mount pose
can be adjusted without editing the Xacro, for example:

```bash
roslaunch cooperation_landing spot_kortex_rviz.launch \
  mount_xyz:='0 0 0.105' mount_rpy:='0 0 0'
```

Controller command topics include:

```text
/joint_group_position_controller/command
/arm_gen3_joint_trajectory_controller/command
/arm_robotiq_2f_85_gripper_controller/gripper_cmd
```
