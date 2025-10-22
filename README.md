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
