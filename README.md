source /opt/ros/${ROS_DISTRO}/setup.bash # please replace ${ROS_DISTRO} with your specific env variable, e.g., noetic
mkdir -p ~/ros/cooperation_landing_ros_ws/src
cd ~/ros/cooperation_landing_ros_ws
rosdep update
wstool init src
git clone git@github.com:Liyunong20000/cooperation_landing.git
catkin build
