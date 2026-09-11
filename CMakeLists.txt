cmake_minimum_required(VERSION 3.0.2)
project(cooperation_landing)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  message_generation
  nav_msgs
  spinal
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  GR2UAV.msg
  UAV2GR.msg
)

generate_messages(
  DEPENDENCIES
  apriltag_ros
  geometry_msgs
  nav_msgs
  spinal
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  apriltag_ros
  geometry_msgs
  message_runtime
  nav_msgs
  spinal
  std_msgs
)

# Executables are thin wrappers; importable modules are installed by setup.py.
# Preserve deployed executable names while using Catkin's interpreter rewriting.
catkin_install_python(PROGRAMS
  scripts/AprillandQilin.py
  scripts/AprilmoveQilin.py
  scripts/basic_function.py
  scripts/dog_joystick.py
  scripts/dog_stall_monitor.py
  scripts/drone_basic_function.py
  scripts/event_trigger.py
  scripts/keyboard_control.py
  scripts/manipulation_motion_display.py
  scripts/manipulation_motion_v2.py
  scripts/send_igmp_report.py
  scripts/simulation_detection_collaborative_work.py
  scripts/gripper/gripper_keyboard.py
  scripts/gripper/gripper_move.py
  scripts/simulation/sim_basic.py
  scripts/simulation/sim_ground_robot_tf.py
  scripts/simulation/sim_links_attachment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS bin/install_igmp_report.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Standalone utilities also use the per-package executable directory.
catkin_install_python(PROGRAMS
  scripts/figures/code/Picture_generate.py
  scripts/figures/code/chessboard_generate.py
  scripts/figures/code/csv_generate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs gazebo_model launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES CHANGELOG.rst LICENSE README.md SECURITY.md
  noetic.rosinstall one.rosinstall
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
