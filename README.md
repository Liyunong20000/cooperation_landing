# cooperation_landing

`cooperation_landing` is a ROS 1 package for cooperative perception,
manipulation, and landing with a Xuanwu aerial robot and a Unitree Go1 ground
robot. It contains AprilTag-based alignment, SMACH task orchestration, network
message bridges, Gazebo assets, camera calibration, and teleoperation tools.

> [!WARNING]
> Several nodes publish velocity, flight, landing, and gripper commands to
> physical robots. Verify namespaces, frame conventions, safety limits, and
> emergency-stop behavior in simulation before enabling motion. The primary
> launch files keep autonomous motion opt-in.

## Supported environment

- Ubuntu 20.04 with ROS Noetic and Python 3.8
- Catkin Tools (`catkin build`)
- The laboratory ROS-O `one` environment where its matching dependencies are
  available

Ubuntu 22.04 is not listed as a validated ROS Noetic target. Containers or a
source-built ROS installation may work, but are not currently tested here.

## Repository layout

| Path | Purpose |
| --- | --- |
| `src/cooperation_landing/` | Importable control, perception, gripper and simulation modules |
| `scripts/` | Thin ROS executable entry points and standalone utilities |
| `setup.py` | Catkin Python package installation metadata |
| `launch/` | Hardware, network, camera, and simulation launch files |
| `config/` | Calibration, AprilTag, manipulation, and RViz configuration |
| `msg/` | Ground-to-air and air-to-ground bridge messages |
| `urdf/` | Robot, payload, landing-gear, and AprilTag descriptions |
| `gazebo_model/` | Gazebo worlds and AprilTag models |
| `docs/` | ROS interfaces, units and migration notes |

The current state machine is launched through `manipulation_motion_v2.py`.
The removed legacy `manipulation_motion.py` and `manipulation_state.py` are
no longer referenced by the build or launch files. Remove `use_v2` from older
launch commands. Python callers should import from `cooperation_landing.*`.

## Build

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install python3-catkin-tools python3-rosdep python3-wstool

mkdir -p ~/ros/cooperation_landing_ros_ws/src
cd ~/ros/cooperation_landing_ros_ws
wstool init src
git clone --branch develop/xuanwu_bricks \
  https://github.com/Liyunong20000/cooperation_landing.git \
  src/cooperation_landing
wstool merge -t src src/cooperation_landing/noetic.rosinstall
wstool update -t src

# Follow the jsk_aerial_robot setup instructions for its third-party sources,
# or source an existing aerial_robot underlay before building this workspace.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin build cooperation_landing
source devel/setup.bash
```

For ROS-O, use `/opt/ros/one/setup.bash` and `one.rosinstall` instead. The
package exports its Gazebo model directory and installs scripts, launch files,
configuration, URDF, messages, and worlds into the Catkin install space.

## Usage

Load the manipulation configuration without moving either robot:

```bash
roslaunch cooperation_landing manipulation.launch
```

Start the current state machine explicitly. Flight and payload transfer remain
disabled unless their separate safety gates are also enabled:

```bash
roslaunch cooperation_landing manipulation.launch run_manipulation:=true
```

After simulation validation and hardware safety checks, enable the guarded
operations with `allow_takeoff:=true allow_payload_transfer:=true`.

Publish the current graph for a SMACH viewer without executing its states:

```bash
roslaunch cooperation_landing manipulation.launch run_manipulation:=true display_only:=true
```

`robot_ns` and `ground_robot_ns` launch arguments configure both the task
interfaces and the stall monitor. They default to `xuanwu` and `go1`.

Start joystick control:

```bash
roslaunch cooperation_landing joy_stick.launch
```

Start the Gazebo scene:

```bash
roslaunch cooperation_landing simulation/bricks_simulation.launch
```

Run the autonomous alignment demonstration only after completing the safety
checks:

```bash
roslaunch cooperation_landing test_align_and_land.launch \
  run_demo:=true allow_takeoff:=true demo_target_index:=0
```

The network bridge endpoints are configured with launch arguments. Override
the peer IP rather than editing the launch files:

```bash
roslaunch cooperation_landing GR2UAV.launch UAV_IP:=10.0.0.20
roslaunch cooperation_landing UAV2GR.launch GROUND_ROBOT_IP:=10.0.0.10
```

For visual landing, start the ground camera, AprilTag detector and relative
pose estimator with one command on the ground computer:

```bash
roslaunch cooperation_landing apriltag_relative_pose.launch robot_ns:=xuanwu
```

If the camera and detector are already running, add `start_camera:=false`.
Set `ground_robot_ns` when the detector uses a namespace other than `qilin`.
The estimator publishes the full UAV pose in the docking frame on
`/xuanwu/visual_landing/info`. The GR2UAV bridge forwards that measurement;
Xuanwu bringup runs the UAV-local controller and executes flight commands.
This package does not launch a second UAV visual controller.

The GR2UAV and UAV2GR launches start both high-speed and low-speed network
transport, including the visual-landing trigger. They do not start the ground
estimator or the Xuanwu UAV controller.

Visual landing also has an independent `std_msgs/Empty` cancel event on
`/xuanwu/visual_landing/cancel`. The Ground `GR2UAV.launch` sends it only on
receipt of the event through low-speed `LOW_PORT_9=1066`; the UAV
`UAV2GR.launch` receives it on the same port. The existing start trigger remains
on `LOW_PORT_8=1065` and still starts a session. With both bridge endpoints
running, send cancel from a Ground robot terminal with:

```bash
rostopic pub -1 /xuanwu/visual_landing/cancel std_msgs/Empty "{}"
```

For a local UAV check, publish the same command on the UAV ROS master. Publish
`/xuanwu/visual_landing/trigger` with the same message type to start, then
observe `/xuanwu/visual_landing/state`: `0` (IDLE), `1` (ALIGNING), and `0`
after cancel. A new trigger is required after cancel. The controller ignores
cancel after standard JSK land handoff.

Run the Silverhammer event bridge with separate Ground and UAV ROS masters;
with a shared master, receiver output can feed the streamer again.

## Important parameters

The v2 state machine uses private parameters on the `manipulation_motion`
node. Common settings include:

- `robot_ns`, `ground_robot_ns`: aerial and ground robot namespaces.
- `picking_marker_far`, `picking_marker_near`, `placing_marker_far`,
  `placing_marker_near`: AprilTag IDs used at different ranges.
- `switching_threshold`, `detaching_takeoff_threshold`: task transition
  heights.
- `odom_wait_timeout_s`, `takeoff_state_timeout_s`,
  `landing_alignment_timeout_s`: bounded safety waits.
- `cog_odom_topic`: COG odometry used by the flight states, defaulting to
  `/<robot_ns>/uav/cog/odom` (`/xuanwu/uav/cog/odom`).
- `waypoint_position_tol_m`, `waypoint_yaw_tol_rad`, `waypoint_timeout_s`,
  `waypoint_odom_timeout_s`: arrival tolerances and feedback timeouts. FlyTarget
  and FlyBack confirm each waypoint with new world-frame odometry before
  advancing; stale or repeated source feedback cannot confirm arrival.
- `waypoint_no_progress_timeout_s`, `waypoint_progress_distance_m`,
  `waypoint_progress_yaw_rad`, `waypoint_retries`: re-send the same pose and
  trigger when progress stalls (default 3 seconds) or an attempt times out.
  The next waypoint is sent only after arrival at the current waypoint.
- `dock_min_linear_vel`, `dock_max_linear_vel`: docking control limits;
  commands run at 10 Hz without velocity smoothing.
- `dock_tag_timeout_s`, `dock_sit_recheck_timeout_s`: detection-cache age limit
  and wait for a new in-range detection after sitting.
- `enable_dog_stall_monitor`: enables command-versus-odometry stall recovery.
- `allow_takeoff`, `allow_payload_transfer`: explicit safety gates for flight
  and gripper payload operations; both default to `false`.

Transforms and calibrated landing points are loaded from `config/`. Treat
those values as robot-specific; do not reuse them on a different mechanical or
camera setup without recalibration.

Default state-machine tuning is collected in `config/Manipulation.yaml`.
For parameters exposed by `manipulation.launch`, launch arguments take
precedence over that file. Other settings can be set in the YAML file or a
custom launch file under the node's private namespace.

## Development and checks

```bash
python3 -m compileall -q src scripts setup.py
python3 -m ruff check src/cooperation_landing/apriltag_relative_pose.py scripts/apriltag_relative_pose.py
python3 -m ruff format --check src/cooperation_landing/apriltag_relative_pose.py scripts/apriltag_relative_pose.py
catkin build cooperation_landing --no-deps
```

Catkin commands above run from the package directory in a conventional
`<workspace>/src/cooperation_landing` layout. For a release, also build an
install space (`catkin config --install` in a separate validation workspace),
source `install/setup.bash`, and verify that both `cooperation_landing.msg`
and `cooperation_landing.control_utils` import from that space. The source
package and generated messages must coexist under the same Python namespace.

## IGMP helper

`send_igmp_report.py` sends one IGMPv2 membership report and normally requires
root or `CAP_NET_RAW`. `install_igmp_report.sh` copies this package's standalone
helper to `/usr/local/lib/cooperation_landing/` and installs the dedicated
`/etc/cron.d/cooperation_landing_igmp` entry, running every two minutes. It
requires `python3-netifaces` and does not need ROS sourced by cron. Inspect the
script before running it. Remove the job
with:

```bash
rosrun cooperation_landing install_igmp_report.sh --uninstall
```

Removal affects only that dedicated entry and helper. Jobs installed by older
versions in root's personal crontab must be removed separately.

## License

BSD-3-Clause. See [`LICENSE`](LICENSE).

Project documentation and experiment-specific notes are also available in the
[project wiki](https://github.com/Liyunong20000/cooperation_landing/wiki).
