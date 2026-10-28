# Lab 1 – ROS 2, Turtlesim, Actions & Services

**Team:** Name1 (ID), Name2 (ID), Name3 (ID)   <!-- TODO: fill in -->

## Packages
| Package | Contents |
|---|---|
| `turtle_interfaces` | `msg/TargetPose` (x,y,yaw), `srv/SetSpeedLimits`, `action/GoToPose` (goal = TargetPose), `action/DrawSquare` |
| `turtle_control` | `goto_pose_server` (action `/go_to_pose` + service `/set_speed_limits` + parameters), `reset_service` (`/reset_turtle`), `square_server` (action `/draw_square`), launch files, YAML config |

## Build
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <YOUR_REPO_URL> .
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src   # needs turtlesim
colcon build
source install/setup.bash
```

## Run
```bash
ros2 launch turtle_control system.launch.py                       # everything
ros2 launch turtle_control system.launch.py params_file:=/path/my.yaml
ros2 launch turtle_control square_demo.launch.py                  # system + auto square
```

## Try every requirement
```bash
# R2 / R8 interfaces
ros2 interface list | grep turtle_interfaces
ros2 interface show turtle_interfaces/msg/TargetPose
ros2 interface show turtle_interfaces/action/GoToPose
ros2 interface show turtle_interfaces/srv/SetSpeedLimits

# R3 goToPose
ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose \
  "{target: {x: 8.0, y: 3.0, yaw: 1.57}}" --feedback

# R4 reset
ros2 service call /reset_turtle std_srvs/srv/Trigger

# R5 square (side_length 0 -> default 3.0, max 5.0)
ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback

# R7 parameters (use decimals, e.g. 1.0)
ros2 param list /goto_pose_server
ros2 param get /goto_pose_server max_linear_speed
ros2 param set /goto_pose_server max_linear_speed 1.0
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits \
  "{max_linear_speed: 0.8, max_angular_speed: 1.0}"
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits \
  "{max_linear_speed: -1.0, max_angular_speed: 1.0}"     # refused
```

## Design notes
* Parameters are read inside the control loop every cycle, so changes apply live.
* Goals outside the 0–11 window are rejected; only one goToPose goal runs at a time.
* Servers use `MultiThreadedExecutor` + `ReentrantCallbackGroup` so the square server can wait on the
  reset service and goToPose action from inside its callback.
* Square: reset -> 4 sequential goToPose goals (counter-clockwise from the centre).

## Bonus – design patterns
* **State** – `controller.py`: `PoseController` is a state machine (`ROTATE_TO_GOAL -> DRIVE -> ALIGN_YAW -> DONE`),
  keeping control logic separate from ROS code.
* **Observer** – the ROS subscription to `/turtle1/pose` feeds the latest pose to the server.

## Bonus – bag file
```bash
ros2 bag record -o square_bag /turtle1/cmd_vel /turtle1/pose   # while the square runs
ros2 bag info square_bag
ros2 service call /reset_turtle std_srvs/srv/Trigger           # same start pose
ros2 bag play square_bag --topics /turtle1/cmd_vel
```
