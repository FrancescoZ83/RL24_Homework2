# Planning and Control with KDL

## :hammer: Setup

To install the KDL packages (kdl_parser and orocos_kdl) run from the ros2 workspace:
```sh
./src/setup.sh
```
If instead the requested libraries are already insalled, just run:
```sh
colcon build
source install/setup.sh
```

## 🚴‍♂️ Running the simulation

In order to run the gazebo simulation environment, do as following:
1. **Position Test:** To test the planned trajectories, run the position interface:
```sh
ros2 launch iiwa_bringup iiwa.launch.py
```
2. **Velocity Test:** You can also test the trajectories in velocity; The velocity interface is:
```sh
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
3. **Effort Control:** To execute the effort control, load Gazebo for the dynamics simulation and set the effort controller:
```sh
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"
```

## 🦾 Loading the control node

The node can accept different parameters from the shell to choose the command interface, the trajectory type and (in case of effort controller) the control type.
On a second terminal, the base command is the following (remember to add '-p' before each parameter)
```sh
ros2 run ros2_kdl_package ros2_kdl_node --ros-args
```
To set the parameters:
- `cmd_interface` - can be set to `position`, `velocity`, `effort`; To choose the command interface
- `traj_type` - can be set to `lin_pol`, `lin_trap`, `cir_pol`, `cir_trap`; To choose the trajectory type (linear or circular) and the s computation (polinomial or trapezoidal)
- `cont_type` - can be set to `jnt`, `op`; To choose the control type (joint or operational space)

### Examples

To test the linear trajectory in trapezoidal profile in position
```sh
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=position -p traj_type:=lin_trap
```

To test the circular trajectory in polinomial profile in velocity
```sh
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p traj_type:=cir_pol
```

To test the linear trajectory in polinomial profile in effort joint control
```sh
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p traj_type:=lin_pol -p cont_type:=jnt
```

To test the circular trajectory in trapezoidal profile in effort operational control
```sh
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p traj_type:=cir_trap -p cont_type:=op
```
