# Nodes and launch files
## Nodes
| Node                      | Description                      |
|---------------------------|----------------------------------|
|transforms.py              |Takes in the MsgState data and produces transforms|
|ch03_kinematic_mav_node.py |Subscribes to a wrench and publishes the uav_state resulting from the wrench.|
|ch04_dynamic_mav_node.py   |Subscribes to a `ControlSurfaceCommands` and  `WindVector`. Publishes the resulting uav state and wrench. |
|ch04_wind_node.py          |Publishes a wind vector (steady state and gust)|
|ch05_trim_traj_node.py     |Calculates and publishes a trim trajectory for the uav|
|ch06_autopilot_node.py     |Ingests a desired autopilot command message and outputs the surface commands|
|ch06_autopilot_dynamics_node.py | Combines both the autopilot node and the dynamics node in one to allow the feedback control to use the latest state|
|ch10_path_follower_node.py |Ingests a desired path segment and produces the appropriate autopilot command messages|
|ch11_path_manager_node.py  |Manages the basic path segments for following a series of waypoints |
|ch11_waypoints_publisher.py|Temporary node used for publishing a path to the uav                |
|ch12_path_planner_node.py  |Creates plans based upon the current position and desired goal position|
|ch12_world_manager_node.py |Manages the building world, publishing messages for visualization and communication of map|
|diagnostics_aggregator_node.py|For for a simple aggregation of diagnostics data to allow viewing of robot status by the rqt_robot_monitor|
|uav_plotting_node.py   | Plots graphs of the states and sensory information |

## Launch files
### ch02_transforms.launch.py
```
ros2 launch uav_launch ch02_transforms.launch.py
```
Runs the `transforms` node as well as opens rviz2.

To change the transforms, use the `State panel` with rviz.

By default, only one frame is shown in Rviz, *ned* (north-est-down). Additional frames can be visualized by opening the *Displays* menu on the left and selecting the desired frames under *TF->Frames*.

Also note the scale in rviz. Each grid square is 100 x 100 $m^2$, so correspondingly large values for position must be chosen to see the difference in position.

The axes are differentiated by color
* Red: x-axis  (body frame - out the nose of the aircraft, ned frame - north)
* Green: y-axis (body frame - out the right wing of the aircraft, ned frame - east)
* Blue: z-axis (body frame - out the belly of the aircraft, ned frame - down)

### ch03_kinematics.launch.py
```
ros2 launch uav_launch ch03_kinematics.launch.py
```

Runs the `transforms` and `ch03_kinematic_mav` nodes for publishing the uav state and transforms. It also opens rviz2, rqt_publisher, and `uav_plotter` for interacting with the sim. The `robot_state_publisher` is used to create the uav visualization. The rqt_publisher can be used to publish the forces and moments acting upon the UAV. Do the following in the rqt_publisher.
* Select the */forces_moments* topic from the "topic" drop down list
* Select a good publishing frequency (1 hz will do)
* Click the *+* button to add the publisher
* Select values for the force and torque vectors (i.e., a wrench)
* Click the checkbox to begin publishing

Keep in mind the rviz scale of 100 x 100 $m^2$. A small force will not appear to move the UAV much. However, a small torque will.

There is no need to restart the entire simulation to rerun a different wrench combination. Simply click the "Reset" button on the control panel in rviz2 and the UAV will move back to the starting position.

This reset works via a service call to set the UAV back to the starting position. This can be called from the rqt gui by doing the following.
* Uncheck the publishing of the wrench message in rqt_publisher
* In a new terminal, source the install directory and type the command `rqt` to bring up the rqt gui
* Select the *Service Caller* tool from the *Plugins->Services* menu
* Select ***/reset_state*** from the dropdown menu
* Click the *Call* button


### ch04_dynamics.launch.py
```
ros2 launch uav_launch ch04_dynamics.launch.py
```
Runs the `transforms`, `ch04_dynamic_mav`, and `ch04_wind` nodes as well as opens rviz2, rqt_publisher, and `uav_plotter`. Again, keep in mind the scale of rviz.

Before the vehicle will begin moving, you must click the "Start" button on the "Control Panel" in rviz2. You can then pause the sim by clicking the button again.

As in Chapter 3, the rqt_publisher can be used to publish commands to the UAV using the */command* topic. As in Chapter 3, the UAV state can also be reset using either the "Reset" button in the control panel or through a service call.

An additional service exists to set the desired state.
* In a new terminal, source the install directory and type the command `rqt` to bring up the rqt gui
* Select the *Service Caller* tool from the *Plugins->Services* menu
* Select ***/set_state*** from the dropdown menu
* Populate the desired state
  * The service only uses position, euler angles, and twist. Modifying any other variable will be ignored except for a warning output to the console.
  * It is assumed that the position is defined in the *ned* frame and that the twist is defined in the *body* frame. If an incorrect frame_id is seen, a warning will be pushed to the terminal. The variables will be used as if they had the correct frame_id.
* Click the *Call* button

### Ch05_trim.launch.py
```
ros2 launch uav_launch ch05_trim.launch.py
```
Runs the `transforms`, `ch04_dynamic_mav`, `ch04_wind`, and `ch05_trim_traj_node` nodes as well as opens rviz2, rqt_reconfigure, and `uav_plotter`. `rqt_robot_monitor` is also opened to allow easy monitoring of the status of different components. Again, keep in mind the scale of rviz.

Before the vehicle will begin moving, you must click the "Start" button on the "Control Panel" in rviz2. You can then pause the sim by clicking the button again.

New trim trajectories can be calculated and commanded through rqt_reconfigure.
* In rqt_reconfigure, click the *Refresh* button until *trim_traj_gen* appears in the left-hand menu
* Click on *trim_traj_gen*
* Modify parameters as desired
  * `va_trim`: Airspeed ($V_a$) for the trim trajectory
  * `gamma_trim`: Flight path angle ($\gamma$) for the trim trajectory
  * `ts`: Time in seconds between publication of the trim trajectory
* Once the parameters have been modified, the trim trajectory node will calculate the trim trajectory and reset the mav state by calling the `set_state` service with the updated trim state


### ch06_autopilot.launch.py
```
ros2 launch uav_launch ch06_autopilot.launch.py
```
Runs the `transforms`, `ch04_wind`, a `relay` for "state estimation", and `ch06_autopilot_dynamics` nodes. `rqt_robot_monitor` is also opened to allow easy monitoring of the status of different components. It also opens rviz2, rqt_publisher, and `uav_plotter`. Again, keep in mind the scale of rviz.

Before the vehicle will begin moving, you must click the "Start" button on the "Control Panel" in rviz2. You can then pause the sim by clicking the button again.

The autopilot command can be sent via the `/autopilot_command` message using the rqt_publisher.

### ch07_sensors.launch.py
```
ros2 launch uav_launch ch07_sensors.launch.py
```
This is the same as *ch06_autopilot.launch.py* with the addition of the sensors node.

### ch10_path_follower_node.py
```
ros2 launch uav_launch ch10_path_follower.launch.py
```
Runs the `transforms`, `ch04_wind`, a `relay` for "state estimation", `ch06_autopilot_dynamics`, and `ch10_path_follower` nodes. `rqt_robot_monitor` is also opened to allow easy monitoring of the status of different components. It also opens rviz2, rqt_publisher, and `uav_plotter`. Again, keep in mind the scale of rviz.

Before the vehicle will begin moving, you must click the "Start" button on the "Control Panel" in rviz2. You can then pause the sim by clicking the button again.

The path segment command can be sent via the `/path_segment` message using the rqt_publisher. Keep in mind the following restrictions on the `/path_segment` message:
* The *header.frame_id* must be "ned"
* *type* must be "line" or "orbit
* *airspeed* must be greater than $0$
* For a "line" type, the (north, east) (or (x,y)) vector of *line_direction* must have a magnitude greater than zero
* For an "orbit type,
  * *orbit_radius* must be positive
  * *orbit_direction* must be either "CW" or "CCW"

If any of the above constraints are not met, an error message will be published to the console and the autopilot command will not be published.

### ch11_path_manager.launch.py
```
ros2 launch uav_launch ch11_path_manager.launch.py
```
Runs the `transforms`, `ch04_wind`, a `relay` for "state estimation", `ch06_autopilot_dynamics`,  `ch10_path_follower`, and `ch11_path_manager` nodes. The temporary node, `ch11_waypoints_relay` is also included by default. `ch11_waypoints_relay` allows the waypoints created using the waypoints_panel in rviz2. Alternatively you may run `ch11_waypoints_publisher` manually with hard coded waypoints. It also opens rviz2 and `uav_plotter`. `rqt_robot_monitor` is also opened to allow easy monitoring of the status of different components. Again, keep in mind the scale of rviz. Also, note that the visualized waypoint path drawn 10 meters below its actual location so that the path segment path is visible.

Before the vehicle will begin moving, you must click the "Start" button on the "Control Panel" in rviz2. You can then pause the sim by clicking the button again.

As mentioned, the `ch11_waypoints_publisher` node provides a simple interface for creating different hard-coded waypoint paths to be followed.

The `ch11_waypoints_relay` node allows for waypoints to be created in rviz2. Use the `2D Goal Pose` in rviz2 to create a path one waypoint at a time. When there are at least three waypoints, click "Publish" in the waypoint panel and the waypoints will be sent to the `ch11_path_manager` for execution. Additional waypoints can be appended to the end by using the `2D Goal Pose`. If a new path is desired (or to simply remove the plotting of the created path) then click "Clear" in the waypoint panel.

### ch12_path_planner.launch.py
```
ros2 launch uav_launch ch12_path_planner.launch.py
```
Runs the `transforms`, `ch04_wind`, a `relay` for "state estimation", `ch06_autopilot_dynamics`,  `ch10_path_follower`, `ch11_path_manager`, and `ch12_path_planner` nodes. `rqt_robot_monitor` is also opened to allow easy monitoring of the status of different components. It also opens rviz2 and `uav_plotter`. Again, keep in mind the scale of rviz. Also, note that the visualized waypoint path drawn 10 meters below its actual location so that the path segment path is visible.

Before the vehicle will begin moving, you must click the "Start" button on the "Control Panel" in rviz2. You can then pause the sim by clicking the button again.

The buildings in the world are drawn as red rectangular boxes and the world boundaries are drawn as a green line. A different version of the world can be created by calling the `regen` service.

After executing the launch file, a goal can be published in two ways:
* Publish to the `/wp_panel/goal_pose` topic (i.e., from command line or via rqt_publisher)
  * If the frame id is "ned", then the goal position will be taken as is
  * If the frame id is "enu", then the altitude will be taken from the `goal_altitude` parameter by adding that value to the ground height at the specified (north,east) position. Note that `goal_altitude` can be altered from the launch file or via rqt_reconfigure
  * Any other frame_id will be rejected
* Use the RVIZ *2D Goal Pose* publishing capability combined with the waypoint_panel
  * Click on the *2D Goal Pose* button and then click within RVIZ to select a point.
  * This point is then published over the `/goal_pose` topic. The point is defined in the RVIZ fixed frame (defaulted to "enu).
  * The waypoint panel reads in this pose and converts it to the "ned" frame
  * When "Publish" is clicked within the waypoint panel, the goal point will be published to `/wp_panel/goal_pose`

Replanning will occur in the following situations
* A new goal is received via the `/wp_panel/goal_pose` topic
* A parameter is changed (`goal_altitude` or `planner_type`)
* A service call is made to the `/replan` service (this can be done via rqt)

When publishing a goal point, keep in mind that a plan will not be created if the goal point is in an obstacle, the goal point lies outside of the city region, or the resulting altitude is less than zero (i.e., "z" in "enu" must be >= 0). Corresponding error messages appear in the terminal when a plan is not possible.



## RVIZ Files
|Package      | File                 | Description                                         |
|-------------|----------------------|-----------------------------------------------------|
|uav_launch   | ch02_transforms.rviz | Provides a visualization of the uav, ned coordinate frame, and an interface to chang the state -- used in ch02 launch|
|uav_launch   | ch03_forces_moments.rviz | Provides a visualization of the uav and ned coordinate frame -- used in ch03 launch|
|uav_launch   | ch04_dynamics.rviz | Provides a visualization of the uav, coordinate frames, and a start/stop sim interface -- used in ch04-ch10|
|uav_launch   | ch11_path_creation.rviz | Provides a visualization of the uav, coordinate frames, start/stop interface, and a waypoint generation interface -- used in ch11 and ch12|