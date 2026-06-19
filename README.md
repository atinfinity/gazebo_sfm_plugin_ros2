# Gazebo SFM Plugin for ROS 2 (Gazebo Ignition)

<p align="center">
  <img src="assets/demo.gif">
</p>

Social Force Model (SFM) pedestrian plugins for **Gazebo Ignition** and **ROS 2**, with LiDAR-detectable collisions. The persons are affected by the obstacles and other persons using the [Social Force Model](https://github.com/robotics-upo/lightsfm). This package was ported from [gazebo_classic_sfm_plugin_ros2](https://github.com/ZhanyuGuo/gazebo_classic_sfm_plugin_ros2).

> [!NOTE]
> Originally written for Gazebo Classic (Gazebo 11) + ROS 2 Humble, this package has
> been ported to the new Gazebo (gz-sim) architecture. The actor is driven through
> the `TrajectoryPose` / `AnimationTime` components of a `gz::sim::System` plugin.
> Because actors in the new Gazebo are visual-only, each pedestrian optionally drags
> a **kinematic collision cylinder** (via `AttachModelPlugin`) so it can be sensed by
> a LiDAR. The old `CollisionActorPlugin` (skeleton-link collisions) is no longer used.

> [!NOTE]
> The **lightsfm** is placed in [include/lightsfm](include/lightsfm).

## Supported Versions

| Component | Supported version |
|---|---|
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (`gz-sim` 8) |
| OS | Ubuntu 24.04 (Noble) |

## Quick Start

Tested on Ubuntu 24.04 with **Gazebo Harmonic (gz-sim8)** and **ROS 2 Jazzy**.

> The actor skin/animation uses the bundled `meshes/walk.dae`, so no internet
> connection or Gazebo Fuel download is required.

### 1. Clone the repository into your ROS 2 workspace:

```bash
cd ~/dev_ws/src
git clone https://github.com/atinfinity/gazebo_sfm_plugin_ros2.git
```

### 2. Build the workspace:

```bash
cd ~/dev_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Source the workspace:

```bash
source ~/dev_ws/install/setup.bash
```

### 4. Run the demo:

```bash
ros2 launch gazebo_sfm_plugin_ros2 sfm.launch.py
```

## Plugin Parameters

All parameters are given as child elements of the `<plugin>` tag inside an
`<actor>` (for `PedestrianSFMRandomPlugin`) or a model/actor (for
`AttachModelPlugin`). See `worlds/sfm.sdf.in` for a complete example.

### `PedestrianSFMRandomPlugin`

Drives an actor along Social-Force-Model trajectories and plays its walking
animation.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `velocity` | double | `0.8` | Desired walking speed [m/s]. |
| `radius` | double | `0.35` | Agent radius [m] used in the SFM interaction forces. |
| `goal_weight` | double | `2.0` | Weight of the attractive force toward the current goal (`forceFactorDesired`). |
| `obstacle_weight` | double | `10.0` | Weight of the repulsive force from obstacles (`forceFactorObstacle`). |
| `social_weight` | double | `2.1` | Weight of the social repulsive force from other pedestrians (`forceFactorSocial`). |
| `group_gaze_weight` | double | `3.0` | Weight of the group gaze force (`forceFactorGroupGaze`). |
| `group_coh_weight` | double | `2.0` | Weight of the group cohesion force (`forceFactorGroupCoherence`). |
| `group_rep_weight` | double | `1.0` | Weight of the intra-group repulsion force (`forceFactorGroupRepulsion`). |
| `animation` | string | `walking` | Name of the skeleton animation to play (must match the `<animation name="...">` of the actor). |
| `animation_factor` | double | `4.5` | Scale that syncs walking-animation time with distance traveled (larger = faster leg motion). |
| `people_distance` | double | `5.0` | Maximum distance [m] at which other pedestrians are detected. |

Composite elements:

| Element | Meaning |
|---|---|
| `<group>` | Defines the actor's walking group. Contains one or more `<model>NAME</model>` child elements listing the group members. Presence enables group behavior. |
| `<ignore_obstacles>` | Models to ignore in obstacle avoidance. Contains one or more `<model>NAME</model>` elements. The actor's own model and all other actors are ignored automatically. |
| `<trajectory>` | Waypoint route to follow. Contains `<cyclic>true/false</cyclic>` and one or more `<waypoint>x y z</waypoint>` (only X/Y are used). |
| `<random_trajectory>` | Auto-generated cyclic route (used only if `<trajectory>` is absent). Children: `<steps>` (int, number of goals), `<seed>` (int, RNG seed), `<origin>x y z</origin>`, `<rx>` and `<ry>` (doubles, radii of the random distribution). |

> `<trajectory>` and `<random_trajectory>` are mutually exclusive; if both are
> present, `<trajectory>` takes precedence.

### `AttachModelPlugin`

Makes other world models follow the actor's pose each step, so a kinematic
collision body can be dragged along with the (visual-only) actor and sensed by a
LiDAR. The followed model is teleported via `components::WorldPoseCmd`, so it
should be a non-static, gravity-disabled (or kinematic) model.

| Element | Meaning |
|---|---|
| `<link>` | Repeatable. One attachment group. |
| `<link>/<link_name>` | Name of a link in the parent (informational; the model follows the actor's body pose). |
| `<link>/<model>` | Repeatable. A world model to drag along. |
| `<link>/<model>/<model_name>` | Name of the world model to follow. |
| `<link>/<model>/<pose>` | Offset pose `x y z roll pitch yaw` in the parent (actor) frame. Defaults to identity. |

> Only a cylinder collision is supported for the follower body: gz-sim does not
> expose actor skeleton bone poses to plugins, so a follower collision can only
> track the actor's position and yaw (not its leg animation), and a Z-symmetric
> cylinder is the only shape that looks correct under that constraint.

## Adding an Actor to the World

Actors live in the world **template** `worlds/sfm.sdf.in`. Do **not** edit a
generated `.sdf`: at launch, `sfm.launch.py` replaces every `@MESH_FILE@` with
the absolute path of the bundled `meshes/walk.dae` and writes the world that gz
actually loads.

### 1. Add a visual-only actor (no collision)

Add an `<actor>` inside `<world>`. Give it a unique name, a start `<pose>`, and
a trajectory:

```xml
<actor name="actor3">
  <pose>3 0 1.0 0 0 0</pose>
  <skin>
    <filename>@MESH_FILE@</filename>
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>@MESH_FILE@</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <plugin filename="PedestrianSFMRandomPlugin"
          name="gazebo_sfm_plugin_ros2::PedestrianSFMRandomPlugin">
    <velocity>1.0</velocity>
    <radius>0.4</radius>
    <animation>walking</animation>
    <animation_factor>5.1</animation_factor>
    <people_distance>6.0</people_distance>
    <goal_weight>2.0</goal_weight>
    <obstacle_weight>80.0</obstacle_weight>
    <social_weight>15</social_weight>
    <ignore_obstacles>
      <model>ground_plane</model>
    </ignore_obstacles>
    <trajectory>
      <cyclic>true</cyclic>
      <waypoint>3 0 1.0</waypoint>
      <waypoint>-3 0 1.0</waypoint>
    </trajectory>
  </plugin>
</actor>
```

Key points:

- Use `@MESH_FILE@` for the skin/animation filename (not a relative path): gz
  mis-resolves relative actor mesh paths and the walking animation breaks.
- Do **not** add an SDF `<script>` trajectory; motion and the walking animation
  are driven entirely by the plugin.
- The actor's start `<pose>` Z should be `1.0` (the mesh hips sit ~1 m above the
  feet); the plugin keeps the actor standing on the ground.
- See [Plugin Parameters](#plugin-parameters) for the full parameter list, and
  use `<random_trajectory>` instead of `<trajectory>` for an auto-generated route.

### 2. (Optional) Add a LiDAR-detectable collision

To make the new actor sensed by a LiDAR, drag a kinematic collision cylinder
with it.  
First, **before** the actor, declare the collision model (so `AttachModelPlugin`
can find it at load time):

```xml
<model name="actor3_collision_model">
  <pose>3 0 0 0 0 0</pose>
  <link name="link">
    <kinematic>true</kinematic>
    <gravity>false</gravity>
    <inertial>
      <mass>1.0</mass>
      <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="collision">
      <pose>0 0 0.9 0 0 0</pose>
      <geometry><cylinder><radius>0.3</radius><length>1.8</length></cylinder></geometry>
    </collision>
  </link>
</model>
```

Then add an `AttachModelPlugin` to the actor (alongside the SFM plugin):

```xml
<plugin filename="AttachModelPlugin" name="servicesim::AttachModelPlugin">
  <link>
    <link_name>Hips</link_name>
    <model>
      <model_name>actor3_collision_model</model_name>
      <pose>0 0 0 0 0 0</pose>
    </model>
  </link>
</plugin>
```

Finally, list the new collision model in every actor's `<ignore_obstacles>` so
the cylinders don't repel each other or their own actor:

```xml
<ignore_obstacles>
  <model>ground_plane</model>
  <model>actor1_collision_model</model>
  <model>actor3_collision_model</model>
</ignore_obstacles>
```

### 3. Build and run

```bash
cd ~/dev_ws
colcon build --symlink-install --packages-select gazebo_sfm_plugin_ros2
source install/setup.bash
ros2 launch gazebo_sfm_plugin_ros2 sfm.launch.py
```

## References

Thanks for the following prior works:

- [gazebo_classic_sfm_plugin_ros2](https://github.com/ZhanyuGuo/gazebo_classic_sfm_plugin_ros2)
- [gazebo_sfm_plugin](https://github.com/robotics-upo/gazebo_sfm_plugin)
- [lightsfm](https://github.com/robotics-upo/lightsfm)
- [turtlebot3_simulations_jp_custom](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_simulations_jp_custom)
- [gazebo_sfm_tb3_ros2](https://github.com/koichirokato/gazebo_sfm_tb3_ros2)
- [sfm_for_gazebo](https://github.com/tech-life-hacking/sfm_for_gazebo)
- [gazebo-actor](https://github.com/BruceChanJianLe/gazebo-actor)
- [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning)
