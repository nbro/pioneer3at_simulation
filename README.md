# Custom Pioneer 3-AT simulation

Simulation of the Pioneer 3-AT with several cameras attached to it.

## Run

### Docker

There is a dockerfile you can build to run the container.

### Host

Or you can copy the `catkin_ws_src` to your ROS work-space and run it there.

## Launch

Run `roslaunch pioneer3at_simulation gazebo.launch world_name:=color_plane` to launch gazebo simulator + pioneer 3-AT model + custom map.

### Data

Using `rostopic list` you can see the available topics that are exposed from the simulation (i.e. odom, cameras, laser, contact points)

### Control

A `cmd_vel` topic is exposed so that linear and angular velocities can be sent to the simulated robot.

## Customisation

Pioneer 3-AT model can be modified by editing the `pioneer3at.urdf.xacro` file (bottom section). There more cameras and laser sensors can be placed or the values of the current ones modified.

Boolean contact information is also provided for the wheels and main body. A developed gazebo plugin carries this out.

Coloring of the plane is done using textures (`materials` in gazebo). We can modify such coloring by editing our custom `color_plane` model or create new ones and place them in `models` folder.
