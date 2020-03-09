# Custom Pioneer 3-AT simulation

This is a ROS package that can be used to simulate a Pioneer 3-AT robot with several cameras attached to it. You can enable and disable certain cameras and other sensors, if you need.

## How to obtain this ROS package?

You can simply clone this repo into your Catkin workspace's `src` folder by issuing the following command (from inside your workspace's `src` folder):

    git clone https://github.com/nbro/pioneer3at_simulation.git

You will likely encounter several issues when you build your workspace if you're not using Gazebo 9.0.0. I recommend that you use [ROS Melodic](http://wiki.ros.org/Installation/Ubuntu) (with Gazebo 9.0.0.).

## Install dependencies and build workspace 

To install the dependencies required by this package, inside your workspace, execute the following command

    rosdep install --ignore-packages-from-source --from-paths src

Then you can build the workspace (with Catkin Tools)

    catkin build

## Launch

To launch a Gazebo world with a Pioneer robot, you can issue the following command from the terminal

    roslaunch pioneer3at_simulation gazebo.launch world_name:=color_plane_bw

In this case, the world called `color_plane_bw` will be used with a _pioneer 3-AT model_ in the middle of it. There's also another model called `color_plane`.

### ROS topics

Using `rostopic list`, you can see the available topics that are exposed from the simulation (e.g. `/pioneer3at/odom`).

#### Control

A `cmd_vel` topic is exposed, so that linear and angular velocities can be sent to the simulated robot.

## Customisation

Pioneer 3-AT model can be modified by editing the `pioneer3at.urdf.xacro` file (bottom section) or the file `model.sdf` under the `models` folder. 

There more cameras and laser sensors can be placed or the values of the current ones modified.

Boolean contact information is also provided for the wheels and main body. A developed gazebo plugin carries this out.

Coloring of the plane is done using textures (`materials` in gazebo). We can modify such coloring by editing our custom `color_plane` model or create new ones and place them in `models` folder.
