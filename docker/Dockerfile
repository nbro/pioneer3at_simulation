# pioneer-simulation
FROM osrf/ros:melodic-desktop-full
RUN apt-get -y update && apt-get install -y ros-melodic-desktop-full
CMD source /opt/ros/melodic/setup.bash
##
# Extra packages
##
RUN apt-get -y update && apt-get install -y ros-melodic-ros-control libarmadillo-dev curl git python-catkin-tools dbus python-pip gfortran nano packagekit-gtk3-module libcanberra-gtk-module
RUN apt-get -y update && apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-rqt-common-plugins ros-melodic-dynamic-reconfigure
RUN /bin/bash -c " cd ~/ ; pip install scipy ; pip install numpy ; pip install sklearn; pip install pandas"
RUN apt-get -y update && apt-get install -y gedit eog nautilus ros-melodic-nodelet ros-melodic-yocs-msgs ros-melodic-ecl-* ros-melodic-ar-* ros-melodic-base-local-planner ros-melodic-move-base-*
##
# workspace setup
##
RUN  /bin/bash -c "mkdir -p ~/catkin_ws/src ; source /opt/ros/melodic/setup.bash ; cd ~/catkin_ws/src ; source /opt/ros/melodic/setup.bash ; catkin_init_workspace; mkdir -p ~/dataset_cvs"

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:/usr/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/lib64:${LD_LIBRARY_PATH}
ENV LIBGL_DEBUG=verbose

##
#
##
WORKDIR /root/catkin_ws
CMD bash

