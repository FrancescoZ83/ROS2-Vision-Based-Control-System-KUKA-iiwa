FROM osrf/ros:humble-desktop

#Uncomment the following line if you get the "release file is not valid yet" error during apt-get
#	(solution from: https://stackoverflow.com/questions/63526272/release-file-is-not-valid-yet-docker)
#RUN echo "Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";" | cat > /etc/apt/apt.conf.d/10no--check-valid-until

#Install essential
RUN apt-get update && apt-get install -y

##You may add additional apt-get here
RUN apt-get install ros-humble-turtlesim
#RUN apt install ros-humble-plotjuggler-ros

#Gazebo ignition
RUN apt-get install -y lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME=/home/user
ENV ROS_DISTRO=humble

#Add non root user using UID and GID passed as argument
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers
USER user

#ROS2 workspace creation and compilation
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws
#COPY --chown=user ./src ${HOME}/ros2_ws/src
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep update; rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y; colcon build --symlink-install

#Add script source to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ${HOME}/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ${HOME}/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${HOME}/.bashrc

USER root
RUN apt-get upgrade -y && apt-get update -y

RUN apt-get install -y ros-humble-ros-ign-bridge && \
apt-get install -y ros-humble-ros-gz
RUN apt-get install ros-humble-controller-manager -y
RUN apt-get install ros-humble-ros2-control -y
RUN apt-get install ros-humble-ros2-controllers -y
RUN apt-get install ros-humble-ign-ros2-control -y
RUN apt-get install ros-humble-ign-ros2-control-demos -y
RUN apt-get install ros-humble-joint-state-publisher-gui -y
RUN apt-get install ros-humble-xacro

RUN apt-get install ros-humble-usb-cam -y
RUN apt-get install ros-humble-image-pipeline -y
RUN apt-get install ros-humble-tf-transformations -y

RUN apt-get upgrade -y && apt-get update -y

RUN apt-get install ros-humble-navigation2 -y 
RUN apt-get install ros-humble-nav2-bringup -y
RUN apt-get install ros-humble-robot-localization -y

#RUN export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models
ENV GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models


#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
#RUN chown root:user /dev/video0
USER user
