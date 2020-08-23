#
# Based on example Dockerfile from https://hub.docker.com/_/ros
#

ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# install turtlebot packages
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  ROBOTIS-GIT/turtlebot3: \n\
    type: git \n\
    url: https://github.com/ROBOTIS-GIT/turtlebot3.git \n\
    version: ros2 \n\
  ROBOTIS-GIT/turtlebot3_simulations: \n\
    type: git \n\
    url: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \n\
    version: ros2 \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && DEBIAN_FRONTEND=noninteractive rosdep install -y \
      --from-paths \
        src/ROBOTIS-GIT \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
     colcon build

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# Ensure setup scripts are sourced every time a new shell is opened
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $OVERLAY_WS/install/setup.bash" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$OVERLAY_WS/src/ROBOTIS-GIT/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc

# Install some useful tools
RUN apt-get update && apt-get install -y \
                                         tmux \
                                         curl

# Add a nice default .tmuxrc
RUN cd ~
RUN git clone https://github.com/gpakosz/.tmux.git
RUN ln -s -f .tmux/.tmux.conf
RUN cp .tmux/.tmux.conf.local .
RUN sed -i '/#set -g mouse on/c\set -g mouse on' .tmux.conf.local
