ARG FROM_IMAGE=ros:galactic
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for extending ros2 installation
FROM $FROM_IMAGE AS ros-desktop
RUN apt update &&\
    apt install -y --no-install-recommends\
      ros-$ROS_DISTRO-desktop\
      ros-$ROS_DISTRO-rqt*\
      ros-$ROS_DISTRO-plotjuggler-ros\
    && rm -rf /var/lib/apt/lists/*

# multi-stage for caching
FROM $FROM_IMAGE as cacher

#default value of arg ELASTICA when not provided in the --build-arg
ARG ELASTICA=false

# clone overlay & elastica(optional) source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY src/core.repos ../core.repos
COPY src/elastica.repos ../elastica.repos

RUN vcs import ./ < ../core.repos
RUN if [ "${ELASTICA}" = false ]; then\
      echo 'Not cloning Elastica';\
    else\
      echo "Cloning Elastica " &&\
      vcs import ./ < ../elastica.repos ;\
    fi

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM ros-desktop AS sr-ros2-bundles

# default value of arg ELASTICA when not provided in the --build-arg
ARG ELASTICA=false
ARG PYTORCH=false

# install some general system dependencies
RUN apt update && apt install -y --no-install-recommends \
    iputils-ping libeigen3-dev python3-pip python3-tk &&\
    rm -rf /var/lib/apt/lists/*

# install pytorch and libtorch
RUN if [ "${PYTORCH}" = "true" ]; then\
      pip3 install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html;\
    fi

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-path src \
      --rosdistro $ROS_DISTRO \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# Add sourcing of workspace to .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $OVERLAY_WS/install/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc

# set ROS_DOMAIN_ID
# ENV ROS_DOMAIN_ID=25

# install RMW middleware
# RUN apt-get update && \
#     apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
#     && rm -rf /var/lib/apt/lists/*
# ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# # Implement ROS2 networking with CYCLONE_DDS out of container
# # https://github.com/eclipse-cyclonedds/cyclonedds/issues/677#issuecomment-768966870
# ENV HOST_ADDR="192.168.200.169"
# # Using participant index
# ENV CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><ExternalNetworkAddress>${HOST_ADDR}</ExternalNetworkAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address='${HOST_ADDR}'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"

# run interactive shell
CMD ["/bin/bash"]

# run launch file
# CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
