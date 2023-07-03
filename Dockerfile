ARG FROM_IMAGE=ros:iron
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

#default value of args when not provided in the --build-arg
ARG ELASTICA=false
ARG HSA=false

ARG OVERLAY_WS

# copy the contents of the repo src folder to the cache image
# e.g. copy ./src to $OVERLAY_WS
COPY ./src $OVERLAY_WS

RUN apt update &&\
    apt install -y --no-install-recommends\
      openssh-client\
    && rm -rf /var/lib/apt/lists/*

# share ssh keys with the container
# Option 4 in: https://www.fastruby.io/blog/docker/docker-ssh-keys.html
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

WORKDIR $OVERLAY_WS/src
# clone sources
RUN --mount=type=ssh vcs import ./ < ../core.repos
RUN --mount=type=ssh if [ "${ELASTICA}" = false ]; then\
      echo 'Not cloning Elastica';\
    else\
      echo "Cloning Elastica " &&\
      vcs import ./ < ../elastica.repos ;\
    fi
RUN --mount=type=ssh if [ "${HSA}" = false ]; then\
      echo 'Not cloning HSA';\
    else\
      echo "Cloning HSA " &&\
      vcs import ./ < ../hsa.repos ;\
    fi

# copy manifests for caching
WORKDIR /opt
# RUN mkdir -p /tmp/opt && \
#     find ./ -name "package.xml" | \
#       xargs cp --parents -t /tmp/opt && \
#     find ./ -name "COLCON_IGNORE" | \
#       xargs cp --parents -t /tmp/opt || true
RUN mkdir -p /tmp/opt
RUN find ./ -name "package.xml" | xargs cp --parents -t /tmp/opt
# this code generates the following error currently:
# cp: missing file operand
# RUN find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM ros-desktop AS sr-ros2-bundles

# default value of arg ELASTICA when not provided in the --build-arg
ARG ELASTICA=false
ARG HSA=false
ARG PYTORCH=false
ARG SOFA=false
ARG SOFA_VERSION='21.06.02'

# install some general system dependencies
# libpython3.7 is required by the Sofa binaries.
# In case of conflict with other version of python, consider removing the line
RUN apt update && apt install -y --no-install-recommends software-properties-common \
    iputils-ping libeigen3-dev openssh-client python3-pip python3-tk wget unzip zip;\
    if [ "${SOFA}" = "true" ]; then\
      add-apt-repository -y ppa:deadsnakes/ppa && apt install -y libpython3.7 && add-apt-repository --remove -y ppa:deadsnakes/ppa;\
    fi;\
    rm -rf /var/lib/apt/lists/*

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

# install pip dependencies
RUN pip3 install --upgrade pip
COPY src/requirements.txt $OVERLAY_WS/requirements.txt
RUN pip3 install -r requirements.txt

# install pytorch and libtorch
RUN if [ "${PYTORCH}" = "true" ]; then\
      echo 'Installing PyTorch';\
      pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113;\
    fi

WORKDIR /opt/sofa
RUN if [ "${SOFA}" = "false" ]; then\
      echo 'Not installing Sofa';\
    else\
      echo 'Installing Sofa';\
      export SOFA_NAME=SOFA_v${SOFA_VERSION}_Linux ;\
      wget https://github.com/sofa-framework/sofa/releases/download/v${SOFA_VERSION}/${SOFA_NAME}.zip &&\
      unzip ${SOFA_NAME}.zip;\
      echo 'SofaPython3 NO_VERSION' >> ${SOFA_NAME}/lib/plugin_list.conf;\
      echo 'SoftRobots NO_VERSION' >> ${SOFA_NAME}/lib/plugin_list.conf;\
      rm ${SOFA_NAME}.zip &&\
      mv $SOFA_NAME sofa_v${SOFA_VERSION};\
      export QT_QPA_PLATFORM_PLUGIN_PATH=${QTDIR}/plugins &&\
      export QT_PLUGIN_PATH=${QTDIR}/plugins;\
    fi
WORKDIR $OVERLAY_WS

# Copy overlay src files
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src

# install overlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-path src \
      --rosdistro $ROS_DISTRO \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# install ros2-elastica dependencies
RUN if [ "${ELASTICA}" = "false" ]; then\
      echo 'Not installing Elastica';\
    else\
      echo "Installing Elastica " &&\
      apt-get update &&\
      apt-get install --no-install-recommends -y povray &&\
      rm -rf /var/lib/apt/lists/*;\
    fi

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
