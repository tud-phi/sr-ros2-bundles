ARG FROM_IMAGE=ros:jazzy
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
ARG PNEUMATIC=false

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
RUN --mount=type=ssh if [ "${PNEUMATIC}" = false ]; then\
      echo 'Not cloning pneumatic robot dependencies.';\
    else\
      echo "Cloning pneumatic robot dependencies." &&\
      vcs import ./ < ../pneumatic.repos ;\
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

SHELL ["/bin/bash", "-c"] 

# default value of args when not provided in the --build-arg
ARG ELASTICA=false
ARG HSA=false
# we install jax by default as jax-soft-robot-modelling depends on it
ARG JAX=true
ARG PYTORCH=false
ARG SOFA=false
ARG SOFA_VERSION='21.06.02'

# install some general system dependencies
# libsdl1.2-dev is required for the ros2-keyboard package
# libpython3.7 is required by the Sofa binaries.
# In case of conflict with other version of python, consider removing the line
RUN apt update && apt install -y --no-install-recommends software-properties-common \
    gh git-lfs htop iputils-ping libeigen3-dev libsdl1.2-dev nvtop openssh-client python3-venv vim wget unzip zip;\
    if [ "${SOFA}" = "true" ]; then\
      add-apt-repository -y ppa:deadsnakes/ppa && apt install -y libpython3.7 && add-apt-repository --remove -y ppa:deadsnakes/ppa;\
    fi;\
    rm -rf /var/lib/apt/lists/*

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

# create virtual environment
RUN mkdir -p /opt/ros/colcon_venv/src
# Make a virtual env and activate it
ENV VENV_PATH=/opt/ros/colcon_venv/venv
RUN python3 -m venv $VENV_PATH &&\
    source $VENV_PATH/bin/activate
ENV PIP_PATH=/opt/ros/colcon_venv/venv/bin/pip3
# Upgrade pip
RUN $PIP_PATH install --upgrade pip setuptools wheel
# Make sure that colcon doesn’t try to build the venv
RUN touch /opt/ros/colcon_venv/venv/COLCON_IGNORE

# install pip dependencies
COPY src/requirements.txt $OVERLAY_WS/requirements.txt
RUN $PIP_PATH install -r requirements.txt

# install jax with GPU support
# https://github.com/google/jax?tab=readme-ov-file#installation
RUN if [ "${JAX}" = "true" ]; then\
      echo 'Installing JAX with GPU support';\
      $PIP_PATH install --upgrade "jax[cuda12]";\
    fi

# install pytorch and libtorch
RUN if [ "${PYTORCH}" = "true" ]; then\
      echo 'Installing PyTorch';\
      $PIP_PATH install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118;\
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
COPY --from=cacher $OVERLAY_WS/src ./src

# install overlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-path src \
      --rosdistro $ROS_DISTRO \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# install pip packages depending on local, cloned repositories
# nml bag: useful for processing ROS2 bag files in Python
RUN $PIP_PATH install -e ./src/nml_bag
# install jax soft robot modelling if jax has been installed
RUN if [ "${JAX}" = "true" ]; then\
      $PIP_PATH install -e "./src/jsrm[dev, examples]";\
    fi

# install ros2-elastica dependencies
RUN if [ "${ELASTICA}" = "false" ]; then\
      echo 'Not installing Elastica';\
    else\
      echo "Installing Elastica " &&\
      apt-get update &&\
      apt-get install --no-install-recommends -y povray &&\
      rm -rf /var/lib/apt/lists/*;\
    fi

# install hsa-planar-control package and its dependencies
RUN if [ "${HSA}" = "true" ]; then\
      $PIP_PATH install ./src/hsa_planar_control;\
    fi

# build overlay source
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
RUN echo "source /opt/ros/colcon_venv/venv/bin/activate" >> ~/.bashrc
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

# copy the vscode settings to the image
COPY ./src/vscode_settings.json $OVERLAY_WS/.vscode/settings.json

# run interactive shell
CMD ["/bin/bash"]

# run launch file
# CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
