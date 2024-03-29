# Soft Robotics ROS2 Docker image

[![ci-main](https://github.com/tud-phi/sr-ros2-bundles/actions/workflows/main.yml/badge.svg)](https://github.com/tud-phi/sr-ros2-bundles/actions/workflows/main.yml)

Repository containing ROS2 Docker images including core ROS2 packages for operation of soft robot

## Initialization and sourcing

We have combined some convenience commands related to USB latency and X server access in the `init.sh` script, which can be run as

```bash
source ./init.sh
```

## Docker Usage

### Build image (standard)

```bash
docker compose build
```

### Build image for pneumatic robots

```bash
docker compose build --build-arg PNEUMATIC=true
```

or alternatively

```bash
docker compose build sr-ros2-pneumatic-ubuntu
```

### Build image for HSA robots

```bash
docker compose build --build-arg HSA=true
```

or alternatively

```bash
docker compose build sr-ros2-hsa-ubuntu
```

### Build image (with Elastica)

```bash
docker compose build --build-arg ELASTICA=true
```

### Build image (with PyTorch)

```bash
docker compose build --build-arg PYTORCH=true
```

### Build image (with SOFA)

```bash
# Build with default SOFA version (21.06.02)
docker compose build --build-arg SOFA=true
```

```bash
# Build with specific SOFA version
docker compose build --build-arg SOFA=true --build-arg SOFA_VERSION=21.12.00
```

### Pull Docker image

```bash
docker pull ghcr.io/tud-phi/sr-ros2-bundles:main
```

### Push Docker image

```bash
docker push ghcr.io/tud-phi/sr-ros2-bundles:main
```

### Compose Docker container

Ubuntu host:

Make sure you have install the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker) if you are running on an Ubuntu host, if not follow [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker). Please be aware, that the NVIDIA Container Toolkit sometimes requires sudo to run the commands below.

```bash
sudo docker compose run sr-ros2-core-ubuntu
```

macOS host:

```bash
docker compose run sr-ros2-core-macos
```

Windows host:

```bash
docker compose run sr-ros2-core-windows
```

## GUI

### X Server

An X server is required for any GUI usage. The X server is started automatically when the container is started.
As an X client, [Xming](https://sourceforge.net/projects/xming/) can be used on Windows and [XQuartz](https://www.xquartz.org/) on macOS.
Please make sure that you allow connections from all clients and most of the time its better to disable OpenGL.

XQuartz can be started and all connections allowed with:

```bash
xhost +local:all
```

#### Running Qt Applications on macOS

Most ROS2 GUI applications such as `rviz2`, `rqt` etc. require the Qt framework. For this to work on a macOS host, we need to run first on the host

```bash
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
```

as explained in [this](https://diysar.medium.com/running-qt-application-using-docker-on-macos-x-ad2e9d34532a) tutorial.

### PlotJuggler

The open-source software PlotJuggler can be used to plot time series and export CSV data from ROS topics.
It can be launched as follows:

```bash
ros2 run plotjuggler plotjuggler --buffer_size 180
```

The buffer size flag determines the maximum size of the streaming buffer.

Please make sure to disable OpenGL on non-linux hosts:

```bash
ros2 run plotjuggler plotjuggler --disable_opengl
```

## Some random advice

### Set RMW implementation to Cyclon DDS

On Ubuntu:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

On Windows:

```bash
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Sourcing of Workspace

On Windows:

```bash
call C:\dev\ros2_foxy\local_setup.bat
```

### Domain ID

As soon as we run ROS2 on different hosts, containers etc. we need to set a domain id.
On Ubuntu:

```bash
export ROS_DOMAIN_ID=25
```

On Windows:

```bash
set ROS_DOMAIN_ID=25
```

In Matlab:

```matlab
setenv("ROS_DOMAIN_ID", "25")
```

### Building on Windows

Source Visual Studio Command prompt:

```bash
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
```

### Set USB Latency for Dynamixel motors

As explained in the [Dynamixel documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#graph-optimization), the default USB latency is 16ms. Usually, we would want to reduce this to 1ms. On Ubuntu, the following command can be run:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
