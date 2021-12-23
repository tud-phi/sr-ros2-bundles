# Soft Robotics ROS2 Docker image

Repository containing ROS2 Docker images including core ROS2 packages for operation of soft robot

## Docker Usage

### Build image (standard)

```bash
docker-compose build
```

### Build image (with Elastica)

```bash
docker-compose build --build-arg ELASTICA=true
```

### Build image (with PyTorch)

```bash
docker-compose build --build-arg PYTORCH=true
```

### Pull Docker image

```bash
docker pull ghcr.io/tud-cor-sr/sr-ros2-bundles:main
```

### Push Docker image

```bash
docker push ghcr.io/tud-cor-sr/sr-ros2-bundles:main
```

### Compose Docker container

Ubuntu host:

Make sure you have install the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker), if not follow [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

```bash
docker compose run sr-ros2-ubuntu
```

macOS host:

```bash
docker compose run sr-ros2-macos
```

Windows host:

```bash
docker compose run sr-ros2-windows
```

## GUI

### X Server

An X server is required for any GUI usage. The X server is started automatically when the container is started.
As an X client, [Xming](https://sourceforge.net/projects/xming/) can be used on Windows and [XQuartz](https://www.xquartz.org/) on macOS.
Please make sure that you allow connections from all clients and most of the time its better to disable OpenGL.

XQuartz can be started and all connections allowed with:
```bash
xhost +
```

### PlotJuggler

The open-source software PlotJuggler can be used to plot time series data from ROS topics. Please make sure to disable OpenGL in the PlotJuggler settings. It can be launched as follows:

```bash
ros2 run plotjuggler plotjuggler
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
