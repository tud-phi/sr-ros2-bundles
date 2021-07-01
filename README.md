# Soft Robotics ROS2 Docker image
Repository containing ROS2 Docker images including core ROS2 packages for operation of soft robot
## Usage
### Build image
```bash
docker compose build
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
```bash
docker compose run sr-ros2
```

## Some random advice

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
```
setenv("ROS_DOMAIN_ID", "25")
```

### RMW implementation
On Windows, we sometimes need to set our RMW implementation manually:
```bash
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
