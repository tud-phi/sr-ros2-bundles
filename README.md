# Soft Robotics ROS2 Docker image
Repository containing ROS2 Docker images including primary ROS2 packages for operation of soft robot
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