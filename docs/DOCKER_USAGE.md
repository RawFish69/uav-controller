# Docker Usage Guide for UAV-Controller

This guide explains how to use Docker to run the UAV-Controller project. Using Docker ensures consistent environment setup and eliminates potential compatibility issues.

## Prerequisites

- Docker installed on your system
- X11 for GUI applications (for visualization)

If you haven't installed these yet, run:
```bash
# Install Docker
sudo apt update
sudo apt install -y docker.io

# Install X11 utilities (for GUI support)
sudo apt install -y x11-xserver-utils

# Add your user to the docker group (to run docker without sudo)
sudo usermod -aG docker $USER
# Note: You'll need to log out and back in for this to take effect
```

## Building the Docker Image

1. Navigate to the project directory:
```bash
cd /path/to/UAV-Controller
```

2. Build the Docker image:
```bash
docker build -f docker/Dockerfile.humble -t uav-controller .
```

## Running the Project

### 1. Basic Run (No GUI)
```bash
# Enter the container with workspace mounted
docker run -it --rm \
    --volume="$PWD:/workspace" \
    uav-controller \
    bash

# Inside the container, install dependencies and build workspace
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Verify the installation
ros2 pkg list | grep sim_dyn  # Should show sim_dyn package
ls install/sim_dyn/lib/sim_dyn  # Check if executables are present
```

### 2. Run with GUI Support (for visualization)

First, on your host machine (not inside Docker), run:
```bash
# Enable X11 forwarding
xhost +local:root

# Set your display (use your actual DISPLAY value)
export DISPLAY=:0
```

Then start the container:
```bash
docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTHORITY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTHORITY:$XAUTHORITY:rw" \
    --volume="$PWD:/workspace" \
    --network=host \
    uav-controller \
    bash
```

### 3. Run Specific Launch Files

First, you need to enter the container and build the workspace:
```bash
# Enter the container with workspace mounted
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD:/workspace" \
    --network=host \
    uav-controller \
    bash

# Inside the container, clean and rebuild the workspace
cd /workspace/ros2_ws
rm -rf build/ install/ log/  # Clean the workspace

# Install Python packages in development mode
pip3 install -e src/sim_dyn/
pip3 install -e src/adapters_crsf/

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Verify the node executables are installed
ls install/sim_dyn/lib/sim_dyn
ls -l $(which dynamics_node)  # Should show the executable
```

### 2. Run with GUI Support
```

#### For PID Controller Simulation:
```bash
# After building and sourcing, run:
ros2 launch sim_dyn sim_pid.launch.py
```

#### For LQR Controller Simulation:
```bash
# After building and sourcing, run:
ros2 launch sim_dyn sim_lqr.launch.py
```

### 4. Run with Hardware (CRSF Link)

#### For PID Controller:
```bash
docker run -it --rm \
    --device=/dev/ttyUSB0 \  # Adjust device path as needed
    uav-controller \
    ros2 launch adapters_crsf crsf_adapter.launch.py controller:=pid
```

#### For LQR Controller:
```bash
docker run -it --rm \
    --device=/dev/ttyUSB0 \  # Adjust device path as needed
    uav-controller \
    ros2 launch adapters_crsf crsf_adapter.launch.py controller:=lqr
```

## Development Inside Container

To develop inside the container with a mounted workspace:
```bash
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD:/workspace" \
    --network=host \
    uav-controller \
    bash
```

This will mount your current directory to `/workspace` in the container, allowing you to edit files on your host machine and build/run them in the container.

## Useful Docker Commands

- List running containers:
  ```bash
  docker ps
  ```

- Stop a running container:
  ```bash
  docker stop <container_id>
  ```

- Remove all stopped containers:
  ```bash
  docker container prune
  ```

- Remove the built image:
  ```bash
  docker rmi uav-controller
  ```

## Important Notes

1. Always build the workspace after entering the container for the first time:
```bash
# First, install dependencies
cd /workspace/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Then build with the correct flags
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Verify installation
ros2 pkg list  # Should show all your packages
```

2. If you make changes to the code, you need to rebuild:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Troubleshooting

1. If you get permission errors:
   - Make sure you've added your user to the docker group
   - Log out and log back in for group changes to take effect

2. If GUI applications don't work:
   - Ensure X11 forwarding is enabled (`xhost +`)
   - Check if your display environment variable is set correctly

3. If hardware devices aren't accessible:
   - Verify the device path (e.g., /dev/ttyUSB0)
   - Ensure you have the correct permissions to access the device
   - Add your user to the dialout group: `sudo usermod -aG dialout $USER`