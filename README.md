# Autonomous Delivery Rover URDF

A comprehensive URDF model of an autonomous delivery rover designed for ROS 2 simulation and visualization. This project was developed as part of a senior project at Cal Poly, San Luis Obispo, using GoBilda hardware components.

## Overview

This package provides a detailed URDF (Unified Robot Description Format) model featuring realistic scaling, visual meshes, and complete component layout for robot simulation in RViz2. The rover includes a four-wheel mecanum drive system with integrated sensors for autonomous navigation.

## Key Features

- Complete robot model with 9 detailed components
- High-quality STL visual meshes for realistic simulation
- Full ROS 2 compatibility with RViz2 visualization
- Mecanum wheel configuration for omnidirectional movement
- Integrated sensor suite including LiDAR and camera
- Realistic dimensional accuracy for simulation and planning

## Project Structure

```
autonomous-rover-urdf/
├── urdf/
│   └── rover.urdf              # Main robot description file
├── meshes/                     # STL visual models (download required)
│   ├── Base.stl                # Main chassis (28.5 MB)
│   ├── back_left_wheel.stl     # Rear left mecanum wheel (103.4 MB)
│   ├── back_right_wheel.stl    # Rear right mecanum wheel (95.3 MB)
│   ├── front_left_wheel.stl    # Front left mecanum wheel (95.3 MB)
│   ├── front_right_wheel.stl   # Front right mecanum wheel (103.4 MB)
│   ├── lidar.stl               # RPLIDAR sensor (154.2 MB)
│   ├── camera.stl              # Vision sensor (2.5 MB)
│   ├── battery.stl             # Power system (343 KB)
│   └── motherboard.stl         # Compute unit (15.3 MB)
├── launch/                     # ROS 2 launch files (optional)
└── README.md                   # Documentation
```

## System Requirements

### Storage Requirements
- **Minimum Required Storage**: 20-30 GB free disk space
- **Breakdown**:
  - STL mesh files: ~600 MB
  - ROS 2 installation: 3-5 GB (native) or 2-4 GB (conda/robostack)
  - Docker images (if using Docker): 5-8 GB
  - Conda environment (if using robostack): 2-3 GB
  - Workspace and build files: 2-3 GB
  - Operating system overhead: 10-15 GB

### Hardware Requirements
- **RAM**: 8 GB minimum, 16 GB recommended (for Docker + ROS + RViz)
- **CPU**: Multi-core processor (4+ cores recommended)
- **GPU**: Integrated graphics sufficient, dedicated GPU recommended for smooth 3D visualization
- **Network**: Internet connection required for initial setup and downloads

### Software Requirements
- **Operating System**: Ubuntu 20.04 LTS or later (recommended), macOS, or Windows with WSL2
- **ROS 2**: Foxy, Galactic, Humble, or Iron
- **Python**: 3.8 or later
- **Conda** (optional): Miniconda or Anaconda for robostack installation

## Required Downloads

Due to GitHub's file size limitations, all STL mesh files must be downloaded separately from Google Drive.

### Mesh Files (Total: ~600 MB)

| Component | File | Size | Description |
|-----------|------|------|-------------|
| Chassis | Base.stl | 28.5 MB | Main robot frame |
| Front Left Wheel | front_left_wheel.stl | 95.3 MB | Mecanum wheel assembly |
| Front Right Wheel | front_right_wheel.stl | 103.4 MB | Mecanum wheel assembly |
| Back Left Wheel | back_left_wheel.stl | 103.4 MB | Mecanum wheel assembly |
| Back Right Wheel | back_right_wheel.stl | 95.3 MB | Mecanum wheel assembly |
| LiDAR Sensor | lidar.stl | 154.2 MB | RPLIDAR sensor |
| Camera | camera.stl | 2.5 MB | Vision sensor |
| Battery | battery.stl | 343 KB | Power system |
| Motherboard | motherboard.stl | 15.3 MB | Compute unit |

**Download Link**: [Google Drive - All STL Files](https://drive.google.com/drive/u/1/folders/1DeFgrq8-kZised_OcWbh--EWJf4TB6_f)

## Installation

### Prerequisites

#### Option 1: Native ROS 2 Installation (Ubuntu/Debian)

Install required ROS 2 packages:

```bash
sudo apt update
sudo apt install ros-<your-ros-distro>-urdf-tutorial ros-<your-ros-distro>-rviz2
```

Replace `<your-ros-distro>` with your ROS 2 distribution (foxy, galactic, humble, iron).

#### Option 2: macOS with Miniforge (Recommended for macOS)

This method provides native macOS performance with working GUI - no Docker complications!

**Step 1: Install Miniforge**
```bash
# Download and install Miniforge
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-$(uname -m).sh"
bash Miniforge3-MacOSX-$(uname -m).sh

# Accept license, use default location, say yes to initialize
# Restart terminal or run:
source ~/.zshrc
```

**Step 2: Create ROS Environment**
```bash
# Create new conda environment for ROS
conda create -n ros_env
conda activate ros_env

# Add robostack channels
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --add channels robostack
conda config --env --remove channels defaults
```

**Step 3: Install ROS 2 Humble**
```bash
# Install ROS 2 Desktop (includes RViz2) - takes 5-10 minutes
conda install ros-humble-desktop ros-humble-robot-state-publisher
```

#### Option 3: Conda/Robostack Installation (Cross-platform)

For other platforms or users preferring conda environments:

```bash
# Install conda if not already installed
# Download from: https://docs.conda.io/en/latest/miniconda.html

# Create and activate ROS environment
conda create -n ros_env python=3.9
conda activate ros_env

# Add robostack channels
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging

# Install ROS 2 and required packages
conda install ros-humble-desktop-full
conda install ros-humble-urdf-tutorial

# Activate ROS environment (run this in each terminal session)
conda activate ros_env
```

**Note**: When using conda/robostack, replace `source /opt/ros/<distro>/setup.bash` with `conda activate ros_env` in all commands below.

### Setup Steps

1. **Clone the Repository**
   ```bash
   git clone https://github.com/Eman-Gon/autonomous-rover-urdf.git
   cd autonomous-rover-urdf
   ```

2. **Download Mesh Files**
   - Visit the [Google Drive link](https://drive.google.com/drive/u/1/folders/1DeFgrq8-kZised_OcWbh--EWJf4TB6_f)
   - Download all 9 STL files
   - Create a `meshes/` directory in your project folder
   - Place all downloaded STL files in the `meshes/` directory

3. **Verify Installation**
   ```bash
   # Check that all mesh files are present
   ls meshes/
   # Expected output: 9 .stl files listed above
   
   # Verify total file count
   ls meshes/*.stl | wc -l
   # Expected output: 9
   ```

4. **Setup ROS Workspace (Optional for Advanced Users)**
   ```bash
   mkdir -p ~/rover_ws/src
   cd ~/rover_ws/src
   cp -r /path/to/autonomous-rover-urdf .
   cd ~/rover_ws
   colcon build
   source install/setup.bash
   ```

### macOS Complete Setup Example

For macOS users using the Miniforge method:

```bash
# Step 1: Setup project
git clone https://github.com/Eman-Gon/autonomous-rover-urdf.git
cd autonomous-rover-urdf

# Step 2: Create meshes directory
mkdir meshes

# Step 3: Download STL files from Google Drive and place in meshes/ folder
# Or copy from existing location:
# cp /path/to/existing/meshes/*.stl meshes/

# Step 4: Verify all 9 files are present
ls meshes/

# Step 5: Every time you want to use ROS, activate environment first:
conda activate ros_env

# Step 6: Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/rover.urdf)" &

# Step 7: Launch RViz2
ros2 run rviz2 rviz2
```

## Usage

### Method 1: Quick Launch (Recommended)

**For Native ROS 2 Installation:**
```bash
# Source ROS 2 environment
source /opt/ros/<your-ros-distro>/setup.bash

# Navigate to project directory
cd /path/to/autonomous-rover-urdf

# Launch RViz2 with rover model
ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/urdf/rover.urdf
```

**For Conda/Robostack Installation:**
```bash
# Activate conda ROS environment
conda activate ros_env

# Navigate to project directory
cd /path/to/autonomous-rover-urdf

# Launch RViz2 with rover model
ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/urdf/rover.urdf
```

### Method 2: Manual Setup

**Terminal 1 - Robot State Publisher:**

For Native ROS 2:
```bash
source /opt/ros/<your-ros-distro>/setup.bash
cd /path/to/autonomous-rover-urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/rover.urdf)"
```

For Conda/Robostack:
```bash
conda activate ros_env
cd /path/to/autonomous-rover-urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/rover.urdf)"
```

**Terminal 2 - RViz2 Visualization:**

For Native ROS 2:
```bash
source /opt/ros/<your-ros-distro>/setup.bash
ros2 run rviz2 rviz2
```

For Conda/Robostack:
```bash
conda activate ros_env
ros2 run rviz2 rviz2
```

**RViz2 Configuration:**
1. Add a RobotModel display
2. Set Robot Description topic to `/robot_description`
3. Set Fixed Frame to `base_link`
4. The complete rover model should appear

## RViz2 Configuration Details

### Step-by-Step RViz2 Setup

1. **Set Fixed Frame**: Change from "map" to `base_link`
2. **Add Robot Model**: Click "Add" → "RobotModel" → "OK"
3. **Add TF Display (optional)**: Click "Add" → "TF" → "OK"
4. **Zoom in**: Your rover will be very small due to real-world scaling

### Expected Results
- All 9 rover components should be visible: base chassis, 4 mecanum wheels, LiDAR, camera, battery, motherboard
- Components should be properly connected and positioned
- No red error messages in RViz2 console
- Smooth 3D navigation and zoom functionality

### Method 3: Docker Setup

For containerized environments or if you prefer not to install ROS 2 directly:

#### Docker Prerequisites
- Docker installed
- X11 forwarding setup (Linux) or XQuartz (macOS)

#### Linux Docker Setup
```bash
# Enable GUI access
xhost +local:docker

# Pull ROS 2 Docker image
docker pull osrf/ros:humble-desktop-full

# Run container with GUI support
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/workspace" \
  osrf/ros:humble-desktop-full

# Inside container
cd /workspace
apt update && apt install ros-humble-urdf-tutorial -y
source /opt/ros/humble/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/rover.urdf
```

#### macOS Docker Setup
```bash
# Install and configure XQuartz
brew install --cask xquartz
# Open XQuartz → Preferences → Security → "Allow connections from network clients"
xhost +localhost

# Pull ROS 2 Docker image
docker pull osrf/ros:humble-desktop-full

# Run container with X11 forwarding
docker run -it --rm \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="$(pwd):/workspace" \
  osrf/ros:humble-desktop-full

# Inside container
cd /workspace
apt update && apt install ros-humble-urdf-tutorial -y
source /opt/ros/humble/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/rover.urdf
```

## Technical Specifications

| Specification | Value |
|---------------|-------|
| Total Links | 9 (base + 4 wheels + 4 sensors/components) |
| Joint Types | Continuous (wheels), Fixed (sensors) |
| Scale Factor | 0.001 (STL files in millimeters, converted to meters) |
| Coordinate Frame | ROS standard (X-forward, Y-left, Z-up) |
| Wheel Configuration | 4x Mecanum wheels for omnidirectional movement |
| Sensor Suite | LiDAR, Camera, Electronics |

## Verification

### Success Indicators

After successful setup, you should observe:

- Robot state publisher successfully loads all 9 components
- No "file not found" errors for STL meshes
- `/robot_description` topic is active and publishing
- RViz2 displays the complete rover model with all components visible
- All joints and links are properly connected

### Verification Commands

```bash
# Validate URDF syntax
ros2 run urdf_tutorial check_urdf urdf/rover.urdf

# Check active topics
ros2 topic list
# Should include: /robot_description, /tf, /tf_static

# Verify robot state publisher output
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/rover.urdf)"
# Should log successful loading of all 9 components

# Check mesh file integrity
file meshes/*.stl
# All files should be identified as "data" or STL files

# Check if all mesh files are present
ls -la meshes/
# Should show 9 .stl files

# Verify mesh file references in URDF
grep -n "mesh" urdf/rover.urdf

# Check ROS environment (for conda users)
conda activate ros_env
ros2 --version

# Test RViz2 (should open without errors)
ros2 run rviz2 rviz2
```

### Success Indicators

After successful setup, you should observe:

- Robot state publisher successfully loads all 9 rover components
- No "file not found" errors for STL meshes
- `/robot_description` topic is active and publishing
- RViz2 displays the complete rover model with all components visible
- All joints and links are properly connected
- Expected ROS topics active: `/robot_description`, `/tf`, `/tf_static`, `/joint_states`

### Expected Component List
The robot state publisher should log all 9 rover components:
- `base_link` (main chassis)
- `front_left_wheel`
- `front_right_wheel`
- `back_left_wheel`
- `back_right_wheel`
- `lidar`
- `camera`
- `battery`
- `motherboard`

### Verifying Success Without GUI
Even if RViz2 doesn't open, you can confirm the rover URDF is working:
```bash
# Check robot state publisher output - should show all 9 components:
# base_link, front_left_wheel, front_right_wheel
# back_left_wheel, back_right_wheel  
# lidar, camera, battery, motherboard

# Verify mesh file references
grep -n "mesh" urdf/rover.urdf

# Check active ROS topics
ros2 topic list
```

## Troubleshooting

### Common Issues

#### Missing STL Files Error
```
Error: Could not find file [meshes/lidar.stl]
```
**Solution**: Ensure all 9 STL files are downloaded from Google Drive and placed in the `meshes/` directory.

#### RViz2 Shows Empty or Broken Model
- Verify all 9 STL files are present in `meshes/`
- Check file permissions: `chmod 644 meshes/*.stl`
- Ensure URDF file paths match actual file locations
- Verify you have sufficient storage space (20-30 GB)

#### Package Not Found Error
```
Package 'urdf_tutorial' not found
```
**Solution**: Install missing ROS 2 packages:
```bash
sudo apt update
sudo apt install ros-<your-ros-distro>-urdf-tutorial ros-<your-ros-distro>-rviz2
```

#### Docker GUI Issues

**Linux:**
```bash
xhost +local:docker  # Allow Docker to access display
```

**macOS:**
```bash
# Install XQuartz
brew install --cask xquartz

# Configure XQuartz preferences to allow network connections
# Then allow Docker access
xhost +localhost

# Run Docker with proper display forwarding
docker run -it --rm \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="$(pwd):/workspace" \
  osrf/ros:humble-desktop-full
```

#### Insufficient Storage Space
If you encounter storage-related errors:
- Free up at least 20-30 GB of disk space
- Consider using external storage for mesh files
- Clean Docker images if using containerized setup: `docker system prune -a`

### Alternative Verification (No GUI Required)

If RViz2 fails to open, you can still verify the rover URDF:

**For Native ROS 2:**
```bash
# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/rover.urdf)" &

# Verify rover components are loaded
ros2 topic echo /robot_description

# Check transform tree
ros2 run tf2_tools view_frames

# Verify all expected topics are active
ros2 topic list
# Expected: /robot_description, /tf, /tf_static, /joint_states
```

**For Conda/Robostack:**
```bash
# Activate environment first
conda activate ros_env

# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/rover.urdf)" &

# Verify rover components are loaded
ros2 topic echo /robot_description

# Check transform tree  
ros2 run tf2_tools view_frames

# Verify all expected topics are active
ros2 topic list
# Expected: /robot_description, /tf, /tf_static, /joint_states
```

## Built With

- **ROS 2** - Robot Operating System framework
- **URDF** - Unified Robot Description Format
- **RViz2** - 3D visualization and debugging tool
- **Blender** - 3D modeling software for STL file creation
- **GoBilda** - Robotics hardware platform

## Contributing

This project was developed by Emanuel Gonzalez as part of a senior project at Cal Poly, San Luis Obispo.

### How to Contribute
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

For questions, issues, or suggestions, please open an issue on GitHub.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Author

**Emanuel Gonzalez**  
Project Developer  
GitHub: [Eman-Gon](https://github.com/Eman-Gon)

## Acknowledgments

- Cal Poly, San Luis Obispo for academic support
- GoBilda for hardware platform and components  
- ROS community for development tools and documentation
- Open source robotics community
