# Autonomous Rover URDF

URDF model of an autonomous delivery rover built using GoBilda hardware for a senior project at Cal Poly, San Luis Obispo. Designed for use with ROS 2 and RViz2, this package includes visual meshes, component layout, and realistic scaling for robot simulation.

## Project Structure

```
autonomous-rover-urdf/
├── urdf/
│   └── rover.urdf              # Main robot description file
├── meshes/                     # STL visual models (download from Google Drive)
│   ├── Base.stl
│   ├── back_left_wheel.stl
│   ├── back_right_wheel.stl
│   ├── front_left_wheel.stl
│   ├── front_right_wheel.stl
│   ├── lidar.stl
│   ├── camera.stl
│   ├── battery.stl
│   └── motherboard.stl
├── launch/                     # Launch files (optional)
└── README.md
```

## Mesh Files (Hosted on Google Drive)

Due to GitHub's file size limits, ALL STL mesh files are hosted on Google Drive:

**Required Downloads (All 9 Files):**
- `Base.stl` (28.5 MB)
- `back_left_wheel.stl` (103.4 MB)
- `back_right_wheel.stl` (95.3 MB)
- `front_left_wheel.stl` (95.3 MB)
- `front_right_wheel.stl` (103.4 MB)
- `lidar.stl` (154.2 MB)
- `camera.stl` (2.5 MB)
- `battery.stl` (343 KB)
- `motherboard.stl` (15.3 MB)

**Download Link:**  
📁 [Download All STL Files](https://drive.google.com/drive/u/1/folders/1DeFgrq8-kZised_OcWbh--EWJf4TB6_f)

## Setup Instructions

### Prerequisites

- **ROS 2** (Foxy, Galactic, Humble, or later)
- **RViz2** 
- **urdf_tutorial** package (for display launch file)

Install required packages:
```bash
sudo apt update
sudo apt install ros-<your-ros-distro>-urdf-tutorial ros-<your-ros-distro>-rviz2
```

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Eman-Gon/autonomous-rover-urdf.git
   cd autonomous-rover-urdf
   ```

2. **Download and place mesh files:**
   - Download ALL 9 STL files from the [Google Drive link](https://drive.google.com/drive/u/1/folders/1DeFgrq8-kZised_OcWbh--EWJf4TB6_f)
   - Create a `meshes/` directory in your project folder
   - Place all downloaded STL files in the `meshes/` directory
   - Verify you have all 9 STL files:
     ```bash
     ls meshes/
     # Should show: Base.stl, back_left_wheel.stl, back_right_wheel.stl,
     # front_left_wheel.stl, front_right_wheel.stl, lidar.stl, 
     # camera.stl, battery.stl, motherboard.stl
     ```

3. **Set up ROS workspace (if needed):**
   ```bash
   mkdir -p ~/rover_ws/src
   cd ~/rover_ws/src
   cp -r /path/to/autonomous-rover-urdf .
   cd ~/rover_ws
   colcon build
   source install/setup.bash
   ```

## Running the Simulation

### Method 1: Using urdf_tutorial (Recommended)

```bash
# Source ROS 2
source /opt/ros/<your-ros-distro>/setup.bash

# Launch RViz2 with the rover model
ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/urdf/rover.urdf
```

### Method 2: Manual RViz2 Launch

1. **Start ROS 2 core and robot state publisher:**
   ```bash
   # Terminal 1 - Load URDF and start robot state publisher
   ros2 param set robot_state_publisher robot_description -t urdf/rover.urdf
   ros2 run robot_state_publisher robot_state_publisher
   ```

2. **Launch RViz2:**
   ```bash
   # Terminal 2 - Start RViz2
   ros2 run rviz2 rviz2
   ```

3. **Configure RViz2:**
   - Click **"Add"** → **"RobotModel"**
   - Set **"Robot Description"** topic to `/robot_description`
   - Set **"Fixed Frame"** to `base_link`

### Method 3: Docker Setup (For Complex Environments)

If you're using Docker or need a containerized environment:

```bash
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
source /opt/ros/humble/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/rover.urdf
```

## Troubleshooting

### Common Issues

**1. Missing STL files error:**
```
Error: Could not find file [meshes/lidar.stl]
```
**Solution:** Download ALL 9 STL files from Google Drive and place in `meshes/` folder. None of the mesh files are included in the GitHub repository.

**2. RViz2 shows empty or broken model:**
- Verify all 9 STL files are present in `meshes/`
- Check file permissions: `chmod 644 meshes/*.stl`
- Ensure URDF file paths match actual file locations

**3. Docker GUI issues (Linux):**
```bash
xhost +local:docker  # Allow Docker to access display
```

**4. ROS 2 package not found:**
```bash
# Make sure urdf_tutorial is installed
sudo apt install ros-<distro>-urdf_tutorial
```

### Verification Commands

```bash
# Check if all mesh files are present
ls -la meshes/
# Should show 9 .stl files

# Validate URDF syntax
ros2 run urdf_tutorial check_urdf urdf/rover.urdf

# Test robot state publisher
ros2 run robot_state_publisher robot_state_publisher --urdf urdf/rover.urdf
```

## Robot Components

| Component | File | Description |
|-----------|------|-------------|
| Base Chassis | `Base.stl` | Main robot frame |
| Front Left Wheel | `front_left_wheel.stl` | Mecanum wheel assembly |
| Front Right Wheel | `front_right_wheel.stl` | Mecanum wheel assembly |
| Back Left Wheel | `back_left_wheel.stl` | Mecanum wheel assembly |
| Back Right Wheel | `back_right_wheel.stl` | Mecanum wheel assembly |
| LiDAR | `lidar.stl` | RPLIDAR sensor |
| Camera | `camera.stl` | Vision sensor |
| Battery | `battery.stl` | Power system |
| Motherboard | `motherboard.stl` | Compute unit |

## Technical Specifications

- **Total Links:** 9 (base + 4 wheels + 4 sensors/components)
- **Joint Types:** Continuous (wheels), Fixed (sensors)
- **Scale Factor:** 0.001 (STL files in millimeters, converted to meters)
- **Coordinate Frame:** ROS standard (X-forward, Y-left, Z-up)

## Built With

- **ROS 2** - Robot Operating System
- **URDF** - Unified Robot Description Format  
- **RViz2** - 3D visualization tool
- **Blender** - 3D modeling for STL files
- **GoBilda** - Hardware platform

## Contributing

This project was developed by **Emanuel Gonzalez** as part of a senior project at Cal Poly, San Luis Obispo. For questions or contributions, please open an issue or pull request.

Troubleshooting
Common Issues and Solutions
1. Package Not Found Error
Package 'urdf_tutorial' not found
Solution: Install the missing package in Docker:
bashapt update
apt install ros-humble-urdf-tutorial -y
2. GUI/Display Issues (macOS with Docker)
qt.qpa.xcb: could not connect to display host.docker.internal:0
Solution: Use alternative commands without GUI:
bash# Start robot state publisher (this works without GUI)
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat rover.urdf)" &

# Verify the rover is loaded correctly
ros2 topic list
Expected output should include:

/robot_description
/tf and /tf_static
All 9 rover components logged successfully

3. macOS X11 Display Setup
For GUI support on macOS:
bash# Install XQuartz (if not already installed)
brew install --cask xquartz

# Open XQuartz and enable network connections:
# XQuartz → Preferences → Security → "Allow connections from network clients"

# Allow Docker access
xhost +localhost

# Run Docker with X11 forwarding
docker run -it --rm \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/workspace" \
  osrf/ros:humble-desktop-full
4. Verifying Success Without GUI
Even if RViz2 doesn't open, you can confirm the rover URDF is working:
bash# Check robot state publisher output - should show all 9 components:
# base_link, front_left_wheel, front_right_wheel
# back_left_wheel, back_right_wheel  
# lidar, camera, battery, motherboard

# Verify mesh file references
grep -n "mesh" rover.urdf

# Check active ROS topics
ros2 topic list
5. Alternative: Local ROS Installation
If Docker GUI issues persist, consider installing ROS 2 directly:

Ubuntu/Linux: Follow ROS 2 Humble installation guide
macOS: Use robostack for conda-based ROS installation

Success Indicators

Robot state publisher logs all 9 rover components
No "file not found" errors for STL meshes
/robot_description topic is active
RViz2 opens (if GUI working) showing complete rover model

Hardware Requirements

Minimum 8GB RAM recommended for Docker + ROS + RViz
GPU acceleration helpful for 3D visualization


Installation Methods
Method 1: macOS with Conda (Recommended)
This method provides native macOS performance with working GUI - no Docker headaches!
Step 1: Install Miniforge
bash# Download and install Miniforge
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-$(uname -m).sh"
bash Miniforge3-MacOSX-$(uname -m).sh

# Accept license, use default location, say yes to initialize
# Restart terminal or run:
source ~/.zshrc
Step 2: Create ROS Environment
bash# Create new conda environment for ROS
conda create -n ros_env
conda activate ros_env

# Add robostack channels
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --add channels robostack
conda config --env --remove channels defaults
Step 3: Install ROS 2 Humble
bash# Install ROS 2 Desktop (includes RViz2) - takes 5-10 minutes
conda install ros-humble-desktop ros-humble-robot-state-publisher
Step 4: Setup Project
bash# Clone repository
git clone https://github.com/Eman-Gon/autonomous-rover-urdf.git
cd autonomous-rover-urdf

# Create meshes directory
mkdir meshes

# Download STL files from Google Drive and place in meshes/ folder
# Or copy from existing location:
# cp /path/to/existing/meshes/*.stl meshes/

# Verify all 9 files are present
ls meshes/
Step 5: Launch Rover Visualization
bash# Every time you want to use ROS, activate environment first:
conda activate ros_env

# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat rover.urdf)" &

# Launch RViz2
ros2 run rviz2 rviz2
Step 6: Configure RViz2

Set Fixed Frame: Change from "map" to base_link
Add Robot Model: Click "Add" → "RobotModel" → "OK"
Add TF Display (optional): Click "Add" → "TF" → "OK"
Zoom in: Your rover will be very small due to real-world scaling

Method 2: Docker (Linux/Advanced Users)
Prerequisites

Docker installed
X11 forwarding setup (Linux) or XQuartz (macOS)

Linux Setup
bash# Enable GUI access
xhost +local:docker

# Run container
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/workspace" \
  osrf/ros:humble-desktop-full
macOS Setup
bash# Install and configure XQuartz
brew install --cask xquartz
# Open XQuartz → Preferences → Security → "Allow connections from network clients"
xhost +localhost

# Run container
docker run -it --rm \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="$(pwd):/workspace" \
  osrf/ros:humble-desktop-full
Inside Container
bashcd /workspace
apt update && apt install ros-humble-urdf-tutorial -y
source /opt/ros/humble/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=rover.urdf
Troubleshooting
Common Issues
1. "Package 'urdf_tutorial' not found"
Solution: Install missing packages
bash# In conda environment:
conda install ros-humble-robot-state-publisher

# In Docker:
apt update && apt install ros-humble-urdf-tutorial -y
2. Robot Not Visible in RViz2
Symptoms: RViz2 opens but no robot visible, "No tf data" warnings
Solutions:

Check Fixed Frame: Must be set to base_link
Add RobotModel display: Click Add → RobotModel
Zoom in significantly: Robot is scaled to real-world size (very small)
Check robot_state_publisher:
bashros2 topic list
# Should show: /robot_description, /tf, /tf_static


3. Parameter Parsing Errors
Error: Failed to parse global arguments or RCLInvalidROSArgsError
Solution: Use simpler parameter format
bash# Instead of complex parameter passing, use:
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat rover.urdf)" &
4. Mesh Loading Issues (Spinning/Loading Symbol)
Cause: RViz2 can't find STL files
Solutions:

Verify all 9 STL files are in meshes/ directory
Check file permissions: chmod 644 meshes/*.stl
Use absolute paths in URDF if needed

5. macOS Docker GUI Issues
Error: qt.qpa.xcb: could not connect to display
Solution: Use conda installation method instead of Docker (recommended for macOS)
Verification Commands
bash# Check ROS environment
conda activate ros_env
ros2 --version

# Verify URDF syntax
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat rover.urdf)" &

# Check active topics
ros2 topic list
# Expected: /robot_description, /tf, /tf_static, /joint_states

# Check mesh files
ls meshes/
# Should show all 9 .stl files

# Test RViz2
ros2 run rviz2 rviz2
Success Indicators ✅

Robot state publisher logs all 9 rover components successfully
No "file not found" errors for STL meshes
/robot_description topic active in ros2 topic list
RViz2 opens and shows complete rover model when properly configured
All components visible: base chassis, 4 mecanum wheels, LiDAR, camera, battery, motherboard

## Author

- **Emanuel Gonzalez** - *Project Developer* - [Eman-Gon](https://github.com/Eman-Gon)
