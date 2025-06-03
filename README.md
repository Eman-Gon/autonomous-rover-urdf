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

## Author

- **Emanuel Gonzalez** - *Project Developer* - [Eman-Gon](https://github.com/Eman-Gon)
