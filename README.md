# ME48B Library Book Organization Robot

![Library Simulation](docs/images/gazebo_01.png)

## Project Overview

This project implements an autonomous mobile robot for organizing books in a library environment. The robot navigates through the library, picks up books from a designated drop-off station, reads QR codes to identify book categories, and places them on the appropriate shelves.

**Course:** ME48B - Introduction to Mobile Robotics

## Features

- **Autonomous Navigation:** Uses AMCL for localization and move_base for path planning
- **QR Code Recognition:** Camera-based book identification system
- **Obstacle Avoidance:** LiDAR and ultrasonic sensors for safe navigation
- **Forklift Mechanism:** Custom mechanism for book pickup and placement

## System Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Python 3.8+

## Installation

```bash
# Clone to catkin workspace
cd ~/catkin_ws/src
git clone <repository_url> me48b_library_robot

# Install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make

# Source
source devel/setup.bash
```

## Quick Start

### Launch the Library Environment
```bash
roslaunch me48b_library_robot bookstore.launch gui:=true
```

### Run with Robot (after completing robot setup)
```bash
roslaunch me48b_library_robot full_system.launch
```

## Project Structure

```
me48b_library_robot/
├── launch/          # ROS launch files
├── models/          # Gazebo models (shelves, tables, books)
├── worlds/          # Gazebo world files
├── maps/            # Navigation maps
├── config/          # Configuration files
├── scripts/         # Python nodes
├── urdf/            # Robot description files
└── rviz/            # RViz configurations
```

## Documentation

- [Project Plan](PROJECT_PLAN.md) - Detailed project roadmap
- [Tasks](TASKS.md) - Development checklist

## Robot Initial Position

Recommended spawn position near the service area: `(0.5, 1.0, 0.0)`

## License

MIT License
