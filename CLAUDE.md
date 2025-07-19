# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

UiAbot (University of Agder Autonomous Mobile Robot) is a ROS 2-based autonomous mobile robot platform designed for research and STEM education. This repository contains the project documentation and supporting Python modules.

## Build and Development Commands

### Documentation
```bash
# Build HTML documentation
make html

# Clean build artifacts
make clean

# Live documentation preview (if needed)
sphinx-autobuild docs/ docs/_build/html
```

### Testing
```bash
# Run all tests
pytest jl/tests/

# Run specific test file
pytest jl/tests/test_hw.py
```

### Python Development
```bash
# Install dependencies
pip install -r requirements.txt

# The jl package can be imported directly in Python scripts
# Example: from jl import hw, lw, mc, mw
```

## Architecture and Structure

### Repository Organization
- `/docs/` - Sphinx documentation source
  - `/src/` - RST documentation files organized by topic
  - `/fig/` - Diagrams and figures
  - `/software_documentation/` - API documentation for ROS packages
- `/jl/` - Python module with hardware/software interfaces
  - `hw.py`, `lw.py`, `mc.py`, `mw.py` - Core modules
  - `/tests/` - Pytest test suite

### Documentation Topics
The documentation follows the robot development lifecycle:
1. **Overview & Installation** - System introduction and setup
2. **Motion Control** - ODrive motor controllers, odometry, kinematics
3. **Perception** - LiDAR, IMU (BNO055), sensor fusion
4. **SLAM** - Localization and mapping implementation
5. **Navigation** - Nav2 stack integration and autonomous navigation

### ROS 2 Integration
While this repository contains documentation, it references several ROS 2 packages:
- `uiabot` - Main robot package
- `odrive_ros2` - Motor controller interface  
- `bno055_i2c_ros2` - IMU driver
- `uiabot_*` - Various robot-specific packages (control, IMU TF, etc.)

The actual ROS 2 packages are maintained separately and documented here.

### Key Technical Context
- **Platform**: Ubuntu 20.04/22.04 with ROS 2 Humble
- **Hardware**: NVIDIA Jetson Orin Nano, LiDAR, IMU, wheel encoders
- **Navigation**: Uses Nav2 stack for autonomous navigation
- **SLAM**: Implements simultaneous localization and mapping

## Development Notes

When modifying documentation:
- Use reStructuredText (RST) format
- Place figures in `/docs/fig/`
- Update the index.rst if adding new sections
- The documentation is published to GitHub Pages

When working with the Python modules:
- Run tests after any changes to jl modules
- Follow existing code style and patterns
- Tests should cover new functionality