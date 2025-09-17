# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

PX4 uses a CMake-based build system with a top-level Makefile that wraps CMake commands for convenience.

### Key Build Commands

```bash
# Build for simulation (SITL - Software In The Loop)
make px4_sitl_default

# Build for specific hardware targets (examples)
make px4_fmu-v6x_default
make px4_fmu-v5_default
make cuav_nora_default

# Build and upload firmware
make px4_fmu-v6x_default upload

# Run tests
make tests
make px4_sitl_test

# Format code
make format

# Check code style
make check_format

# Clean build
make clean

# List all available build targets
make list_config_targets
```

### Board Configuration Files

Board configurations are defined in `.px4board` files located in `boards/<vendor>/<board>/`. These files specify:
- Target platform (nuttx, posix, qurt, ros2)
- Enabled modules and drivers
- Hardware-specific settings

## Architecture Overview

### Core Directory Structure

- `src/modules/` - Core flight control modules (commander, navigator, sensors, etc.)
- `src/drivers/` - Hardware drivers (IMU, GPS, barometer, etc.)
- `src/lib/` - Shared libraries and utilities
- `src/systemcmds/` - System commands and utilities
- `platforms/` - Platform-specific code (NuttX, POSIX, QURT, ROS2)
- `boards/` - Board-specific configurations and code
- `ROMFS/` - Root filesystem configurations and scripts

### Key Modules

- **commander** - System state management and mode switching
- **navigator** - Mission planning and waypoint navigation
- **sensors** - Sensor fusion and data processing
- **mc_pos_control** - Multicopter position controller
- **fw_pos_control** - Fixed-wing position controller
- **vtol_att_control** - VTOL attitude controller
- **mavlink** - MAVLink protocol implementation
- **logger** - Flight data logging
- **land_detector** - Landing state detection
- **control_allocator** - Control surface/motor allocation

### Platform Support

- **nuttx** - Real-time OS for flight controllers
- **posix** - SITL simulation and Linux companion computers
- **qurt** - Qualcomm Hexagon DSP platform
- **ros2** - ROS 2 integration layer

## Development Workflow

### Python Dependencies

Install required Python packages:
```bash
pip install -r Tools/setup/requirements.txt
```

### Testing

Run unit tests:
```bash
make tests
```

Run SITL simulation tests:
```bash
make px4_sitl_default
make rostest
```

### Code Style

PX4 uses astyle for C/C++ formatting:
```bash
# Check formatting
make check_format

# Auto-format code
make format
```

### Debugging

For SITL debugging:
```bash
make px4_sitl_default
# In another terminal:
gdb build/px4_sitl_default/bin/px4
```

## Build Types and Sanitizers

- **Debug** - Debug symbols, no optimization
- **Release** - Full optimization, no debug symbols
- **RelWithDebInfo** - Optimization with debug symbols (default for POSIX)
- **MinSizeRel** - Size optimization (default for NuttX)
- **Coverage** - Code coverage instrumentation

Enable sanitizers via environment variables:
```bash
PX4_ASAN=1 make px4_sitl_default    # Address sanitizer
PX4_MSAN=1 make px4_sitl_default    # Memory sanitizer
PX4_TSAN=1 make px4_sitl_default    # Thread sanitizer
PX4_UBSAN=1 make px4_sitl_default   # Undefined behavior sanitizer
```

## Configuration System

PX4 uses Kconfig for build-time configuration. Module configurations are defined in `module.yaml` files within each module directory.

## Message System (uORB)

PX4 uses a custom publish-subscribe messaging system called uORB (micro Object Request Broker). Message definitions are in `msg/` directory and generate C++ classes for inter-module communication.

## External Modules

External modules can be added by setting the `EXTERNAL_MODULES_LOCATION` environment variable to point to external module source code.