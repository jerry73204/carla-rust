# LibCarla C Wrapper

This directory contains a C wrapper around LibCarla's C++ API, providing a C-compatible interface for the CARLA simulator client library.

## Overview

The wrapper exposes the main Client public interfaces from LibCarla 0.10.0, including:

- **Client**: Connection management, world loading, server queries
- **World**: Actor spawning, simulation control, weather management
- **Map**: Waypoint queries, spawn points, road topology
- **Actor**: Actor properties, physics control, blueprint management

## Directory Structure
```
libcarla_c/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # Documentation
├── example.c               # Example usage
├── include/carla_c/        # Public headers
│   ├── types.h             # Common types and enums
│   ├── client.h            # Client interface
│   ├── world.h             # World management
│   ├── map.h               # Map and waypoint operations
│   └── actor.h             # Actor and blueprint management
└── src/                    # Implementation
    ├── common.cpp          # Common utilities
    ├── client.cpp          # Client implementation
    ├── world.cpp           # World implementation
    ├── map.cpp             # Map implementation
    └── actor.cpp           # Actor implementation
```

## Building

### Prerequisites

- CMake 3.27.2 or higher (matching CARLA's requirement)
- C++14 compatible compiler
- CARLA simulator source code (version 0.10.0)
- Internet connection (for downloading dependencies on first build)

### Build Steps

#### Option 1: Using CMake directly (Recommended)
```bash
# Configure and build in one step
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release

# Run the example
cmake --build build --target run-example
```

#### Option 2: Using CMake presets (CMake 3.19+)
```bash
# Configure with preset
cmake --preset default

# Build
cmake --build --preset default

# Run example
cmake --build --preset example
```

#### Option 3: Traditional approach
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./bin/carla_c_example
```

Note: The first build will take longer as CMake downloads all dependencies (rpclib, boost, etc.).

### CMake Options

- `CARLA_ROOT_PATH`: Path to CARLA root directory (default: `../carla-simulator`)
- `CMAKE_BUILD_TYPE`: Build type (Debug/Release/RelWithDebInfo)
- `CMAKE_INSTALL_PREFIX`: Installation directory

### What Gets Built

1. **Dependencies** (automatically downloaded):
   - rpclib (CARLA's fork)
   - Boost (required components)
   - Other LibCarla dependencies

2. **Libraries**:
   - `libcarla-client.a/so` - LibCarla client library
   - `libcarla_c.so` - C wrapper library

3. **Example**:
   - `carla_c_example` - Example executable demonstrating usage

### Build Targets

- `all` (default) - Build everything
- `carla_c` - Build only the C wrapper library
- `carla_c_example` - Build only the example
- `run-example` - Build and run the example
- `install` - Install libraries and headers

## API Design

The C API follows these conventions:

1. **Opaque Pointers**: All C++ objects are wrapped in opaque struct pointers
2. **Error Handling**: Functions that can fail return error codes or NULL pointers
3. **Memory Management**: 
   - Objects created by the API must be freed using corresponding `_free` functions
   - Strings returned by the API must be freed using `carla_free_string`
4. **Naming Convention**: Functions are prefixed with the module name (e.g., `carla_client_`, `carla_world_`)

## Example Usage

```c
#include <carla_c/client.h>
#include <carla_c/world.h>
#include <stdio.h>

int main() {
    // Connect to CARLA server
    carla_client_t* client = carla_client_new("localhost", 2000, 0);
    if (!client) {
        printf("Failed to connect to CARLA server\n");
        return 1;
    }
    
    // Get server version
    char* version = carla_client_get_server_version(client);
    printf("Server version: %s\n", version);
    carla_free_string(version);
    
    // Get current world
    carla_world_t* world = carla_client_get_world(client);
    if (world) {
        // Get world ID
        uint64_t world_id = carla_world_get_id(world);
        printf("World ID: %lu\n", world_id);
        
        carla_world_free(world);
    }
    
    carla_client_free(client);
    return 0;
}
```

## Implementation Status

Currently implemented:
- ✅ Client connection and world management
- ✅ World actor spawning and queries
- ✅ Map waypoint operations
- ✅ Actor physics and properties
- ✅ Blueprint library access

Not yet implemented:
- ❌ Sensor data callbacks
- ❌ Vehicle-specific controls
- ❌ Traffic manager integration
- ❌ ROS2 native support
- ❌ OpenDRIVE generation

## Notes

- This wrapper is designed for CARLA 0.10.0 and may need updates for other versions
- Some features from the C++ API may not be directly exposable in C
- Thread safety depends on the underlying LibCarla implementation