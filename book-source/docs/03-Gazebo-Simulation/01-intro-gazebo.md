---
sidebar_position: 2
title: "Lesson 1: Introduction to Gazebo"
description: "Learn Gazebo physics simulation fundamentals and explore the physics engine landscape"
---

# Lesson 1: Introduction to Gazebo (45 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Successfully launch Gazebo 11+ and understand the empty world structure
2. Explain the role of physics engines in robot simulation
3. Compare ODE, Bullet, and DART physics engines and their trade-offs
4. Observe and describe how gravity, collisions, and friction work in simulation
5. Troubleshoot common Gazebo startup issues

---

## Prerequisites

- Chapter 1 completed (comfortable with ROS 2 and terminal)
- Ubuntu 22.04 LTS with ROS 2 Humble installed
- Gazebo 11+ installed: `sudo apt install gazebo`

**Verify installation**:
```bash
gazebo --version
# Expected output: Gazebo multi-robot simulator, version 11.x.x
```

---

## Core Concepts (7 Total)

### 1. Gazebo Ecosystem
Gazebo is a modular 3D physics simulation platform for robotics. It consists of:
- **Gazebo Server** (`gzserver`): Physics engine, simulation, sensor plugins
- **Gazebo Client** (`gzclient`): 3D visualization and GUI
- **Gazebo Plugins**: Extend functionality (cameras, LiDAR, IMU, joint control)
- **ROS 2 Bridge** (`gazebo_ros`): Connect simulation to ROS 2 topics and services

### 2. Physics Engines
A physics engine calculates realistic motion, collisions, and forces. Gazebo supports three:

| Engine | Speed | Accuracy | Stability | Best For |
|--------|-------|----------|-----------|----------|
| **ODE** (Default) | Fast | Good | Good | Real-time robotics, wheels and joints |
| **Bullet** | Faster | Better | Excellent | Complex collisions, soft bodies |
| **DART** | Slower | Excellent | Best | High-precision, humanoid motion |

### 3. World Structure
A Gazebo world contains:
- **Ground plane**: Default floor collision surface
- **Physics parameters**: Gravity (9.81 m/s²), friction, damping
- **Models**: Robots and objects (described in URDF or SDF)
- **Lights**: Ambient, point, directional lights for visualization
- **Sensors**: Virtual cameras, LiDAR, IMU publishing ROS 2 topics

### 4. Simulation Loop
Every simulation step:
1. Update physics (forces, collisions, gravity)
2. Update sensors (read cameras, calculate ranges)
3. Publish ROS 2 messages (sensor data, joint states)
4. Render visualization (update camera view)
5. Apply new commands (actuator commands from ROS 2)

Typical rate: **1000 Hz** (1 millisecond per step)

### 5. Physics Plugins
Gazebo plugins add realism:
- **Joint controllers**: Command joint angles/velocities
- **Sensor plugins**: Camera, LiDAR, IMU simulation
- **Contact plugins**: Detect collisions and friction
- **ROS 2 bridges**: Translate between Gazebo and ROS 2 topics

### 6. Visualization vs. Collision Geometry
- **Visual geometry** (what you see in RViz): Represents appearance
- **Collision geometry** (what physics engine uses): Determines collisions
- Example: A robot with a camera sensor might have a sphere for collision but a detailed model for visuals

### 7. SDF vs. URDF
- **URDF** (Unified Robot Description Format): Describes robot structure (links, joints)
- **SDF** (Simulation Description Format): Describes simulation world (physics, plugins)
- You'll write URDF for robots; Gazebo converts to SDF internally

---

## Gazebo Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                   Gazebo 11+                        │
├─────────────────────────────────────────────────────┤
│ Physics Engine (ODE/Bullet/DART)                   │
│  ├─ Gravity & forces                               │
│  ├─ Collision detection                            │
│  └─ Friction & damping                             │
├─────────────────────────────────────────────────────┤
│ Plugins                                             │
│  ├─ Joint Controllers (velocity, effort, position) │
│  ├─ Sensor Plugins (camera, LiDAR, IMU)           │
│  └─ ROS 2 Bridges (gazebo_ros)                    │
├─────────────────────────────────────────────────────┤
│ Visualization (OpenGL)                             │
│  ├─ 3D renderer                                    │
│  └─ Gazebo GUI (gzclient)                         │
└─────────────────────────────────────────────────────┘
         ↓ ROS 2 Topics & Services ↓
    ┌─────────────────────────┐
    │   ROS 2 Nodes           │
    │ (control, perception)   │
    └─────────────────────────┘
```

---

## Layer 1: Manual Foundation - Exploring Gazebo GUI

Before writing code, let's explore Gazebo interactively.

### Exercise 1: Launch Gazebo Empty World

Open a terminal and run:

```bash
# Terminal 1: Start Gazebo
gazebo

# Or explicitly start the server and client separately:
# gzserver                    # Physics + simulation
# gazebo                      # GUI (in separate terminal)
```

**Expected output**: A window appears showing:
- Ground plane (light gray grid)
- Sky (blue background)
- Grid pattern (helps with scale)
- Toolbar with model icons, view controls

**What's happening**: Gazebo is running the simulation at 1000 Hz, even with no robots present. Gravity is actively pulling at objects (though there are none yet).

### Exercise 2: Add Objects to the World

In the **Insert** menu (top toolbar):
1. Click **Insert Tab** (left sidebar)
2. Select **Simple Shapes** folder
3. Drag a **Box** into the world
4. Observe: The box falls due to gravity and lands on the ground plane

**Expected behavior**:
- Box appears at origin (0,0,0)
- Falls straight down
- Bounces slightly when hitting the ground
- Settles on the ground plane

### Exercise 3: Adjust View and Inspect Physics

In the **View** menu:
- Click **Top View**: See the world from above
- Click **Front View**: Side perspective
- Right-click + drag: Rotate camera view

In the **World** menu:
- **Physics**: View current physics engine and settings (likely ODE with gravity 9.81)
- **Statistics**: See simulation statistics (FPS, iterations per second)

---

## Physics Engine Comparison

### ODE (Open Dynamics Engine)
**Use when**: Building typical mobile robots, manipulators, wheeled systems

**Pros**:
- Fast and stable for most robotics applications
- Good wheel-ground friction simulation
- Handles joint constraints well

**Cons**:
- Can be unstable with extreme forces
- Contact modeling less accurate than alternatives

**Typical parameters**:
```
gravity: 0 0 -9.81
solver: quick
iterations: 50  # Higher = more stable but slower
```

### Bullet Physics
**Use when**: Complex collisions, soft bodies, or better contact modeling

**Pros**:
- Excellent collision detection
- Handles multiple contacts better
- Good for human-robot interaction (soft contact)

**Cons**:
- Slightly slower than ODE
- Different tuning approach (different stability characteristics)

**Typical parameters**:
```
gravity: 0 0 -9.81
solver: sequential impulse
contact_surface_layer: 0.0001
```

### DART (Dynamic Animation and Robotics Toolkit)
**Use when**: Humanoid robots, precise motion capture, complex constraint systems

**Pros**:
- Best accuracy for humanoid robotics
- Excellent constraint handling
- Good energy conservation

**Cons**:
- Slower than ODE/Bullet
- More complex tuning
- Less commonly used in ROS ecosystem

**Typical parameters**:
```
gravity: 0 0 -9.81
friction_coeff: 1.0
restitution_coeff: 0.5
```

---

## Code Example: Verifying Gazebo Installation

Create a shell script to verify your Gazebo installation:

**File**: `examples/chapter-2-gazebo/lesson-01/verify-gazebo-install.sh`

```bash
#!/bin/bash
# Gazebo 11+ installation verification script
# Simulation environment: Gazebo 11+

echo "=== Gazebo Installation Verification ==="

# Check Gazebo version
echo -e "\n1. Gazebo version:"
gazebo --version || echo "ERROR: gazebo not found"

# Check ROS 2 integration
echo -e "\n2. ROS 2 Gazebo integration:"
dpkg -l | grep gazebo-ros || echo "WARNING: gazebo-ros package not found"

# Check Gazebo plugins
echo -e "\n3. Gazebo plugins directory:"
ls /opt/ros/humble/lib/gazebo* 2>/dev/null | head -5 || echo "WARNING: plugins not found"

# Verify models
echo -e "\n4. Gazebo models:"
[ -d ~/.gazebo/models ] && echo "Models directory exists" || echo "WARNING: ~/.gazebo/models not found"

# Test empty world launch
echo -e "\n5. Testing Gazebo launch (10-second test):"
timeout 10 gazebo -s libgazebo_ros_init.so --verbose 2>&1 | head -20
echo "Gazebo launch test completed"

echo -e "\n=== Verification Complete ==="
```

**Expected output**:
```
=== Gazebo Installation Verification ===

1. Gazebo version:
Gazebo multi-robot simulator, version 11.11.0

2. ROS 2 Gazebo integration:
ros-humble-gazebo-ros-pkgs/jammy ...

3. Gazebo plugins directory:
/opt/ros/humble/lib/gazebo_ros_camera_plugin.so
/opt/ros/humble/lib/gazebo_ros_lidar.so
...

4. Gazebo models:
Models directory exists

5. Testing Gazebo launch...
[Gazebo] [INFO] starting Gazebo [...]
Gazebo launch test completed

=== Verification Complete ===
```

---

## Layer 2: AI Collaboration Notes

**Prompts to explore with Claude**:

💬 **Prompt 1**: "Why does Gazebo have separate physics engines? When would I choose Bullet over ODE?"

Claude will explain:
- Different physics problems have different solvers
- ODE is fine for wheels and joints (most robots)
- Bullet shines with complex collisions and soft contacts
- DART excels at humanoid motion precision

💬 **Prompt 2**: "If I increase the gravity value from 9.81 to 20 m/s², what happens to my robot's behavior?"

Claude will note:
- Objects fall faster and with more impact force
- Heavier collisions (more energy dissipated)
- Joints feel more "weighted down"
- Useful for simulating high-gravity planets or stress-testing stability

💬 **Prompt 3**: "What's the difference between visual geometry and collision geometry? Why have both?"

Claude will explain:
- Visual = what you see (detailed 3D mesh)
- Collision = what physics engine uses (simpler shapes)
- Reason: Performance (complex meshes slow down collision detection)
- Example: A gripper might have a pretty mesh but use simple boxes for collision

---

## Troubleshooting

### Error: "gazebo: command not found"
**Cause**: Gazebo not installed
```bash
sudo apt update
sudo apt install gazebo
```

### Error: "Could not load plugin..."
**Cause**: Gazebo ROS plugins missing
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Issue: Gazebo hangs at startup
**Cause**: Initialization hang (common on slow machines)
**Solution**:
```bash
# Kill any stuck processes
killall gzserver gzclient
# Try with reduced rendering
gazebo --profile viz
```

### Issue: "No display" error (headless mode)
**Cause**: Running over SSH without X11 forwarding
**Solution**:
```bash
# Start server only (no GUI)
gzserver

# In a separate terminal with display:
gazebo  # Connect to running server
```

### Issue: Physics simulation very slow
**Cause**: Graphics rendering bottleneck
**Solution**:
```bash
# Run server-only mode
gzserver --verbose

# Verify update rate with: rostopic hz /gazebo/stats
```

### Issue: Objects falling through ground plane
**Cause**: Collision group configuration or ground not properly spawned
**Solution**:
- Verify ground plane exists in world file
- Check object collision geometry is defined

### Issue: Keyboard input not registering
**Cause**: Gazebo window focus lost
**Solution**: Click the 3D viewport to ensure window focus

### Issue: "Address already in use" error
**Cause**: gzserver still running from previous session
**Solution**:
```bash
# Kill all Gazebo processes
killall -9 gzserver gzclient

# Verify:
ps aux | grep gazebo  # Should be empty
```

### Issue: Slow frame rate (less than 30 fps)
**Cause**: Rendering too complex or system underpowered
**Solution**:
- Reduce number of objects in world
- Disable visual plugins not needed
- Upgrade GPU or use headless mode

### Issue: Multiple Gazebo instances interfere
**Cause**: ROS_DOMAIN_ID or port conflicts
**Solution**:
```bash
# Use unique domain IDs
export ROS_DOMAIN_ID=1
gzserver

# In another terminal
export ROS_DOMAIN_ID=2
gzserver
```

---

## Self-Assessment Checklist

- [ ] I can launch Gazebo and see the empty world with ground plane
- [ ] I understand the three physics engines (ODE, Bullet, DART) and when to use each
- [ ] I can explain what a physics engine does in a robot simulation
- [ ] I've observed gravity pulling objects down and collisions working correctly
- [ ] I know how to verify Gazebo 11+ is properly installed on my system
- [ ] I can troubleshoot common Gazebo startup issues

---

## Key Takeaways

1. **Gazebo is your robot laboratory**: Run unlimited simulations, learn without constraints
2. **Physics engines matter**: Choose the right one for your robot type
3. **Gravity is always on**: Default 9.81 m/s²; affects all motion
4. **Collisions are automatic**: Ground plane and objects interact realistically
5. **Simulation-first approach**: Master simulation before touching real hardware

---

## Next Lesson

Ready to build robots? In Lesson 2, you'll create your first URDF robot model and spawn it in Gazebo.

[Go to Lesson 2: URDF Basics →](./lesson-02-urdf-basics.md)

---

**Lesson Status**: Complete with physics engine overview, hands-on Gazebo exploration, verification script, and troubleshooting guide.

**Duration**: 45 minutes (including manual exploration)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-01/verify-gazebo-install.sh`
- `examples/chapter-2-gazebo/lesson-01/physics-engines.txt` (reference table)

**Concepts Covered**: 7 (Gazebo ecosystem, physics engines, world structure, simulation loop, plugins, geometry types, SDF vs URDF)
