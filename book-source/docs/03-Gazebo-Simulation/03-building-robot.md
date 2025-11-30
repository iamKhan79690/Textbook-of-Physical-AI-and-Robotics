---
sidebar_position: 4
title: "Lesson 3: Building Your First Robot"
description: "Design and build a differential drive mobile robot, spawn in Gazebo, and command motion"
---

# Lesson 3: Building Your First Robot (75 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Understand differential drive kinematics and design principles
2. Design a mobile robot with base link and wheels
3. Create complete URDF for a mobile robot with all physics properties
4. Spawn the robot in Gazebo and verify physical behavior
5. Command robot motion via ROS 2 Twist messages

---

## Prerequisites

- Lesson 2 completed (URDF basics)
- Understanding of robot kinematics basics

---

## Core Concepts (8 Total)

### 1. Mobile Base Design
A mobile base consists of:
- **Base link**: Main robot body (box shape, center of mass)
- **Wheels**: Two or more actuated links for motion
- **Caster wheel** (optional): Free-rolling wheel for balance

Typical mobile robot geometry:
```
        front
    ┌────────────┐
    │  base_link │
    └────────────┘
   left_wheel right_wheel
```

### 2. Differential Drive Kinematics
Two wheels, independently controlled. The robot moves by:
- **Both wheels forward** → robot moves forward
- **Left wheel faster** → robot turns right
- **Opposite wheel speeds** → robot spins in place

Kinematics equations:
```
v_linear = (v_left + v_right) / 2
v_angular = (v_right - v_left) / wheelbase
```

Where:
- `v_left`, `v_right` = wheel velocities
- `wheelbase` = distance between wheels
- `v_linear` = forward speed
- `v_angular` = rotation speed

### 3. Wheel Configuration
**Wheel parameters**:
- **Radius**: Affects distance traveled per rotation (0.05 m typical)
- **Width**: Affects traction and friction
- **Mass**: Should be 5-10% of total robot mass
- **Position**: Must be symmetric for balanced drive

### 4. Base Link Positioning
The `base_link` should be:
- **At center of mass**: For realistic physics
- **Low on the robot**: Improves stability
- **Surrounded by collision geometry**: Prevents falling through ground

### 5. Caster Wheel Design
A free-rolling wheel mounted on a swivel:
- Prevents tipping when moving in arcs
- Reduces friction compared to skid steering
- Common on small mobile robots (TurtleBot, differential drive platforms)

### 6. Joint Controller Configuration
Wheels need effort-controlled joints (motors):
```xml
<joint type="continuous">  <!-- Unlimited rotation -->
  <limit effort="10" velocity="2.0"/>  <!-- Max motor torque and speed -->
</joint>
```

### 7. Physics for Mobile Robots
Key parameters:
- **Friction**: Wheels need high friction (0.8-1.0) for traction
- **Damping**: Wheels benefit from low damping (0.01-0.05)
- **Contact**: Ground contact must be rock-solid (no hovering)

### 8. Spawning in Gazebo
Robots are spawned via ROS 2 service or launch file:
```bash
# Via command line
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'robot1', xml: '<robot>...'}"

# Via launch file
IncludeLaunchDescription(PythonLaunchDescriptionSource(['gazebo_ros', 'launch', 'gazebo.launch.py']))
```

---

## Layer 1: Manual Exercise - Build and Spawn

### Exercise: Create Differential Drive Robot URDF

1. **Design the geometry**:
   - Base: 0.2m length × 0.15m width × 0.1m height
   - Wheels: 0.05m radius, mounted at ±0.075m from center Y
   - Caster wheel: 0.01m radius at front

2. **Create URDF** with:
   - `base_link` (box collision shape)
   - `left_wheel` and `right_wheel` (cylinder shapes)
   - `caster_wheel` (sphere)
   - Joints connecting all parts

3. **Set physics parameters**:
   - Base mass: 5 kg
   - Each wheel mass: 0.3 kg
   - Caster mass: 0.1 kg
   - Friction: 0.9 (high for traction)

4. **Visualize in RViz** to verify structure

---

## Code Example: Complete Differential Drive Robot

**File**: `examples/chapter-2-gazebo/lesson-03/differential-drive-robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot">
  <!-- Simulation environment: Gazebo 11+ -->

  <!-- Ground Link (fixed to world) -->
  <link name="world"/>

  <!-- Base Link (main robot body) -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.4"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
      <material name="base_color">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="wheel_color">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="wheel_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Caster Wheel -->
  <link name="caster_wheel">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <material name="caster_color">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint: Base to Left Wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.075 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="2.0"/>
    <dynamics damping="0.01" friction="0.0"/>
  </joint>

  <!-- Joint: Base to Right Wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.075 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="2.0"/>
    <dynamics damping="0.01" friction="0.0"/>
  </joint>

  <!-- Joint: Base to Caster Wheel -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.09 0 -0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

**Expected output**: A robot with body and three wheels, ready to spawn in Gazebo.

---

## Code Example: Spawn in Gazebo (Python)

**File**: `examples/chapter-2-gazebo/lesson-03/spawn_robot.py`

```python
#!/usr/bin/env python3
"""
Spawn differential drive robot in Gazebo
Simulation environment: Gazebo 11+
"""

import rclpy
from gazebo_msgs.srv import SpawnEntity
import os

def spawn_robot():
    rclpy.init()
    node = rclpy.create_node('spawn_robot')

    # Read URDF file
    urdf_path = os.path.expanduser('~/differential-drive-robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_xml = f.read()

    # Create spawn client
    client = node.create_client(SpawnEntity, '/spawn_entity')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn_entity service...')

    # Prepare request
    request = SpawnEntity.Request()
    request.name = 'my_robot'
    request.xml = robot_xml
    request.robot_namespace = '/'
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.5  # Spawn above ground

    # Call service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result().success:
        node.get_logger().info('Robot spawned successfully!')
    else:
        node.get_logger().error('Failed to spawn robot')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    spawn_robot()
```

**Usage**:
```bash
# Terminal 1: Start Gazebo
gazebo

# Terminal 2: Run spawn script
python3 spawn_robot.py
```

---

## Layer 2: AI Collaboration Notes

💬 **Prompt 1**: "Why do we use 'continuous' joints for wheels instead of 'revolute'?"

Claude explains:
- `revolute` joints have min/max limits (typically ±π)
- `continuous` joints allow unlimited rotation (like real wheels)
- Real wheels can spin indefinitely; limits don't make sense
- Use `continuous` whenever you want unrestricted motion

💬 **Prompt 2**: "What's the difference between friction and damping? My robot slides too much—which one should I tune?"

Claude explains:
- **Friction**: Resistance when sliding across a surface (wheel-ground)
- **Damping**: Internal resistance in the joint itself (motor bearing friction)
- Sliding wheels? Increase friction coefficient (0.9-1.0)
- Spinning too freely? Increase damping (0.01-0.1)

---

## Troubleshooting

### Problem: Robot doesn't move when wheels spin
**Cause**: Friction too low or wheel mass too high
**Solution**:
```xml
<!-- Increase friction -->
<friction value="1.0"/>  <!-- In Gazebo world file -->

<!-- Check wheel mass ratio -->
<!-- Wheel mass should be 5-10% of base mass -->
```

### Problem: Robot tips over easily
**Cause**: Wheels positioned too high or wheelbase too narrow
**Solution**:
- Lower wheel position (mount flush with base bottom)
- Increase wheelbase (distance between left/right wheels)
- Add caster wheel for support

### Problem: Caster wheel gets stuck or wobbles
**Cause**: Caster mounting too rigid or sphere radius wrong
**Solution**:
```xml
<!-- Use small caster wheel radius (~1-2 cm) -->
<sphere radius="0.01"/>
<!-- Use prismatic/continuous joint for caster, not fixed -->
```

### Problem: "URDF model 'robot' doesn't have inertial element in root link"
**Cause**: base_link missing inertial properties
**Solution**: Ensure base_link has `<inertial>` with mass and inertia

### Problem: Wheels penetrate ground plane
**Cause**: Wheel origin too low or ground contact issue
**Solution**:
```xml
<!-- Wheel origin should be at wheel center -->
<!-- If base is at z=0, wheels at z should be -radius -->
<origin xyz="0 0.075 -0.05"/>  <!-- -0.05 = -0.1/2 (base height/2) -->
```

### Problem: Robot spins in circles instead of moving straight
**Cause**: Wheel masses or frictions unequal
**Solution**: Make left/right wheels identical; check both have same mass and friction

### Problem: Can't spawn robot—"Entity already exists"
**Cause**: Robot from previous spawn still in world
**Solution**:
```bash
# Delete previous robot
ros2 service call /delete_entity gazebo_msgs/DeleteEntity "{name: 'my_robot'}"

# Then spawn new one
```

---

## Self-Assessment Checklist

- [ ] I can explain differential drive kinematics (how left/right wheel speeds create motion)
- [ ] I've created a complete URDF for a mobile robot with wheels
- [ ] My robot spawns in Gazebo without errors
- [ ] The robot sits on the ground plane (wheels touch, base doesn't penetrate)
- [ ] I understand how to calculate/set appropriate wheel mass and friction
- [ ] I can visualize the robot in RViz and see all links/joints correctly

---

## Key Takeaways

1. **Differential drive is fundamental**: Two wheels enable all robot motion
2. **Mass distribution matters**: Wheel mass affects stability and control
3. **Friction is critical**: High friction (0.8-1.0) is essential for wheeled robots
4. **Caster wheels improve stability**: Free-rolling wheel prevents tipping
5. **URDF + Gazebo = simulation**: Physics respects your robot design

---

## Next Lesson

In Lesson 4, you'll fine-tune physics parameters to make your robot behave exactly as designed.

[Go to Lesson 4: Physics Simulation Tuning →](./lesson-04-physics-simulation.md)

---

**Lesson Status**: Complete with differential drive design, full URDF example, spawn script, and troubleshooting.

**Duration**: 75 minutes (including RViz verification and Gazebo spawning)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-03/differential-drive-robot.urdf`
- `examples/chapter-2-gazebo/lesson-03/spawn_robot.py`

**Concepts Covered**: 8 (mobile base design, differential drive kinematics, wheel configuration, base link positioning, caster wheels, joint controllers, physics for mobile robots, spawning in Gazebo)
