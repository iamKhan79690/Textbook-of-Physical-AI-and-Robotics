---
sidebar_position: 3
title: "Lesson 2: URDF Robot Modeling Basics"
description: "Learn URDF structure, create robot models with links and joints, and visualize in RViz"
---

# Lesson 2: URDF Robot Modeling Basics (60 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Understand URDF XML structure and its role in robotics
2. Define robot links with inertial, visual, and collision properties
3. Create joints that connect links and define kinematic structure
4. Write a complete URDF for a multi-link robot
5. Visualize robot models in RViz and verify kinematic correctness

---

## Prerequisites

- Lesson 1 completed (understand Gazebo basics)
- Comfortable with XML syntax
- RViz installed: `sudo apt install ros-humble-rviz2`

---

## Core Concepts (7 Total)

### 1. URDF (Unified Robot Description Format)
URDF is an XML-based standard that describes:
- **Robot structure**: Which parts (links) are connected by which joints
- **Geometry**: Visual appearance and collision shapes
- **Inertia**: Mass distribution for physics simulation
- **Sensors**: Where cameras, LiDAR, IMU are mounted
- **Actuators**: Joint controllers and power specifications

Example structure:
```xml
<robot name="my_robot">
  <link name="base_link"><!-- rigid body --></link>
  <joint name="joint1" type="revolute"><!-- connection --></joint>
</robot>
```

### 2. Links (Rigid Bodies)
A link is a rigid body with three properties:

**Inertial**: Mass and moment of inertia
```xml
<inertial>
  <mass value="2.5"/>  <!-- kg -->
  <inertia ixx="0.1" ixy="0" ixz="0"
           iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

**Visual**: 3D shape for rendering
```xml
<visual>
  <geometry><box size="0.2 0.3 0.1"/></geometry>
  <material name="base_color">
    <color rgba="0.8 0.8 0.8 1.0"/>
  </material>
</visual>
```

**Collision**: 3D shape for physics (often simpler than visual)
```xml
<collision>
  <geometry><box size="0.2 0.3 0.1"/></geometry>
</collision>
```

### 3. Joints (Connections Between Links)
A joint connects two links and defines allowed motion:

**Joint attributes**:
- **Type**: revolute (rotation), prismatic (sliding), fixed (no motion), continuous (unlimited rotation)
- **Parent/Child**: Which links this joint connects
- **Axis**: Direction of motion (e.g., Z for rotation around vertical axis)
- **Limits**: Min/max angle or range, max velocity, max effort
- **Dynamics**: Damping and friction coefficients

Example (revolute joint—rotation around Z-axis):
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Rotate around Z -->
  <limit lower="-1.57" upper="1.57"
         effort="10" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.2"/>
</joint>
```

### 4. Inertial Properties
The inertia matrix (3x3) describes mass distribution. For simple shapes:

**Box** (width×height×depth):
```
Ixx = (mass/12) × (height² + depth²)
Iyy = (mass/12) × (width² + depth²)
Izz = (mass/12) × (width² + height²)
```

**Cylinder** (radius r, length L along Z):
```
Ixx = Iyy = (mass/12) × (3×r² + L²)
Izz = (mass/2) × r²
```

Most physics engines calculate inertia automatically if you don't specify it.

### 5. Collision vs. Visual Geometry
- **Visual**: Defines appearance for humans viewing RViz. Can be complex 3D meshes
- **Collision**: Defines shape for physics engine. Should be simple (box, cylinder, sphere)
- **Why separate?**: Complex visual meshes slow down collision detection; simple collision shapes are fast

Example (gripper with detailed visual, simple collision):
```xml
<link name="gripper">
  <visual>
    <geometry>
      <mesh filename="package://gripper_mesh.dae"/>  <!-- Detailed 3D model -->
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.1"/>  <!-- Simple box for collision -->
    </geometry>
  </collision>
</link>
```

### 6. Joint Types
| Type | Motion | Use Case | Limits |
|------|--------|----------|--------|
| **revolute** | Rotation around axis | Robot joints, wheels (with limits) | Lower/upper angle in radians |
| **continuous** | Unlimited rotation | Wheels, spinners | No limits |
| **prismatic** | Linear motion along axis | Sliding grippers, elevator | Lower/upper distance |
| **fixed** | No motion | Attach sensor to link | No limits |
| **floating** | Free 6-DOF motion | Base link in air | None (simulation only) |

### 7. Frame Naming Convention
- **base_link**: The primary body of the robot (usually immobile relative to ground)
- **link1, link2, ...**: Additional bodies
- **tool_link** or **end_effector_link**: Final link (tool tip for manipulators)
- **sensor_link**: Link where a sensor is attached

Good naming is crucial for debugging ROS 2 TF transforms!

---

## URDF Anatomy: A Complete Simple Robot

Let's build a 2-link arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Link 1: Base -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
    <visual>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
      <material name="base_material">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
  </link>

  <!-- Joint 1: Base to Link 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="3.14" effort="5" velocity="1.0"/>
    <dynamics damping="0.01" friction="0.05"/>
  </joint>

  <!-- Link 2: Upper Arm -->
  <link name="link1">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0"
               iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry><box size="0.05 0.05 0.3"/></geometry>
      <material name="link_material">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.3"/></geometry>
    </collision>
  </link>

  <!-- Joint 2: Link 1 to Link 2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="3" velocity="1.0"/>
    <dynamics damping="0.01" friction="0.05"/>
  </joint>

  <!-- Link 3: Lower Arm / Tool -->
  <link name="link2">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <geometry><box size="0.04 0.04 0.2"/></geometry>
      <material name="link_material"/>
    </visual>
    <collision>
      <geometry><box size="0.04 0.04 0.2"/></geometry>
    </collision>
  </link>

</robot>
```

---

## Layer 1: Manual Exercise - Visualize in RViz

### Exercise: Load and Inspect the 2-Link Robot

1. **Create the file** `two-link-robot.urdf` with the content above
2. **Launch RViz**:
```bash
rviz2
```

3. **Load the URDF** in RViz:
   - In RViz, set **Fixed Frame** to `base_link`
   - Open **File** → **Open Config** (if you have a saved config)
   - Or manually add displays:
     - Add **RobotModel** display
     - In the **RobotModel** settings, change description source to "File"
     - Select your URDF file

4. **Observe**:
   - RViz shows the robot as a kinematic tree
   - You should see: base_link (box), joint1 (connection), link1 (box), joint2, link2 (box)
   - Frames appear as XYZ axes (red=X, green=Y, blue=Z)

5. **Inspect the transform tree**:
   - In RViz menu, select **View** → **Panels** → **TF Tree**
   - You should see: `base_link` → `link1` (via joint1) → `link2` (via joint2)

---

## Code Example: Two-Link Robot URDF

**File**: `examples/chapter-2-gazebo/lesson-02/two-link-robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="two_link_robot">
  <!-- Simulation environment: Gazebo 11+ -->

  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
    <visual>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
      <material name="base_material">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="3.14" effort="5" velocity="1.0"/>
    <dynamics damping="0.01" friction="0.05"/>
  </joint>

  <link name="link1">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0"
               iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry><box size="0.05 0.05 0.3"/></geometry>
      <material name="link_material">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.3"/></geometry>
    </collision>
  </link>

</robot>
```

**Expected output**: RViz shows a 2-link kinematic chain with proper frame transforms.

---

## Layer 2: AI Collaboration Notes

💬 **Prompt 1**: "Why do I need to define inertia? Can't I just give mass?"

Claude explains:
- Mass alone isn't enough; inertia describes where the mass is distributed
- Inertia affects rotational dynamics (how fast it spins, how it tumbles)
- Without correct inertia, robot physics will be unrealistic
- Gazebo can auto-calculate for simple shapes if you just provide mass

💬 **Prompt 2**: "What happens if I set joint limits to [0, 3.14] instead of [-3.14, 3.14]?"

Claude explains:
- First (0, 3.14): Joint can only rotate from 0 to 180 degrees (half rotation)
- Second (-3.14, 3.14): Joint can rotate ±180 degrees (full rotation except 360°)
- Limits prevent unrealistic joint configurations
- Violating limits damages real robots; simulation can handle it but physics becomes weird

---

## Troubleshooting

### Error: "RDF should have a root element named 'robot'"
**Cause**: Invalid XML structure
**Solution**: Check that your first tag is `<robot name="...">` and last is `</robot>`

### Error: "Link 'link1' not found, child link does not exist"
**Cause**: Joint references non-existent link
**Solution**:
```xml
<!-- Make sure link exists before joint references it -->
<link name="link1">...</link>  <!-- Define first -->
<joint name="joint1">
  <parent link="base_link"/>
  <child link="link1"/>  <!-- Then reference -->
</joint>
```

### Warning: "Ignoring inertia for link 'base_link'"
**Cause**: Inertia values are zero or very small
**Solution**: Use realistic mass values (>0.1 kg) and non-zero inertia

### Problem: URDF loads in RViz but looks wrong (distorted geometry)
**Cause**: Scale units or origin wrong
**Solution**:
- Check origin xyz values (should be in meters)
- Verify box sizes are positive numbers
- Remember: 0.2 = 20 cm, not 20 meters

### Problem: Joint appears in wrong position
**Cause**: Joint origin xyz is incorrect
**Solution**:
```xml
<!-- Joint origin moves from parent link frame to joint attachment point -->
<origin xyz="0 0 0.05"/>  <!-- 5 cm above parent's origin -->
```

### Problem: TF tree shows cyclic dependencies
**Cause**: Circular link/joint definitions (link1 → link2 → link1)
**Solution**: Verify each joint has exactly one parent and one child, with no cycles

### Problem: RViz shows skeleton but no visual geometry
**Cause**: Material colors undefined or collision shapes shown instead of visual
**Solution**:
```xml
<!-- Define material once at robot level, then reference -->
<material name="my_material">
  <color rgba="0.5 0.5 0.5 1.0"/>  <!-- RGBA: red, green, blue, alpha -->
</material>

<!-- Use in link -->
<visual>
  <geometry><box size="0.1 0.1 0.1"/></geometry>
  <material name="my_material"/>
</visual>
```

---

## Self-Assessment Checklist

- [ ] I can explain what URDF is and why robots need it
- [ ] I can write a complete `<link>` definition with inertial, visual, and collision geometry
- [ ] I can write a `<joint>` definition and explain parent/child relationships
- [ ] I've created a multi-link robot (3+ links) and loaded it in RViz
- [ ] I can visualize the TF tree and verify the kinematic chain is correct
- [ ] I understand the difference between visual and collision geometry
- [ ] I can calculate or estimate inertia values for simple shapes

---

## Key Takeaways

1. **URDF is the language of robots**: Every ROS 2 robot is described in URDF
2. **Links are rigid bodies**: They have mass, shape, and inertia
3. **Joints are connections**: They define how links move relative to each other
4. **Frames matter**: Every link has a coordinate frame; these must form a tree with no cycles
5. **Visual ≠ Collision**: Use complex meshes for visuals, simple shapes for collision

---

## Next Lesson

Ready to build a real robot? In Lesson 3, you'll create a differential drive mobile robot and spawn it in Gazebo.

[Go to Lesson 3: Building Your First Robot →](./lesson-03-building-robot.md)

---

**Lesson Status**: Complete with URDF anatomy breakdown, two-link example, RViz visualization guide, and troubleshooting.

**Duration**: 60 minutes (including RViz exploration)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-02/two-link-robot.urdf`
- `examples/chapter-2-gazebo/lesson-02/multi-link-arm.urdf` (optional advanced example)

**Concepts Covered**: 7 (URDF format, links, joints, inertia, collision vs visual, joint types, frame naming)
