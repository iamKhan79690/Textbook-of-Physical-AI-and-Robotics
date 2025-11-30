---
sidebar_position: 5
title: "Lesson 4: Physics Simulation Tuning"
description: "Understand and fine-tune physics parameters for realistic robot behavior"
---

# Lesson 4: Physics Simulation Tuning (75 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Understand gravity, friction, and damping and their physical meanings
2. Tune physics parameters to achieve specified robot behavior
3. Diagnose and fix physics instability (bouncing, extreme oscillation)
4. Predict how parameter changes affect robot motion
5. Achieve realistic simulation-to-reality correspondence

---

## Prerequisites

- Lesson 3 completed (differential drive robot)
- Basic understanding of forces and motion

---

## Core Concepts (8 Total)

### 1. Gravity
- **Default**: 9.81 m/s² (Earth gravity)
- **Effect**: Constant downward force on all objects
- **Tuning**: Usually left at 9.81; changing simulates other planets
- **Physics**: F = m × g (heavier objects experience proportionally more gravity force)

### 2. Static Friction
- **Definition**: Resistance to motion when object is stationary
- **Range**: 0.0 to 2.0 (most robots use 0.5-1.0)
- **Effect**: Determines how much force needed to start sliding
- **Example**: μ_static = 0.9 means object needs 90% of its weight in horizontal force to start sliding

### 3. Dynamic Friction (Sliding Friction)
- **Definition**: Resistance to motion when object is already sliding
- **Range**: 0.0 to 2.0 (usually slightly less than static)
- **Effect**: Slows sliding motion after object starts moving
- **Example**: Robot wheels with high friction (0.9) create strong traction

### 4. Linear Damping
- **Definition**: Internal energy loss in joints (air resistance, bearing friction)
- **Range**: 0.0 to 1.0 (most use 0.01-0.1)
- **Effect**: Slows movement over time (viscous drag)
- **Analogy**: Moving through honey is more damped than moving through air

### 5. Angular Damping
- **Definition**: Energy loss in rotational motion
- **Range**: 0.0 to 1.0 (typical 0.01-0.1)
- **Effect**: Stops spinning over time
- **Use case**: Joint motors have internal damping preventing wild oscillations

### 6. Restitution (Bounciness)
- **Definition**: Energy returned after collision
- **Range**: 0.0 (dead bounce, no rebound) to 1.0 (perfect elastic bounce)
- **Effect**: 0.0 = objects stick to ground; 1.0 = balls bounce forever
- **Typical**: Robots use 0.0-0.1 (almost no bounce for stability)

### 7. Contact Margin
- **Definition**: Threshold distance at which contact is detected
- **Range**: 0.0001 to 0.01 m (tiny values important)
- **Effect**: Prevents jittering and false collisions
- **Tuning**: Too small = missed collisions; too large = objects hover above ground

### 8. Physics Engine Selection
- **ODE**: Default, fast, good for wheels
- **Bullet**: Better collision handling
- **DART**: Most accurate but slowest

---

## Physics Parameter Reference Guide

### For Mobile Robots (Wheels)
```xml
<!-- Gazebo world file physics section -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>       <!-- 1 ms simulation step -->
  <real_time_factor>1.0</real_time_factor>   <!-- Real-time speed -->
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- More iterations = more stable but slower -->
    </solver>
  </ode>
</physics>

<!-- Wheel friction (high for traction) -->
<surface>
  <friction>
    <ode>
      <mu>0.9</mu>      <!-- Static friction -->
      <mu2>0.9</mu2>    <!-- Dynamic friction -->
    </ode>
  </friction>
  <contact>
    <ode>
      <soft_cfm>0.0</soft_cfm>
      <soft_erp>0.2</soft_erp>
      <kp>10000000</kp>  <!-- Contact stiffness -->
      <kd>1</kd>         <!-- Contact damping -->
    </ode>
  </contact>
</surface>
```

### For Manipulators (Arm Joints)
```xml
<!-- Low friction for smooth motion -->
<friction>
  <ode>
    <mu>0.3</mu>
    <mu2>0.3</mu2>
  </ode>
</friction>

<!-- Damping prevents oscillation -->
<dynamics damping="0.1" friction="0.05"/>
```

### For Soft Bodies (Grippers)
```xml
<!-- High restitution not needed -->
<contact>
  <ode>
    <restitution>0.0</restitution>  <!-- No bounce -->
    <soft_cfm>0.3</soft_cfm>        <!-- Soft contact -->
  </ode>
</contact>
```

---

## Layer 1: Manual Tuning Exercise

### Exercise: Make Robot Move Exactly 1 Meter

1. **Setup**: Place your differential drive robot in Gazebo
2. **Goal**: Apply command that makes robot move exactly 1.0 meter forward, then stop
3. **Method**:
   - Observe current behavior (too fast? too slow? drifts?)
   - Adjust friction or damping
   - Repeat until behavior matches goal

**Steps**:
```bash
# Terminal 1: Gazebo with your robot
gazebo

# Terminal 2: Send Twist command (forward motion)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Terminal 3: Monitor odometry to measure distance
ros2 topic echo /odom | grep -A2 position
```

---

## Code Example: Physics-Tuned Robot URDF

**File**: `examples/chapter-2-gazebo/lesson-04/differential-drive-tuned.urdf`

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot_tuned">
  <!-- Simulation environment: Gazebo 11+ -->

  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.4"/>
    </inertial>
    <visual>
      <geometry><box size="0.2 0.15 0.1"/></geometry>
      <material name="base_color">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.15 0.1"/></geometry>
    </collision>
  </link>

  <link name="left_wheel">
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="wheel_color">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wheel">
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="wheel_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <link name="caster_wheel">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry><sphere radius="0.01"/></geometry>
      <material name="caster_color">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <geometry><sphere radius="0.01"/></geometry>
    </collision>
  </link>

  <!-- Left Wheel Joint (with optimized physics) -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.075 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="2.0"/>
    <dynamics damping="0.01" friction="0.0"/>  <!-- Low joint damping, friction handled by contact -->
  </joint>

  <!-- Right Wheel Joint (symmetric) -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.075 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="2.0"/>
    <dynamics damping="0.01" friction="0.0"/>
  </joint>

  <!-- Caster Wheel Joint -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.09 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo physics tuning (this would typically be in world file) -->
  <gazebo>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- High friction on wheels for traction -->
    <gazebo reference="left_wheel">
      <surface>
        <friction>
          <ode>
            <mu>0.9</mu>
            <mu2>0.9</mu2>
          </ode>
        </friction>
      </surface>
    </gazebo>

    <gazebo reference="right_wheel">
      <surface>
        <friction>
          <ode>
            <mu>0.9</mu>
            <mu2>0.9</mu2>
          </ode>
        </friction>
      </surface>
    </gazebo>

    <!-- Lower friction on caster for free rolling -->
    <gazebo reference="caster_wheel">
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>
            <mu2>0.1</mu2>
          </ode>
        </friction>
      </surface>
    </gazebo>
  </gazebo>

</robot>
```

---

## Layer 2: AI Collaboration Notes

💬 **Prompt 1**: "My robot oscillates (shakes) after stopping. Is that friction or damping?"

Claude explains:
- Oscillation = underdamped spring-like behavior
- Increase **damping** to absorb energy: `<dynamics damping="0.1"/>`
- Check **contact stiffness** in Gazebo physics (kp parameter)
- Also ensure friction is high enough (0.7+)

💬 **Prompt 2**: "When should I tune physics vs. tuning my control algorithm?"

Claude explains:
- **First**: Get physics right (realistic simulation)
- **Then**: Tune control algorithms to handle that physics
- Bad physics → even perfect control looks bad
- Good physics + simple control → realistic behavior

---

## Troubleshooting

### Problem: Simulation runs in slow motion
**Cause**: `max_step_size` too small or `real_time_factor` set wrong
**Solution**:
```xml
<max_step_size>0.001</max_step_size>  <!-- 1 ms default -->
<real_time_factor>1.0</real_time_factor>  <!-- Keep at 1.0 for real-time -->
```

### Problem: Objects shake or vibrate excessively
**Cause**: Contact stiffness too high or solver not converged
**Solution**:
- Reduce kp (contact stiffness)
- Increase ODE solver iterations (iters: 50 → 100)
- Increase damping on objects

### Problem: Robot drifts to one side
**Cause**: Unequal friction or mass between left/right wheels
**Solution**: Ensure wheels are identical (same mass, friction, radius)

### Problem: "Real time factor dropping below 1.0"
**Cause**: Simulation complexity too high for system performance
**Solution**:
- Reduce solver iterations
- Remove unnecessary collision meshes
- Disable unnecessary plugins

### Problem: Can't get robot to move exactly 1 meter
**Cause**: Wheel slip, incorrect friction, or control loop not tuned
**Solution**:
- First ensure high friction (0.8-1.0)
- Then tune motor command strength
- Finally implement closed-loop control with odometry feedback

---

## Self-Assessment Checklist

- [ ] I understand how gravity, friction, and damping affect robot motion
- [ ] I can predict behavior changes before adjusting parameters
- [ ] I've tuned my robot to move more realistically
- [ ] I can diagnose oscillation vs. drift issues
- [ ] I understand friction coefficients and typical ranges
- [ ] I know how to tune Gazebo physics settings (solver, step size)

---

## Key Takeaways

1. **Physics tuning is iterative**: Start with defaults, adjust based on observations
2. **Friction ≠ Damping**: Friction is contact resistance; damping is energy loss
3. **High friction for wheels**: 0.8-1.0 prevents slipping
4. **Low damping in joints**: 0.01-0.05 allows smooth motion
5. **Stability requires iteration**: No universal "perfect" settings

---

## Next Lesson

In Lesson 5, you'll add sensors to your robot so it can perceive its environment.

[Go to Lesson 5: Adding Sensors →](./lesson-05-adding-sensors.md)

---

**Lesson Status**: Complete with physics parameters guide, tuning exercise, optimized robot URDF, and troubleshooting.

**Duration**: 75 minutes (including tuning iteration)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-04/differential-drive-tuned.urdf`
- `examples/chapter-2-gazebo/lesson-04/physics-parameter-ranges.txt`

**Concepts Covered**: 8 (gravity, static friction, dynamic friction, linear damping, angular damping, restitution, contact margin, physics engine selection)
