---
sidebar_position: 8
title: "Lesson 7: Gazebo-ROS 2 Integration"
description: "Command robot joints via ROS 2 topics and implement feedback control"
---

# Lesson 7: Gazebo-ROS 2 Integration (75 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Publish ROS 2 messages to command robot joints (effort/velocity/position)
2. Subscribe to joint state feedback and monitor robot configuration
3. Implement basic proportional feedback control
4. Command differential drive robots with Twist messages
5. Coordinate perception (sensors) with control (actuators) in a closed loop

---

## Prerequisites

- Lesson 6 completed (sensor processing)
- Understanding of ROS 2 publishers
- Basic control theory concepts (setpoint, error, feedback)

---

## Core Concepts (8 Total)

### 1. Joint Command Interface
ROS 2 joint control via topics:
- **Effort**: Apply force/torque directly (N or N⋅m)
- **Velocity**: Command desired speed (rad/s)
- **Position**: Command target angle (rad)

Message types:
```python
# Effort control (motors)
std_msgs.msg.Float64(data=10.0)  # 10 N⋅m torque

# Trajectory control (position + velocity + acceleration)
trajectory_msgs.msg.JointTrajectory(joint_names=['joint1'], points=[...])
```

### 2. Joint State Feedback
ROS 2 publishes robot state on `/joint_states`:
```python
sensor_msgs.msg.JointState:
  name: [left_wheel, right_wheel, shoulder, elbow]
  position: [0.5, 0.5, 1.2, -0.8]  # Current angles (rad)
  velocity: [0.1, 0.1, 0.0, 0.0]   # Current speeds (rad/s)
  effort: [5.0, 5.0, 0.0, 0.0]     # Current torques (N⋅m)
```

### 3. Twist Messages (Differential Drive)
Command differential drive robots with high-level velocity:
```python
geometry_msgs.msg.Twist:
  linear.x: 0.5   # Forward speed (m/s)
  linear.y: 0.0   # Lateral speed (not used for differential drive)
  linear.z: 0.0
  angular.x: 0.0
  angular.y: 0.0
  angular.z: 0.2  # Rotation speed (rad/s)
```

### 4. Differential Drive Kinematics (Control)
Convert Twist → wheel velocities:
```
v_left = (linear_x - angular_z × wheelbase/2)
v_right = (linear_x + angular_z × wheelbase/2)
```

Where `wheelbase` = distance between left/right wheels

### 5. JointTrajectory Messages
Command multiple joints with trajectory:
```python
trajectory_msgs.msg.JointTrajectory:
  joint_names: ['shoulder', 'elbow']
  points:
    - positions: [0.0, 0.0]
      velocities: [0.1, 0.1]
      time_from_start: {sec: 1, nsec: 0}
    - positions: [1.57, 0.5]
      velocities: [0.0, 0.0]
      time_from_start: {sec: 3, nsec: 0}
```

### 6. Proportional (P) Control
Simple feedback loop:
```
error = target - current
effort = Kp × error
```

Example: Move joint to target position
```python
target_angle = 1.57  # rad
current_angle = 0.0
Kp = 5.0  # Proportional gain

error = target_angle - current_angle
torque = Kp * error  # Publish to joint
```

### 7. State Publisher (TF Broadcasting)
Publish robot pose in world frame:
```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

broadcaster = TransformBroadcaster(self)
transform = TransformStamped()
transform.header.frame_id = "world"
transform.child_frame_id = "base_link"
transform.transform.translation.x = 1.0  # Robot position
broadcaster.sendTransform(transform)
```

### 8. Closed-Loop System
Full loop: Sense → Process → Control → Execute
```
Sensors (image, LiDAR, IMU)
  ↓ (ROS 2 topics)
Processing (edge detection, obstacle detection)
  ↓ (decision making)
Control (compute effort from sensor data)
  ↓ (ROS 2 topics)
Actuators (wheel motors, arm joints)
  ↓ (Gazebo simulation)
Back to Sensors (feedback loop)
```

---

## Layer 1: Manual Exercise - Publish Commands

### Exercise 1: Move Differential Drive with Twist

```bash
# Terminal 1: Gazebo with robot
gazebo

# Terminal 2: Send Twist command (move forward)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Terminal 3: Monitor odometry
ros2 topic echo /odom
```

### Exercise 2: Subscribe to Joint State

```bash
# Terminal 1: Gazebo

# Terminal 2: Watch joint states update
ros2 topic echo /joint_states
```

---

## Code Examples: Control Nodes

### Differential Drive Controller

**File**: `examples/chapter-2-gazebo/lesson-07/differential_drive_controller.py`

```python
#!/usr/bin/env python3
"""
Differential drive controller (Twist → wheel efforts)
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Robot parameters
        self.wheel_radius = 0.05  # meters
        self.wheelbase = 0.15     # distance between left/right wheels
        self.max_torque = 10.0    # Newton⋅meters

        # Publishers for wheel efforts
        self.left_wheel_pub = self.create_publisher(
            Float64, '/left_wheel_joint/effort', 10)
        self.right_wheel_pub = self.create_publisher(
            Float64, '/right_wheel_joint/effort', 10)

        # Subscriber for velocity commands
        self.vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10)

        self.get_logger().info('Differential drive controller initialized')

    def velocity_callback(self, msg):
        """Convert Twist to wheel efforts"""
        try:
            # Extract linear and angular velocity
            v_linear = msg.linear.x      # m/s
            v_angular = msg.angular.z    # rad/s

            # Differential drive kinematics
            # v_left = v_linear - (wheelbase/2) * v_angular
            # v_right = v_linear + (wheelbase/2) * v_angular
            v_left = v_linear - (self.wheelbase / 2.0) * v_angular
            v_right = v_linear + (self.wheelbase / 2.0) * v_angular

            # Convert velocity to motor effort (simple proportional control)
            # effort = velocity / wheel_radius × Kp
            Kp = 2.0
            effort_left = (v_left / self.wheel_radius) * Kp
            effort_right = (v_right / self.wheel_radius) * Kp

            # Clamp to max torque
            effort_left = max(-self.max_torque, min(self.max_torque, effort_left))
            effort_right = max(-self.max_torque, min(self.max_torque, effort_right))

            # Publish efforts
            self.left_wheel_pub.publish(Float64(data=effort_left))
            self.right_wheel_pub.publish(Float64(data=effort_right))

            self.get_logger().debug(
                f'Twist: linear={v_linear:.2f}, angular={v_angular:.2f} → '
                f'efforts: left={effort_left:.2f}, right={effort_right:.2f}')

        except Exception as e:
            self.get_logger().error(f'Control error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Joint State Listener

**File**: `examples/chapter-2-gazebo/lesson-07/joint_state_listener.py`

```python
#!/usr/bin/env python3
"""
Subscribe to joint states and monitor robot configuration
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.get_logger().info('Joint state listener initialized')

    def joint_callback(self, msg):
        """Process joint state feedback"""
        try:
            # Log all joint states
            self.get_logger().info('Joint states:')
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                eff = msg.effort[i] if i < len(msg.effort) else 0.0
                self.get_logger().info(
                    f'  {name}: pos={pos:.3f} rad, vel={vel:.3f} rad/s, '
                    f'effort={eff:.3f} N⋅m')

        except Exception as e:
            self.get_logger().error(f'Error processing joint state: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### PID-Based Position Controller

**File**: `examples/chapter-2-gazebo/lesson-07/pid_control_demo.py`

```python
#!/usr/bin/env python3
"""
PID feedback control for joint position
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Target position
        self.target_position = 1.57  # 90 degrees (radians)

        # PID gains
        self.Kp = 5.0    # Proportional
        self.Ki = 0.1    # Integral
        self.Kd = 0.5    # Derivative

        # State tracking
        self.last_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

        # Publishers and subscribers
        self.joint_effort_pub = self.create_publisher(
            Float64, '/shoulder_joint/effort', 10)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.get_logger().info(f'PID controller initialized, target: {self.target_position:.2f} rad')

    def joint_callback(self, msg):
        """PID control loop"""
        try:
            # Find shoulder joint in state message
            joint_index = msg.name.index('shoulder_joint')
            current_position = msg.position[joint_index]

            # Calculate error
            error = self.target_position - current_position

            # Calculate time delta
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            # Proportional term
            p_term = self.Kp * error

            # Integral term
            self.integral_error += error * dt
            i_term = self.Ki * self.integral_error

            # Derivative term
            d_term = self.Kd * (error - self.last_error) / dt if dt > 0 else 0
            self.last_error = error

            # Total effort
            effort = p_term + i_term + d_term

            # Clamp to max
            max_effort = 10.0
            effort = max(-max_effort, min(max_effort, effort))

            # Publish
            self.joint_effort_pub.publish(Float64(data=effort))

            self.get_logger().debug(
                f'PID: pos={current_position:.3f}, target={self.target_position:.3f}, '
                f'error={error:.3f}, effort={effort:.3f}')

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Joint not found or error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Layer 2: AI Collaboration Notes

💬 **Prompt 1**: "My Twist command doesn't move the robot. What's wrong?"

Claude checks:
- Is differential drive controller running? (`ros2 node list`)
- Is `/cmd_vel` topic being received? (`ros2 topic hz /cmd_vel`)
- Are wheel effort topics being published? (`ros2 topic list | grep effort`)
- Check Gazebo has joint controllers enabled

💬 **Prompt 2**: "My PID controller oscillates around the target. How do I tune it?"

Claude suggests:
- Start with Kp only (set Ki, Kd to 0)
- Increase Kp until oscillation starts
- Back off Kp by 50%
- Add small Kd to dampen oscillation
- Add Ki to eliminate steady-state error

---

## Troubleshooting

### Problem: Commands published but robot doesn't move
**Cause**: Effort topic wrong name or controller not reading it
**Solution**:
```bash
# Verify effort topic
ros2 topic list | grep -i effort

# Manually publish test command
ros2 topic pub --once /left_wheel_joint/effort std_msgs/Float64 "{data: 5.0}"
```

### Problem: Joint feedback reports wrong values
**Cause**: Joint state publisher not running or joint name mismatch
**Solution**:
```bash
# Check joint states available
ros2 topic echo /joint_states
```

### Problem: Robot moves but too slow or too fast
**Cause**: Scaling factor (Kp) wrong
**Solution**: Adjust `Kp` in controller (lower = slower, higher = faster)

### Problem: PID controller diverges (huge torques)
**Cause**: Gains too aggressive or integral term accumulating
**Solution**:
- Reduce Kp
- Reduce Ki (or set to 0)
- Add Kd for damping

### Problem: "/left_wheel_joint/effort" topic doesn't exist
**Cause**: Joint controller plugin not loaded in Gazebo
**Solution**: Add gazebo_ros joint controller plugins to URDF

---

## Self-Assessment Checklist

- [ ] I can publish Twist messages to move a differential drive robot
- [ ] I can subscribe to joint state and extract position/velocity/effort values
- [ ] I've implemented proportional control for a single joint
- [ ] I've converted Twist (linear, angular velocity) to wheel efforts using kinematics
- [ ] I can close the perception-action loop (sensor → process → control → actuate)
- [ ] My robot responds predictably to ROS 2 commands

---

## Key Takeaways

1. **Joint control is topic-based**: Publish efforts/velocities to command motors
2. **Joint state feedback enables closed-loop control**: Monitor what you've achieved
3. **Proportional control works but oscillates**: Add derivative and integral for smoothness
4. **Differential drive kinematics converts Twist → wheel efforts**: Essential for mobile robots
5. **Closed-loop systems need feedback**: Sensors + control = autonomous behavior

---

## Next Lesson

In Lesson 8, you'll coordinate multiple robots, implement collision avoidance, and build a full autonomous system.

[Go to Lesson 8: Multi-Robot Capstone →](./lesson-08-multi-robot-capstone.md)

---

**Lesson Status**: Complete with three control node examples (differential drive, state listener, PID controller).

**Duration**: 75 minutes (including feedback control experimentation)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-07/differential_drive_controller.py`
- `examples/chapter-2-gazebo/lesson-07/joint_state_listener.py`
- `examples/chapter-2-gazebo/lesson-07/pid_control_demo.py`

**Concepts Covered**: 8 (joint command interface, joint state feedback, Twist messages, differential drive kinematics, JointTrajectory, proportional control, state publishing, closed-loop systems)
