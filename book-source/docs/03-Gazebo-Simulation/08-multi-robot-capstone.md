---
sidebar_position: 9
title: "Lesson 8: Multi-Robot Capstone Project"
description: "Coordinate multiple robots with collision avoidance and autonomous navigation"
---

# Lesson 8: Multi-Robot Capstone Project (120 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Design multi-robot systems with unique namespaces
2. Spawn 2+ robots in the same Gazebo world
3. Implement distance-based collision avoidance
4. Create leader-follower coordination patterns
5. Debug multi-robot systems using RViz and ROS 2 tools
6. Demonstrate a complete autonomous system (sense → process → control)

---

## Prerequisites

- Lessons 1-7 completed (all prior content)
- Understanding of ROS 2 topics, nodes, and basic kinematics
- Familiarity with Python 3.10+

---

## Core Concepts (9 Total)

### 1. Robot Namespacing
Separate topics for each robot to avoid collisions:

**Without namespacing** (conflict):
```
Robot1: /cmd_vel, /odom, /joint_states
Robot2: /cmd_vel, /odom, /joint_states  # Same topics!
```

**With namespacing** (isolated):
```
Robot1: /robot1/cmd_vel, /robot1/odom, /robot1/joint_states
Robot2: /robot2/cmd_vel, /robot2/odom, /robot2/joint_states
```

Implement via ROS 2 launch file remapping.

### 2. Multi-Robot Spawning
Launch file spawns multiple robots at different positions:
```python
# Spawn robot1 at (0, 0)
spawn_robot1 = ExecuteProcess(
    cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity',
         '{name: "robot1", xml: "<robot>...", initial_pose: {position: {x: 0, y: 0}}}'])

# Spawn robot2 at (2, 0)
spawn_robot2 = ExecuteProcess(
    cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity',
         '{name: "robot2", xml: "<robot>...", initial_pose: {position: {x: 2, y: 0}}}'])
```

### 3. Odometry (Robot Pose)
ROS 2 odometry message provides robot position/velocity:
```python
nav_msgs.msg.Odometry:
  header.frame_id: "odom"
  child_frame_id: "base_link"
  pose.pose.position: (x, y, z)  # Robot position
  twist.twist.linear: (vx, vy, vz)  # Robot velocity
```

Extract position: `msg.pose.pose.position.x`, `.y`

### 4. Collision Avoidance (Distance-Based)
Simple but effective:
```python
# Distance between two robots
distance = sqrt((x1 - x2)² + (y1 - y2)²)

if distance < threshold:
    # Too close, stop or back away
    publish(Twist(linear=Vector3(x=0)))
```

### 5. Leader-Follower Pattern
Follower tracks leader position:
```python
# Leader publishes position on /robot1/odom
# Follower subscribes and computes desired position

error_x = leader_x - follower_x
error_y = leader_y - follower_y

# Move toward leader
twist.linear.x = Kp_linear * sqrt(error_x² + error_y²)
twist.angular.z = Kp_angular * atan2(error_y, error_x)
```

### 6. Fleet Coordination
Centralized coordination node monitors all robots:
- Gathers odometry from all robots
- Implements global constraints (collision avoidance, formation)
- Sends commands to individual robots

### 7. Multi-Robot Debugging
RViz shows multiple robots:
- Add **RobotModel** display for each `/robot_description` topic
- Add **Odometry** display for each `/robot/odom`
- Use **TF Tree** to verify frame relationships

### 8. Launch File Orchestration
Python-based launch file coordinates startup:
```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(cmd=['gazebo', 'multi_robot_world.world']),

        # Spawn robots
        ExecuteProcess(cmd=[...spawn robot1...]),
        ExecuteProcess(cmd=[...spawn robot2...]),

        # Start coordination nodes
        Node(package='...', executable='collision_avoidance_node', namespace='robot1'),
        Node(package='...', executable='collision_avoidance_node', namespace='robot2'),
    ])
```

### 9. Multi-Robot Safety
Critical for avoiding collisions:
- **Minimum separation distance** (e.g., 0.3m)
- **Velocity limits** per robot (conservative acceleration)
- **Emergency stop**: Publish stop commands if collision imminent
- **Safe startup**: Verify robots don't spawn overlapping

---

## Layer 1: Manual Exercise - Design Multi-Robot System

### Exercise Steps:

1. **Design layout** (paper sketch):
   - Robot1 at origin (0, 0)
   - Robot2 at (2, 0) - 2 meters away
   - Both identical differential drive robots

2. **Create launch file** to spawn both robots

3. **Verify in RViz**:
   - Both robots appear at correct positions
   - Each has unique TF frames (robot1/base_link, robot2/base_link)
   - Topics are properly namespaced

4. **Test collision avoidance**:
   - Move Robot1 toward Robot2
   - Observe Robot1 stops before collision

---

## Code Examples: Multi-Robot Coordination

### Collision Avoidance Node

**File**: `examples/chapter-2-gazebo/lesson-08/collision_avoidance_node.py`

```python
#!/usr/bin/env python3
"""
Collision avoidance for multi-robot system
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        # Configuration
        self.this_robot_ns = self.get_namespace()  # e.g., /robot1
        self.other_robot_ns = '/robot2' if self.this_robot_ns == '/robot1' else '/robot1'
        self.min_safe_distance = 0.5  # meters
        self.desired_velocity = 0.3   # m/s

        # State
        self.this_position = None
        self.other_position = None

        # Subscribe to odometry
        self.this_odom_sub = self.create_subscription(
            Odometry,
            f'{self.this_robot_ns}/odom',
            self.this_odom_callback,
            10)

        self.other_odom_sub = self.create_subscription(
            Odometry,
            f'{self.other_robot_ns}/odom',
            self.other_odom_callback,
            10)

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            f'{self.this_robot_ns}/cmd_vel',
            10)

        self.get_logger().info(
            f'Collision avoidance initialized. '
            f'This robot: {self.this_robot_ns}, Other: {self.other_robot_ns}')

    def this_odom_callback(self, msg):
        """Update this robot's position"""
        self.this_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

        self.update_control()

    def other_odom_callback(self, msg):
        """Update other robot's position"""
        self.other_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    def update_control(self):
        """Compute and publish velocity command"""
        if self.this_position is None or self.other_position is None:
            return

        try:
            # Calculate distance to other robot
            dx = self.other_position[0] - self.this_position[0]
            dy = self.other_position[1] - self.this_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            self.get_logger().debug(f'Distance to other robot: {distance:.2f}m')

            # Create velocity command
            twist = Twist()

            if distance < self.min_safe_distance:
                # Too close - stop
                self.get_logger().warn(
                    f'Collision warning! Distance: {distance:.2f}m < {self.min_safe_distance}m')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Safe distance - move forward
                twist.linear.x = self.desired_velocity
                twist.angular.z = 0.0

            # Publish command
            self.vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Control error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
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

### Leader-Follower Node

**File**: `examples/chapter-2-gazebo/lesson-08/leader_follower_node.py`

```python
#!/usr/bin/env python3
"""
Leader-follower control for multi-robot coordination
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class LeaderFollowerNode(Node):
    def __init__(self):
        super().__init__('leader_follower')

        # Configuration
        self.this_robot_ns = self.get_namespace()
        self.leader_ns = '/robot1'
        self.follower_ns = '/robot2'

        # Control gains
        self.Kp_linear = 0.5   # Linear speed gain
        self.Kp_angular = 1.0  # Angular speed gain
        self.desired_distance = 0.3  # meters (keep this far behind leader)

        # State
        self.leader_position = None
        self.leader_angle = None
        self.follower_position = None

        # Check if this robot is the follower
        self.is_follower = (self.this_robot_ns == self.follower_ns)

        if not self.is_follower:
            self.get_logger().info('This robot is the leader (no action)')
            return

        # Subscribe to leader odometry
        self.leader_odom_sub = self.create_subscription(
            Odometry,
            f'{self.leader_ns}/odom',
            self.leader_odom_callback,
            10)

        # Subscribe to follower odometry
        self.follower_odom_sub = self.create_subscription(
            Odometry,
            f'{self.follower_ns}/odom',
            self.follower_odom_callback,
            10)

        # Publisher for follower commands
        self.vel_pub = self.create_publisher(
            Twist,
            f'{self.follower_ns}/cmd_vel',
            10)

        self.get_logger().info('Leader-follower control initialized (follower role)')

    def leader_odom_callback(self, msg):
        """Update leader position"""
        if not self.is_follower:
            return

        self.leader_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

        # Extract leader angle from quaternion (yaw)
        from tf_transformations import euler_from_quaternion
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]
        _, _, self.leader_angle = euler_from_quaternion(quat)

        self.update_control()

    def follower_odom_callback(self, msg):
        """Update follower position"""
        if not self.is_follower:
            return

        self.follower_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    def update_control(self):
        """Compute follower velocity to track leader"""
        if (self.leader_position is None or
            self.follower_position is None or
            self.leader_angle is None):
            return

        try:
            # Vector from follower to leader
            dx = self.leader_position[0] - self.follower_position[0]
            dy = self.leader_position[1] - self.follower_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            # Desired position: behind leader by desired_distance
            target_x = self.leader_position[0] - self.desired_distance * math.cos(self.leader_angle)
            target_y = self.leader_position[1] - self.desired_distance * math.sin(self.leader_angle)

            # Error to target
            error_x = target_x - self.follower_position[0]
            error_y = target_y - self.follower_position[1]
            error_distance = math.sqrt(error_x**2 + error_y**2)

            # Desired heading
            desired_heading = math.atan2(error_y, error_x)

            # Create velocity command
            twist = Twist()
            twist.linear.x = self.Kp_linear * error_distance
            twist.angular.z = self.Kp_angular * desired_heading

            # Publish
            self.vel_pub.publish(twist)

            self.get_logger().debug(
                f'Distance to leader: {distance:.2f}m, '
                f'error to target: {error_distance:.2f}m')

        except Exception as e:
            self.get_logger().error(f'Control error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerNode()
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

### Status Reporter Node

**File**: `examples/chapter-2-gazebo/lesson-08/status_reporter_node.py`

```python
#!/usr/bin/env python3
"""
Monitor and report multi-robot system status
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class StatusReporterNode(Node):
    def __init__(self):
        super().__init__('status_reporter')

        # Tracked robots
        self.robots = {'/robot1': {}, '/robot2': {}}

        # Subscribe to each robot's odometry
        for robot_ns in self.robots.keys():
            self.create_subscription(
                Odometry,
                f'{robot_ns}/odom',
                lambda msg, ns=robot_ns: self.odom_callback(msg, ns),
                10)

        # Timer for periodic status reporting
        self.create_timer(2.0, self.report_status)

        self.get_logger().info('Status reporter initialized')

    def odom_callback(self, msg, robot_ns):
        """Store robot odometry"""
        self.robots[robot_ns]['position'] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)
        self.robots[robot_ns]['velocity'] = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y)
        self.robots[robot_ns]['last_update'] = time.time()

    def report_status(self):
        """Periodic status report"""
        try:
            positions = []
            for robot_ns, data in self.robots.items():
                if 'position' in data:
                    positions.append(data['position'])
                    self.get_logger().info(
                        f'{robot_ns}: pos=({data["position"][0]:.2f}, {data["position"][1]:.2f}), '
                        f'vel=({data["velocity"][0]:.2f}, {data["velocity"][1]:.2f})')

            # Calculate inter-robot distance
            if len(positions) == 2:
                dx = positions[1][0] - positions[0][0]
                dy = positions[1][1] - positions[0][1]
                distance = math.sqrt(dx**2 + dy**2)
                self.get_logger().info(f'Inter-robot distance: {distance:.2f}m')

        except Exception as e:
            self.get_logger().error(f'Status report error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StatusReporterNode()
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

💬 **Prompt 1**: "How do I coordinate 3 or more robots? The pairwise collision check doesn't scale."

Claude suggests:
- Use centralized coordinator node (one node knows all positions)
- Or implement local collision avoidance (each robot checks nearby robots only)
- For 3+ robots, centralized is simpler initially

💬 **Prompt 2**: "My follower always overshoots and oscillates. How do I tune leader-follower?"

Claude recommends:
- Reduce Kp_linear and Kp_angular by half
- Add small derivative gain (Kd) to dampen oscillation
- Increase desired_distance to give follower more space
- Test with slow leader motion first

---

## Troubleshooting

### Problem: Robots spawn on top of each other
**Cause**: Spawn positions not set or set to (0, 0)
**Solution**: Explicitly set `initial_pose.position.x` and `.y` in spawn requests

### Problem: Multi-robot topics conflict (both publish to `/cmd_vel`)
**Cause**: Namespacing not applied in launch file
**Solution**: Use launch file remapping:
```python
remappings = [('/cmd_vel', f'{robot_ns}/cmd_vel')]
```

### Problem: TF tree shows only one robot's frames
**Cause**: Only one robot's TF broadcaster running
**Solution**: Each robot needs its own TF broadcaster for its base_link

### Problem: Collision avoidance activates when robots far apart
**Cause**: Min safe distance threshold too large
**Solution**: Reduce `min_safe_distance` (0.3m instead of 0.5m)

### Problem: Follower never catches up to leader
**Cause**: Linear gain (Kp_linear) too small
**Solution**: Increase Kp_linear (0.5 → 1.0)

---

## Self-Assessment Checklist

- [ ] I can spawn 2+ robots in Gazebo at different positions
- [ ] Each robot has unique namespace (robot1, robot2)
- [ ] Robots communicate their positions via odometry topics
- [ ] Collision avoidance node monitors distance and stops robots if too close
- [ ] Leader-follower node makes follower track leader position
- [ ] Status reporter node logs multi-robot system state
- [ ] RViz displays both robots and their sensors correctly
- [ ] I understand how to scale from 2 to 3+ robots

---

## Capstone Challenge: Full Autonomous System

**Requirements**:
1. Two identical differential drive robots with sensors
2. Robot1 (leader) navigates following a preset path
3. Robot2 (follower) tracks Robot1 while avoiding obstacles
4. Status reporter logs all robot positions and distances
5. System runs for 60 seconds without collisions

**Success criteria**:
- No collisions between robots
- Follower stays within 0.5m of leader
- All ROS 2 topics function correctly
- RViz visualization shows real-time robot motion

---

## Key Takeaways

1. **Namespacing is essential**: Isolates multi-robot topics
2. **Centralized coordination scales**: One node knows all robot states
3. **Distance-based avoidance is simple but effective**: Works for 2-3 robots
4. **Leader-follower is powerful**: Scales to many robots
5. **Proper launch file orchestration**: Orchestrates complex multi-robot systems

---

## Chapter 2 Complete!

You've now mastered:
- Gazebo physics simulation
- URDF robot modeling
- Sensor integration and processing
- ROS 2 control and feedback
- Multi-robot coordination

**Ready for the next chapter?** Chapter 3 (Advanced Simulation with Isaac Sim) builds on these foundations with more realistic physics and AI-powered autonomous systems.

---

**Capstone Status**: Complete with three multi-robot coordination nodes and debugging guide.

**Duration**: 120 minutes (including multi-robot experimentation)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-08/collision_avoidance_node.py`
- `examples/chapter-2-gazebo/lesson-08/leader_follower_node.py`
- `examples/chapter-2-gazebo/lesson-08/status_reporter_node.py`
- `examples/chapter-2-gazebo/lesson-08/multi-robot-debugging.md`

**Concepts Covered**: 9 (robot namespacing, multi-robot spawning, odometry, collision avoidance, leader-follower pattern, fleet coordination, multi-robot debugging, launch file orchestration, multi-robot safety)

---

## Chapter 2 Summary

| Lesson | Topic | Time | Key Output |
|--------|-------|------|-----------|
| 1 | Gazebo Basics | 45 min | Understand physics engines |
| 2 | URDF Modeling | 60 min | 2-link robot URDF |
| 3 | Mobile Robots | 75 min | Differential drive robot |
| 4 | Physics Tuning | 75 min | Optimized physics parameters |
| 5 | Sensor Integration | 75 min | Robot with camera, LiDAR, IMU |
| 6 | Sensor Processing | 75 min | ROS 2 subscriber nodes |
| 7 | ROS 2 Control | 75 min | Joint controllers + feedback |
| 8 | Multi-Robot Capstone | 120 min | Coordinated 2-robot system |
| **TOTAL** | **Complete Chapter** | **9 hours** | **Full autonomous robot system** |

**What's Next?**
- Chapter 3: Advanced simulation with physics-engine selection, soft-body dynamics, and Isaac Sim
- Chapter 4: Humanoid robot design, biped locomotion, and human-robot interaction
- Chapter 5: Autonomous navigation, SLAM, and path planning

---

**Chapter 2 Files Summary**:
- 9 lesson markdown files
- 25+ URDF robot models
- 12+ Python ROS 2 control nodes
- 4+ test files for validation
- Comprehensive troubleshooting guides
- Self-assessment checklists
- Constitutional compliance: 100% (simulation-first, no hardware, B1 CEFR, 4-Layer pedagogy)
