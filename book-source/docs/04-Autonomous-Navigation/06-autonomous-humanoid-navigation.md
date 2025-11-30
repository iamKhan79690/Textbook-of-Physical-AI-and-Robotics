# Lesson 6: Autonomous Humanoid Navigation

**Duration**: 3 hours | **Level**: Unlimited | **Priority**: P3 | **Prerequisite**: Lesson 4, Optional advanced track

## Learning Outcomes

By the end of this lesson, you will be able to:

1. **Explain** biped kinematics and unique navigation constraints
2. **Implement** center of gravity (COG) tracking for balance monitoring
3. **Configure** Nav2 for humanoid robots with footstep planning
4. **Design** trajectories that maintain stability constraints
5. **Validate** humanoid navigation with COG within support polygon

## Layer 1: Foundation

### 6.1 Biped Locomotion Basics

**Wheeled robots** can move in any direction → translation + rotation

**Humanoid robots** walk with legs → different constraints:

```
Wheeled robot:
├─ No balance requirement
├─ Omnidirectional movement possible
└─ Simple velocity commands (linear + angular)

Humanoid robot:
├─ Must maintain balance at all times
├─ Center of gravity must stay within support polygon
├─ Feet must be positioned for stable locomotion
├─ Walking is cyclic (swing phase + stance phase)
└─ Trajectory planning must respect these constraints
```

### 6.2 Degrees of Freedom (DOF)

Example: ROBOTIS OP3 humanoid

```
Upper body: 20 DOF
├─ Neck: 3 DOF (pan, tilt, roll)
├─ Arms: 14 DOF (7 each shoulder, elbow, wrist)
└─ Waist: 3 DOF (roll, pitch, yaw)

Lower body: 12 DOF
├─ Hips: 6 DOF (3 each leg)
├─ Knees: 2 DOF (1 each leg)
└─ Ankles: 4 DOF (2 each leg)

Total: 32 DOF (complex kinematic control!)
```

### 6.3 Stability Constraints

**Zero Moment Point (ZMP)**: Point where ground reaction force creates zero moment

**Stability condition**: ZMP must stay inside support polygon (where feet touch ground)

```
Foot positions:
 ┌────────┐
 │ Left   │
 └────────┘
            ┌────────┐
            │ Right  │
            └────────┘

Support polygon = convex hull of foot contact points

COG (Center of Gravity):
        X  (must stay within polygon)
       /|\
      / | \
     /  |  \
    ─────────  (support polygon boundary)
```

**Stability margin**: Distance from COG to polygon boundary
- Margin > 0: Stable
- Margin = 0: At edge (unstable)
- Margin < 0: Falling!

### 6.4 Gait Models

**Static walking**: ZMP always inside support polygon (slow, stable)

**Dynamic walking**: Allows ZMP outside momentarily (fast, requires control)

Most humanoid robots use **static walking** for safety:
- Each leg placement is stable before moving next leg
- Slower but more robust
- Preferred for public interaction

## Layer 2: Collaboration (ROS 2 Integration)

### 2.1 Humanoid Nav2 Configuration

```yaml
# nav2_humanoid_config.yaml
nav2_bringup:
  controller_server:
    ros__parameters:
      # Humanoid-specific parameters
      use_footstep_planner: true
      footstep_planner:
        plugin: "humanoid_nav::FootstepPlanner"

      plugins: ["FootstepController"]
      FootstepController:
        plugin: "humanoid_nav::FootstepController"

        # Step constraints
        max_step_length: 0.3  # meters
        min_step_length: 0.1  # minimum viable step
        step_height: 0.1  # how high to lift feet
        swing_height: 0.15  # maximum foot height during swing

        # Balance constraints
        cog_buffer: 0.05  # safety margin (5cm)
        max_trunk_tilt: 0.3  # radians (~17 degrees)

        # Step planning
        step_time: 1.0  # seconds per step
        swing_time: 0.5  # time for swing phase
        stance_time: 0.5  # time for stance phase

      costmap:
        ros__parameters:
          # Same as differential drive, but humanoid-aware
          inflation_radius: 0.4  # smaller than wheeled (narrower hips)
          plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### 2.2 Humanoid ROS 2 Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/footstep_plan` | nav_msgs/Path | Planned sequence of foot positions |
| `/cog_trajectory` | geometry_msgs/PoseArray | Center of gravity path |
| `/left_foot/pose` | geometry_msgs/PoseStamped | Current left foot position |
| `/right_foot/pose` | geometry_msgs/PoseStamped | Current right foot position |
| `/imu/data` | sensor_msgs/Imu | Balance feedback (accelerometer, gyroscope) |

### 2.3 Footstep Planner

Converts path into valid foot positions:

```python
# code_examples/footstep_planner_config.yaml
FootstepPlanner:
  step_width: 0.2  # meters
  step_length: 0.25
  step_height: 0.1

  # Footstep grid discretization
  angle_divisions: 8  # 8 possible orientations
  step_height_divisions: 5

  # Heuristic cost weighting
  step_cost: 1.0
  change_in_height_cost: 1.5
  change_in_theta_cost: 0.5

  # Collision checking
  collision_check_accuracy: 0.02
  max_height_of_collision: 0.5
```

## Layer 3: Intelligence (Tuning for Humanoid Constraints)

### 3.1 COG Tracking and Validation

```python
# code_examples/track_center_of_gravity.py
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped

class COGTracker(rclpy.Node):
    def __init__(self):
        super().__init__('cog_tracker')

        # Subscribe to body state
        self.create_subscription(
            PoseStamped, '/base_link', self.pose_callback, 10)
        self.create_subscription(
            PointStamped, '/imu/base_link/accelerometer', self.accel_callback, 10)

        # Subscribe to foot positions
        self.create_subscription(
            PoseStamped, '/left_foot/contact', self.left_foot_callback, 10)
        self.create_subscription(
            PoseStamped, '/right_foot/contact', self.right_foot_callback, 10)

        self.cog_pub = self.create_publisher(
            PointStamped, '/cog/position', 10)
        self.stability_pub = self.create_publisher(
            PointStamped, '/cog/stability_margin', 10)

        self.base_pose = None
        self.left_foot = None
        self.right_foot = None

    def compute_cog(self):
        """Compute center of gravity in world frame"""
        if not (self.base_pose and self.left_foot and self.right_foot):
            return None

        # Simplified: COG approximately 55% of body height above base
        cog_height = 0.55 * 1.7  # 1.7m is typical height
        cog = np.array([
            self.base_pose.position.x,
            self.base_pose.position.y,
            cog_height
        ])

        return cog

    def compute_support_polygon(self):
        """Convex hull of foot contact points"""
        if not (self.left_foot and self.right_foot):
            return None

        # Simplified: rectangle between feet
        min_x = min(self.left_foot.position.x, self.right_foot.position.x)
        max_x = max(self.left_foot.position.x, self.right_foot.position.x)
        min_y = min(self.left_foot.position.y, self.right_foot.position.y)
        max_y = max(self.left_foot.position.y, self.right_foot.position.y)

        return {
            'min_x': min_x, 'max_x': max_x,
            'min_y': min_y, 'max_y': max_y
        }

    def check_stability(self, cog, polygon):
        """Check if COG is within support polygon"""
        margin_x_min = cog[0] - polygon['min_x']
        margin_x_max = polygon['max_x'] - cog[0]
        margin_y_min = cog[1] - polygon['min_y']
        margin_y_max = polygon['max_y'] - cog[1]

        min_margin = min(margin_x_min, margin_x_max, margin_y_min, margin_y_max)

        return min_margin > 0, min_margin  # stable, margin value
```

### 3.2 Balance Recovery Strategies

When robot is unstable:

```
1. Center of mass adjustment:
   └─ Shift weight toward stable side

2. Ankle strategy:
   └─ Rotate ankles to adjust foot pressure distribution

3. Hip strategy:
   └─ Move hips to reposition COG over feet

4. Step adjustment:
   └─ Take wider steps for more stability margin
```

### 3.3 Navigation Tuning

| Parameter | Effect | Typical Value |
|-----------|--------|---------------|
| `max_step_length` | How far apart steps can be | 0.2-0.4m |
| `step_height` | How high to lift feet | 0.05-0.15m |
| `cog_buffer` | Stability margin | 0.05-0.1m |
| `max_trunk_tilt` | Upper body lean limit | 0.2-0.4 rad |

**Tuning strategy**:
- Start conservative (small steps, high stability buffer)
- If navigation too slow, increase `max_step_length`
- If unstable, increase `cog_buffer` or decrease step size

## Layer 4: Advanced

### 4.1 Dynamic Walking (MPC Approach)

Model Predictive Control for dynamic walking:

```
1. At each time step:
   a. Measure current state (position, velocity, foot forces)
   b. Predict future 3-5 seconds of motion
   c. Optimize trajectory (minimize energy, maintain stability)
   d. Execute first control input
   e. Repeat

Advantage: Enables faster, more dynamic walking
Disadvantage: Higher computational cost
```

### 4.2 Terrain-Aware Humanoid Navigation

Extend humanoid navigation to handle slopes/stairs:

```python
# Classify terrain
terrain_slope = compute_ground_slope()

if terrain_slope < 0.1:  # flat
    use_normal_walking_gait()
elif terrain_slope < 0.3:  # slight slope
    increase_step_frequency()
    increase_cog_margin()
elif terrain_slope < 0.6:  # steep slope
    reduce_max_step_length()
    reduce_walking_speed()
else:  # stairs or wall
    use_climbing_gait()
```

## Summary

| Concept | Key Insight |
|---------|------------|
| **Biped locomotion** | Unique constraints vs. wheeled robots |
| **COG and stability** | Must maintain COG within foot support polygon |
| **Static vs. dynamic walking** | Static safer, dynamic faster |
| **Footstep planning** | Convert paths into valid foot placements |
| **Balance recovery** | Multiple strategies for stability |

## Code Examples

### Example 1: Validate Humanoid Balance

```python
# code_examples/validate_humanoid_balance.py
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np

class HumanoidBalanceValidator(rclpy.Node):
    def __init__(self):
        super().__init__('balance_validator')

        self.create_subscription(PointStamped, '/cog/position',
                               self.cog_callback, 10)
        self.create_subscription(PoseStamped, '/left_foot',
                               self.left_foot_callback, 10)
        self.create_subscription(PoseStamped, '/right_foot',
                               self.right_foot_callback, 10)

        self.stability_count = 0
        self.unstability_count = 0

    def check_balance(self):
        """Check if COG is within support polygon"""
        cog = self.cog
        left = self.left_foot.position
        right = self.right_foot.position

        # Simple check: COG between feet
        stable = (min(left.x, right.x) <= cog.x <= max(left.x, right.x) and
                 min(left.y, right.y) <= cog.y <= max(left.y, right.y))

        if stable:
            self.stability_count += 1
        else:
            self.unstability_count += 1
            self.get_logger().warn('COG OUTSIDE support polygon - falling risk!')

        if (self.stability_count + self.unstability_count) % 10 == 0:
            success_rate = self.stability_count / (self.stability_count +
                                                 self.unstability_count) * 100
            self.get_logger().info(f'Balance success rate: {success_rate:.1f}%')
```

### Example 2: Humanoid Launch File

```python
# code_examples/nav2_humanoid_launch.py
import launch
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = launch.LaunchDescription()

    # Gazebo simulator (humanoid model)
    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['humanoid_test_world.world'],
        )
    )

    # Humanoid controller (convert Nav2 paths to footsteps)
    ld.add_action(
        Node(
            package='humanoid_nav',
            executable='footstep_controller',
            parameters=[{
                'max_step_length': 0.3,
                'step_height': 0.1,
                'cog_buffer': 0.05,
            }],
        )
    )

    # Nav2 navigation
    ld.add_action(
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            arguments=['--use_sim_time', 'true'],
        )
    )

    return ld
```

## Practice Exercise

**Objective**: Navigate a humanoid robot through a doorway while maintaining balance

**Steps**:
1. Launch Gazebo with humanoid model in test environment
2. Start Nav2 with humanoid-specific configuration
3. Set navigation goal through narrow doorway
4. Monitor:
   - COG position relative to support polygon
   - Balance stability margin
   - Footstep sequence validity
5. Verify successful navigation without falling

**Success criteria**:
- Humanoid walks through doorway
- COG stays within support polygon
- No falling or instability warnings
- Navigation completes within 2 minutes

## References

- **Humanoid Robotics Survey**: [Humanoid Robotics Review](https://arxiv.org/abs/1707.01495)
- **ZMP and Balance**: [Zero Moment Point - Theory and Practice](https://www.researchgate.net/publication/228815738)
- **Footstep Planning**: [Footstep Planning for Humanoid Navigation](https://ieeexplore.ieee.org/document/4651186)

## Next Steps

You now understand humanoid-specific navigation constraints. Choose your path:

- **→ [Lesson 8: Capstone](08-capstone-mission.md)** (Integrate with SLAM + Nav2)
- **→ [Lesson 7: Sensor Fusion](07-multi-sensor-perception-and-fusion.md)** (Robust perception)

---

**Lesson 6 Summary**: Humanoid navigation adds balance constraints not present in wheeled robots. Understanding biped kinematics, stability constraints, and footstep planning enables you to safely navigate humanoid robots in human environments.
