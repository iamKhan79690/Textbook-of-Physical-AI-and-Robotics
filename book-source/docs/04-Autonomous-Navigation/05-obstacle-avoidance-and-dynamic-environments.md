# Lesson 5: Obstacle Avoidance and Dynamic Environments

**Duration**: 3 hours | **Level**: Unlimited | **Priority**: P2 | **Prerequisite**: Lesson 4

## Learning Outcomes

By the end of this lesson, you will be able to:

1. **Implement** local obstacle avoidance with dynamic window approach
2. **Design** recovery behaviors for stuck detection
3. **Configure** costmap layers for real-time dynamic obstacle handling
4. **Tune** dynamic window parameters for different robot types
5. **Debug** navigation failures in constrained environments

## Layer 1: Foundation

### 5.1 Static vs. Dynamic Obstacles

**Static obstacles**: Permanent features in environment
- Buildings, walls, furniture
- Known from map
- Don't change over time

**Dynamic obstacles**: Temporary, moving objects
- People, other robots, vehicles
- Not in pre-built map
- Change position in real-time

**Challenge**: Robot must avoid both simultaneously while reaching goal.

### 5.2 Costmap Inflation for Safety

Inflation layer prevents robot from touching obstacles by expanding them:

```
Robot footprint: circle with radius R
Safety margin: distance from robot to obstacles

Inflation = R + safety_margin

Example:
Robot radius: 0.3m
Safety margin: 0.25m
Total inflation: 0.55m

Effect on costmap:
─────────┐
     O   │ Original obstacle
 (1.0m)  │
─────────┤
  ○ ○ ○  │ Inflated region
   ○ ○   │ (safety margin)
─────────┘
```

**Parameters**:
- `inflation_radius`: Total expansion distance
- `cost_scaling_factor`: Gradient steepness
  - Higher = steeper cost gradient (avoid close to obstacles)
  - Lower = gentler gradient (allows closer approach)

### 5.3 Local Trajectory Evaluation (DWA)

The local planner evaluates candidate trajectories:

```
Current state (x, y, theta, v, omega)
                │
    ┌───────────┼───────────┐
    │           │           │
Sample velocity pairs:
(v=0.0, omega=0.0)  →  Trajectory 1: goes straight
(v=0.5, omega=0.0)  →  Trajectory 2: accelerates
(v=0.3, omega=0.5)  →  Trajectory 3: curves right
...

For each trajectory:
├─ Simulate next 1 second of motion
├─ Check collision with obstacles
├─ Measure distance to goal
├─ Calculate cost = obstacle_cost + goal_distance_cost
│
Select lowest-cost trajectory → execute first control input
```

### 5.4 Recovery Behaviors

When robot gets stuck, recovery behaviors attempt to escape:

```
Stuck detection:
├─ Move_base_flex monitors navigation progress
├─ If position doesn't change after T seconds → stuck
└─ Trigger recovery behavior

Recovery sequence:
1. Clear local costmap (maybe sensor noise?)
2. Rotate in place (360 degrees)
3. If still stuck: backup 0.5m
4. If still stuck: rotate again
5. If still stuck: declare failure
```

**Why this works**:
- Clears spurious obstacles from sensor noise
- Rotation explores alternative paths
- Backup escapes local minima

## Layer 2: Collaboration (ROS 2 Integration)

### 2.1 Dynamic Costmap Layers

```yaml
controller_server:
  ros__parameters:
    costmap:
      ros__parameters:
        plugins: ["static_layer", "obstacle_layer", "inflation_layer", "voxel_layer"]

        static_layer:
          plugin: "nav2_costmap_2d::StaticLayer"
          enabled: true

        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: true
          observation_sources: scan
          scan:
            topic: /scan  # Real-time LiDAR data
            max_obstacle_height: 2.0
            obstacle_max_range: 2.5
            min_obstacle_height: 0.0  # Floor level
            raytrace_max_range: 3.0
            marking: true  # Mark obstacles
            clearing: true  # Clear when no obstacle

        voxel_layer:
          plugin: "nav2_costmap_2d::VoxelLayer"
          enabled: true
          observation_sources: scan_voxel
          scan_voxel:
            topic: /scan
            max_obstacle_height: 2.0
            min_obstacle_height: 0.0
            marking: true
            clearing: true
            decay_model: exponential  # Fade old sensor data
            decay_parameter: 0.85

        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          enabled: true
          inflation_radius: 0.55
          cost_scaling_factor: 10.0
          inflate_unknown: false  # Don't inflate unknown cells
```

### 2.2 Dynamic Window Approach (DWA) Parameters

```yaml
controller_server:
  ros__parameters:
    plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_dwb_controllers/DWBLocalPlanner"

      # Simulation parameters
      sim_time: 1.0  # seconds (prediction horizon)
      samples_in_time: 8  # iterations for forward simulation
      num_y_samples: 20  # trajectory samples
      num_theta_samples: 20

      # Velocity limits
      min_vel_x: -0.26  # backward
      max_vel_x: 0.5   # forward
      max_vel_y: 0.0   # differential drive (no lateral)
      max_vel_theta: 1.57  # rad/s

      # Acceleration limits (ramp-up)
      acc_lim_x: 2.5  # m/s²
      acc_lim_theta: 3.2  # rad/s²

      # Costs (weighted scoring)
      path_distance_bias: 32.0  # Prefer paths close to reference
      goal_distance_bias: 24.0  # Prefer paths toward goal
      occdist_scale: 0.02  # Avoid obstacles (low weight)
      forward_point_distance: 0.325  # Look-ahead distance
```

### 2.3 Behavior Tree for Recovery

```xml
<!-- navigate_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="main_sequence">
      <!-- Try to reach goal -->
      <FollowPath name="follow_path"
                   path="{path}"
                   error_code_id="follow_error"/>

      <!-- If failed, run recovery -->
      <ReactiveSequence name="recovery_sequence">
        <!-- 1. Clear costmap -->
        <ClearEntireCostmap name="clear_costmap" service_name="local_costmap/clear_entirely_local_costmap"/>

        <!-- 2. Try again -->
        <FollowPath name="follow_path_after_clear" path="{path}"/>

        <!-- 3. If still failing, rotate -->
        <Spin name="spin" service_name="spin"/>

        <!-- 4. Backup if rotating didn't help -->
        <BackUp name="back_up" service_name="backup" distance="0.5"/>
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
```

### 2.4 ROS 2 Services for Recovery

| Service | Request | Response | Purpose |
|---------|---------|----------|---------|
| `/clear_costmap` | - | - | Clear costmap cells |
| `/spin` | max_spin_vel | Success | Rotate in place |
| `/backup` | distance, speed | Success | Move backward |

## Layer 3: Intelligence (Tuning and Debugging)

### 3.1 Obstacle Avoidance Tuning

**Scenario: Robot crashes into obstacles**
- Problem: Inflation not sufficient
- Solution: Increase `inflation_radius` or `cost_scaling_factor`

**Scenario: Robot stays too far from obstacles**
- Problem: Overly conservative
- Solution: Decrease `cost_scaling_factor`

**Scenario: Jerky motions near obstacles**
- Problem: DWA not smoothing trajectory
- Solution: Increase `forward_point_distance`

**Scenario: Robot doesn't react to dynamic obstacles**
- Problem: Costmap update too slow
- Solution: Increase `update_frequency`

### 3.2 Parameter Tuning Guide

| Environment | Inflation | Cost Scaling | Sim Time | Recommendation |
|-------------|-----------|--------------|----------|-----------------|
| Open space | 0.3m | 5.0 | 1.0s | Aggressive, fast |
| Cluttered | 0.7m | 15.0 | 1.0s | Conservative, safe |
| Narrow corridor | 0.5m | 20.0 | 0.5s | Very cautious, short horizon |
| Fast navigation | 0.5m | 10.0 | 1.5s | Longer look-ahead |

### 3.3 Debugging Navigation Failures

**Tool: RViz Costmap Visualization**
```
View → Displays → Add → Map (costmap/costmap)
├─ Black regions: Occupied
├─ Gray: Unknown
├─ White: Free
└─ Colors between: Inflation gradient
```

**Tool: Nav2 Diagnostics**
```bash
# Monitor planner performance
ros2 run rqt_plot rqt_plot /diagnostics[0]/diagnostics

# Topics to monitor:
# - /move_base_flex/planner/frequency (should be ~20 Hz)
# - /move_base_flex/controller/frequency (should be ~100 Hz)
# - /costmap/costmap_updates (should show dynamic obstacles)
```

### 3.4 Common Failure Modes

**Oscillation near goal**:
- Cause: DWA overshoots and recovers repeatedly
- Fix: Decrease `max_vel_x` near goal

**Doesn't avoid moving obstacles**:
- Cause: Costmap update frequency too low
- Fix: Increase to 10+ Hz

**Recovery behaviors activate too often**:
- Cause: Stuck detection too sensitive
- Fix: Increase stuck detection timeout

## Layer 4: Advanced

### 4.1 Custom Recovery Behavior Plugin

Extend Nav2 with custom recovery action:

```cpp
// custom_recovery_behavior.hpp
#include "nav2_core/recovery_behavior.hpp"

class CustomRecoveryBehavior : public nav2_core::RecoveryBehavior {
public:
  void onConfigure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                   const std::string & plugin_name) override;

  nav2_core::CancelRecoveryBehavior::SharedPtr
  onRun(const std::shared_ptr<nav2_msgs::action::Spin::Goal> goal) override;
};
```

### 4.2 Replanning Strategies

When planning fails, different strategies:

```
1. Replanning with increased look-ahead:
   └─ Global planner resolves remaining goal

2. Time-elastic band (TEB):
   └─ Optimizes time and space for feasibility

3. Model Predictive Control (MPC):
   └─ Plans and controls simultaneously

Nav2 default: Replanning with behavior tree
```

## Summary

| Concept | Key Insight |
|---------|------------|
| **Dynamic obstacles** | Handled in real-time by costmap layers |
| **Inflation layer** | Creates safety margins around all obstacles |
| **DWA** | Evaluates multiple trajectories, selects safest |
| **Recovery behaviors** | Escape local minima via clear → rotate → backup |
| **Costmap decay** | Fade old sensor readings to prevent stale obstacles |

## Code Examples

### Example 1: Custom Recovery Behavior

```python
# custom_recovery_behavior.py
import rclpy
from nav2_core import RecoveryBehavior
from geometry_msgs.msg import Twist

class RotateRecovery(RecoveryBehavior):
    def __init__(self):
        self.velocity_pub = None

    def configure(self, node):
        self.velocity_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self):
        """Execute recovery: rotate 360 degrees"""
        twist = Twist()
        twist.angular.z = 0.5  # rad/s

        start_time = rclpy.get_clock().now()
        while (rclpy.get_clock().now() - start_time).nanoseconds < 6e9:  # 6 seconds
            self.velocity_pub.publish(twist)
            rclpy.spin_once()

        # Stop
        twist.angular.z = 0.0
        self.velocity_pub.publish(twist)
        return True
```

### Example 2: Dynamic Obstacle Validator

```python
# validate_obstacle_avoidance.py
import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math

class ObstacleAvoidanceValidator(rclpy.Node):
    def __init__(self):
        super().__init__('obstacle_validator')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/amcl_pose', self.pose_callback, 10)

        self.collision_count = 0
        self.obstacle_count = 0

    def scan_callback(self, msg):
        """Check LiDAR for obstacles"""
        min_range = min([r for r in msg.ranges if not math.isinf(r)])

        if min_range < 0.3:
            self.collision_count += 1
            self.get_logger().warn(f'Obstacle at {min_range:.2f}m - collision risk!')
        else:
            self.obstacle_count += 1

    def report(self):
        if self.obstacle_count > 0:
            success_rate = (1 - self.collision_count /
                          (self.collision_count + self.obstacle_count)) * 100
            self.get_logger().info(f'Avoidance success rate: {success_rate:.1f}%')
```

## Practice Exercise

**Objective**: Configure obstacle avoidance and test with dynamic obstacles

**Steps**:
1. Create Gazebo world with static obstacles
2. Configure DWA local planner with provided YAML
3. Launch Nav2 with dynamic obstacle layer enabled
4. Move moving obstacle (using Python script) toward robot
5. Verify robot avoids collision
6. Measure:
   - Distance to closest obstacle (should be >0.5m)
   - Time to react (&lt;2 seconds)
   - Avoidance success rate (>95%)

**Success criteria**:
- Robot successfully avoids dynamic obstacles
- No collisions in 10 test scenarios
- Recovery behaviors activate appropriately

## References

- **Dynamic Window Approach**: [The Dynamic Window Approach to Obstacle Avoidance](http://mobile-robots.mybluemix.net/docs/DWAapproach.pdf)
- **Nav2 Local Planner**: [DWB Controller Documentation](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- **Costmap Configuration**: [Costmap2D Guide](https://navigation.ros.org/configuration/packages/costmap-plugins.html)

## Next Steps

You now understand obstacle avoidance in dynamic environments. Choose your path:

- **→ [Lesson 8: Capstone](08-capstone-mission.md)** (Integrate complete system)
- **→ [Lesson 6: Humanoid Navigation](06-autonomous-humanoid-navigation.md)** (Advanced: handle biped constraints)
- **→ [Lesson 7: Sensor Fusion](07-multi-sensor-perception-and-fusion.md)** (Advanced: robust perception)

---

**Lesson 5 Summary**: Obstacle avoidance combines costmap inflation for safety, dynamic window approach for trajectory evaluation, and recovery behaviors for stuck detection. Proper tuning enables robots to navigate safely through dynamic environments with people and moving objects.
