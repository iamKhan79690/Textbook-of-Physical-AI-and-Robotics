# Lesson 4: Nav2 Path Planning Stack

**Duration**: 3 hours | **Level**: Unlimited | **Priority**: P1 | **Prerequisite**: Lesson 1

## Learning Outcomes

By the end of this lesson, you will be able to:

1. **Explain** Nav2 architecture and the multi-level planning approach
2. **Configure** costmaps with static, obstacle, and inflation layers
3. **Understand** global planner algorithms (RRT, Theta*, graph-based) and their trade-offs
4. **Implement** local trajectory planning with dynamic window approach (DWA)
5. **Tune** Nav2 parameters for different robot types and environments
6. **Debug** planning failures using RViz and diagnostics

## Layer 1: Foundation

### 4.1 Navigation Pipeline Overview

Autonomous navigation requires solving the path planning problem at multiple time scales:

```
Goal: Robot must reach destination while avoiding obstacles

┌─ Global Planning (slow, far-looking)
│  ├─ Input: Full occupancy grid, start, goal
│  ├─ Output: Reference path through free space
│  ├─ Frequency: 10-20 Hz (when goal changes)
│  └─ Horizon: Entire environment
│
├─ Local Planning (fast, near-looking)
│  ├─ Input: Local costmap, current pose
│  ├─ Output: Velocity commands
│  ├─ Frequency: 100 Hz
│  └─ Horizon: 1-5 meters
│
└─ Control (lowest-level)
   ├─ Input: Velocity commands
   ├─ Output: Motor commands
   └─ Frequency: 1000+ Hz
```

### 4.2 The Nav2 Stack

Nav2 (Navigation2) is the ROS 2 standard navigation framework:

```
Nav2 Components:
├─ Costmap2D (static + dynamic obstacle representation)
├─ GlobalPlanner (compute path from start to goal)
├─ LocalPlanner (generate safe velocity commands)
├─ BehaviorTree (high-level mission orchestration)
├─ Recovery behaviors (stuck detection + recovery)
└─ Lifecycle manager (startup/shutdown coordination)
```

**Why multi-level?**
- **Efficiency**: Global planner doesn't need to run every millisecond
- **Responsiveness**: Local planner reacts to dynamic obstacles quickly
- **Robustness**: If local planner fails, recovery behaviors activate

### 4.3 Costmap Representation

A **costmap** encodes navigable vs. non-navigable space:

```
Occupancy grid:
[0] = free space (white)
[254] = occupied (black)
[255] = unknown (gray)

But costmaps add gradients for safety:
0-50: Free space
51-127: Cautionary region (approaching obstacles)
128-252: Lethal region (collision likely)
254: Unknown
255: Lethal (definite obstacle)
```

**Three-layer costmap architecture**:

```
Layer 1: Static map layer
├─ Loaded from map file (.pgm)
├─ Represents permanent obstacles
└─ Never changes during execution

Layer 2: Obstacle layer
├─ Populated from sensor data (LiDAR, camera)
├─ Detects dynamic obstacles
└─ Updated every sensor frame

Layer 3: Inflation layer
├─ Expands obstacles by robot radius
├─ Creates safety margin
├─ Inflated obstacles prevent collision with sharp edges
└─ Inflation_radius = robot_radius + safety_margin
```

### 4.4 Global Planning Algorithms

#### RRT (Rapidly-Exploring Random Tree)

```
1. Start with tree containing only start position
2. Loop:
   a. Sample random point in space
   b. Find nearest node in tree
   c. Extend tree toward sampled point
   d. If new node valid (no collision), add to tree
   e. Check if goal reached
3. When goal found, backtrack to get path
```

**Advantages**: Works in complex, high-dimensional spaces
**Disadvantages**: Path may be suboptimal, non-deterministic

#### RRT* (RRT-Star)

Improvement over RRT:

```
Same as RRT, but with optimization:
c. After extending, check nearby nodes
d. Rewire: If path through new node shorter, update
e. Gradually optimizes path as algorithm runs
```

**Advantage**: Asymptotically optimal path
**Disadvantage**: More computation per iteration

#### Theta* (Theta-Star)

Graph-based planner with line-of-sight optimization:

```
1. Discretize environment into grid
2. Build graph with grid cells as nodes
3. Use A* search to find shortest path
4. Post-process: Check if line-of-sight shortcuts exist
5. Use shortcuts if valid
```

**Advantages**: Deterministic, smooth paths, fast
**Disadvantages**: Grid-dependent, may miss narrow passages

### 4.5 Local Planning: Dynamic Window Approach (DWA)

DWA generates safe velocity commands by evaluating trajectories:

```
DWA Algorithm:
1. Current state: position, velocity
2. Predict robot motion for 0.5-1.0 seconds with various velocities
3. For each predicted trajectory:
   a. Calculate distance to obstacles
   b. Calculate progress toward goal
   c. Compute trajectory cost (weighted sum)
4. Select lowest-cost trajectory
5. Execute first velocity command
6. Repeat next cycle
```

**Why it works**:
- Considers dynamic constraints (max acceleration, velocity limits)
- Naturally handles narrow spaces (curves if needed)
- Runs at 100+ Hz for reactive obstacle avoidance

## Layer 2: Collaboration (ROS 2 Integration)

### 2.1 Nav2 ROS 2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/map` | nav_msgs/OccupancyGrid | Out | Static occupancy grid |
| `/costmap/costmap` | nav_msgs/OccupancyGrid | Out | Combined costmap |
| `/move_base/global_plan` | nav_msgs/Path | Out | Planned path from start to goal |
| `/move_base/local_plan` | nav_msgs/Path | Out | Local path (next 1-2m) |
| `/goal_pose` | geometry_msgs/PoseStamped | In | Navigation goal |
| `/cmd_vel` | geometry_msgs/Twist | Out | Velocity commands |

### 2.2 Nav2 Configuration Structure

```yaml
# nav2_config.yaml
nav2_bringup:
  bt_xml_filename: "navigate_w_replanning_and_recovery.xml"  # behavior tree
  use_lifecycle_mgr: true

  planner_server:
    ros__parameters:
      expected_planner_frequency: 20.0  # Hz
      plugins: ["GridBased"]
      GridBased:
        plugin: "nav2_navfn_planner/NavfnPlanner"
        tolerance: 0.5  # distance to goal before success
        use_astar: true  # use A* instead of Dijkstra

  controller_server:
    ros__parameters:
      expected_controller_frequency: 20.0  # Hz
      plugins: ["FollowPath"]
      FollowPath:
        plugin: "nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController"
        desired_linear_vel: 0.5  # m/s
        max_angular_vel: 1.57  # rad/s (~90 deg/s)

  costmap:
    costmap:
      ros__parameters:
        update_frequency: 5.0  # Hz
        publish_frequency: 2.0  # Hz
        global_frame: map
        robot_base_frame: base_link
        use_sim_time: true

        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

        static_layer:
          plugin: "nav2_costmap_2d::StaticLayer"
          map_subscribe_transient_local: true

        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: true
          observation_sources: scan
          scan:
            topic: /scan  # LiDAR topic
            max_obstacle_height: 2.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
            raytrace_max_range: 3.0

        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          inflation_radius: 0.55  # expand obstacles by 0.55m
          cost_scaling_factor: 10.0  # slope of inflation gradient
```

### 2.3 Launch Nav2

```python
# nav2_launch.py
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Load configuration file
    nav2_config = os.path.join(nav2_bringup_dir, 'nav2_params.yaml')

    # Create launch description
    ld = launch.LaunchDescription()

    # Start Nav2 bringup
    ld.add_action(
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            arguments=[
                '--use_sim_time', 'true',
                '--params_file', nav2_config,
            ],
        )
    )

    return ld
```

## Layer 3: Intelligence (Tuning and Optimization)

### 3.1 Costmap Tuning

| Parameter | Effect | Typical Value |
|-----------|--------|---------------|
| `inflation_radius` | Safety distance around obstacles | 0.5-1.0m |
| `cost_scaling_factor` | Gradient steepness | 5-20 |
| `update_frequency` | How often costmap updates | 5-10 Hz |
| `obstacle_max_range` | Sensor range for obstacles | 2.5-5.0m |
| `raytrace_max_range` | Clear cells beyond this distance | 3.0-6.0m |

**Tuning strategy**:
- Start with large `inflation_radius` (conservative)
- Reduce if robot can navigate tight spaces
- Increase `cost_scaling_factor` if robot hugs obstacles too closely

### 3.2 Planner Selection

| Planner | Best For | Speed | Optimality |
|---------|----------|-------|-----------|
| **NavFn (Dijkstra)** | Familiar layouts, real-time | Fast | Suboptimal |
| **Theta*** | Smooth paths, open spaces | Medium | Near-optimal |
| **RRT** | Complex obstacles | Slow | Varies |
| **RRT*** | Guaranteed optimality | Very Slow | Optimal |

**Recommendation**: Use Theta* for balance of speed and quality

### 3.3 Local Controller Tuning

For Regulated Pure Pursuit (common local planner):

```yaml
RegulatedPurePursuitController:
  desired_linear_vel: 0.5  # Target velocity
  lookahead_dist: 0.6  # How far ahead to aim
  min_approach_linear_vel: 0.05  # Slow down approaching goal
  max_allowed_time_to_collision_up_to_carrot: 1.0  # Emergency stop time
  use_velocity_scaled_lookahead_dist: true  # Adaptive lookahead
```

**Tuning**:
- Increase `desired_linear_vel` for faster navigation
- Increase `lookahead_dist` for smoother paths (but less responsive)
- Decrease `max_allowed_time_to_collision_up_to_carrot` for faster reaction

### 3.4 Debugging Planning Failures

**Issue: "No path found"**
- Check: Is goal in free space? (use costmap viewer)
- Solution: Ensure inflation_radius not too large

**Issue: Robot oscillates near goal**
- Check: `min_approach_linear_vel` too high
- Solution: Decrease to 0.05 or lower

**Issue: Planner too slow**
- Check: Environment too large or planner too conservative
- Solution: Switch to faster planner (Theta* vs. RRT)

## Layer 4: Advanced

### 4.1 Behavior Trees for Mission Control

Nav2 uses behavior trees (BT) for high-level logic:

```xml
<!-- navigate_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <!-- Compute path -->
      <ComputePath name="compute_path" goal="{goal}" path="{path}" planner_id="GridBased"/>

      <!-- Follow path with replanning -->
      <FollowPath name="follow_path" path="{path}" controller_id="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Behavior trees enable:
- Replanning when path becomes invalid
- Recovery behaviors (rotate, backup)
- Goal switching without recompilation

### 4.2 Custom Global Planner Plugin

Extend Nav2 with custom planning algorithm:

```cpp
// custom_planner.hpp
#include "nav2_core/global_planner.hpp"

class CustomPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
};
```

## Summary

| Concept | Key Insight |
|---------|------------|
| **Multi-level planning** | Global (slow, far) + Local (fast, reactive) |
| **Costmaps** | Three-layer: static + obstacles + inflation |
| **RRT/Theta*** | Different trade-offs between speed and optimality |
| **DWA** | Evaluates trajectories, not just steering angles |
| **Tuning** | Start conservative, gradually loosen constraints |

## Code Examples

### Example 1: Nav2 Configuration for Differential Drive

```yaml
# nav2_differential_drive_config.yaml
nav2_bringup:
  planner_server:
    ros__parameters:
      plugins: ["GridBased"]
      GridBased:
        plugin: "nav2_navfn_planner/NavfnPlanner"
        tolerance: 0.5
        use_astar: true

  controller_server:
    ros__parameters:
      plugins: ["FollowPath"]
      FollowPath:
        plugin: "nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController"
        desired_linear_vel: 0.5
        max_angular_vel: 1.57
        lookahead_dist: 0.6
        use_velocity_scaled_lookahead_dist: true

  costmap:
    costmap:
      ros__parameters:
        robot_radius: 0.25
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        obstacle_layer:
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
        inflation_layer:
          inflation_radius: 0.55
          cost_scaling_factor: 10.0
```

### Example 2: Goal Publisher

```python
# nav2_goal_publisher.py
import rclpy
from geometry_msgs.msg import PoseStamped

def publish_goal(x, y, yaw):
    node = rclpy.create_node('goal_publisher')
    pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = sin(yaw/2)
    goal.pose.orientation.w = cos(yaw/2)

    pub.publish(goal)
    print(f"Goal published: ({x}, {y})")
```

## Practice Exercise

**Objective**: Configure Nav2 and navigate a robot to multiple waypoints

**Steps**:
1. Create a Gazebo world with obstacles
2. Write a static map from Gazebo world
3. Configure Nav2 with provided YAML template
4. Launch Nav2 + robot simulator
5. Publish 5 goal poses sequentially
6. Verify:
   - Costmap displays obstacles correctly
   - Robot plans paths around obstacles
   - Navigation completes in &lt;10 seconds per goal

**Success criteria**:
- All 5 goals reached successfully
- Paths are smooth and collision-free
- Nav2 diagnostics show no errors

## References

- **Nav2 Documentation**: [ROS 2 Nav2 Docs](https://navigation.ros.org/)
- **Path Planning Survey**: [Motion Planning Survey (LaValle)](http://planning.cs.umn.edu/)
- **RRT Algorithm**: [Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1105.1186)

## Next Steps

You now understand Nav2 path planning architecture and configuration. Choose your path:

- **→ [Lesson 5: Obstacle Avoidance](05-obstacle-avoidance-and-dynamic-environments.md)** (Handle dynamic obstacles)
- **→ [Lesson 8: Capstone](08-capstone-mission.md)** (Integrate full system)

---

**Lesson 4 Summary**: Nav2 provides a mature, ROS 2-native framework for autonomous navigation using multi-level planning (global + local). Understanding costmaps, planner algorithms, and parameter tuning enables you to deploy robust navigation in diverse environments.
