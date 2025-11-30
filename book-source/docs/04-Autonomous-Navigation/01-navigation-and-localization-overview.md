# Lesson 1: Navigation and Localization Overview

**Duration**: 2.5 hours | **Level**: CEFR B1-C2 | **Priority**: P1

## Learning Outcomes

By the end of this lesson, you will be able to:

1. **Explain** SLAM algorithms and their role in autonomous navigation
2. **Distinguish** between localization, mapping, and odometry
3. **Describe** the differences between monocular and stereo visual SLAM
4. **Understand** sensor requirements for VSLAM systems
5. **Navigate** basic ROS 2 SLAM systems and verify odometry output
6. **Identify** loop closure detection and its importance

## Layer 1: Foundation

### 1.1 What is Autonomous Navigation?

**Definition**: Autonomous navigation is the ability of a robot to move from point A to point B without human control, making intelligent decisions to avoid obstacles and reach its destination.

**Key components**:
- **Localization**: Knowing where you are
- **Mapping**: Understanding your environment
- **Planning**: Deciding how to get somewhere
- **Control**: Executing planned motions

In Chapters 1-2, you learned about robots, sensors, and simulation. Now we bring it together: **autonomous navigation** is the integration of all these components into a system that can move intelligently.

### 1.2 The Three Hard Problems of Autonomous Navigation

#### Problem 1: Localization (Where am I?)

Traditional robots use:
- **GPS**: Works outdoors, fails indoors/urban canyons
- **Odometry**: Tracks wheel rotation, but drifts over time
- **Compass**: Unreliable due to magnetic interference

**Visual SLAM solution**: Use camera images + feature matching to estimate robot position while simultaneously building a map.

#### Problem 2: Mapping (What's around me?)

- Raw sensor data (point clouds, images) is high-dimensional and noisy
- Must represent obstacles for planning (costmaps)
- Must handle dynamic obstacles (moving objects)

**Solution**: Process sensor data into structured maps:
- **Occupancy grids**: Each cell is occupied (1) or free (0)
- **Costmaps**: Extend occupancy grids with "cost" for safety margins
- **Semantic maps**: Include object labels and properties

#### Problem 3: Planning (How do I get there?)

- **Global planning**: Find a path from start to goal
- **Local planning**: Avoid obstacles while executing the path
- Both must be fast (replanning in &lt;2 seconds)

**Solution**: Multi-level planning with different time horizons:
- **Global planner**: Runs every 10-20 seconds on full costmap
- **Local planner**: Runs every 100ms on immediate neighborhood

### 1.3 SLAM: The Elegant Solution

**SLAM** = **S**imultaneous **L**ocalization **A**nd **M**apping

**The Insight**: Instead of solving localization and mapping separately, solve them together!

**How it works**:
1. Robot moves and observes visual features (corners, edges, textures)
2. Match features between consecutive images
3. Estimate robot motion from feature matches
4. Accumulate features into a global map
5. When revisiting known areas, use loop closure to correct drift

**Why it works**:
- Camera provides rich information about the environment
- Feature matching is fast and reliable
- Loop closure detection prevents unbounded drift
- Works without pre-built maps

**Visual SLAM systems** use only camera input:
- ORB-SLAM3 (our choice): Feature-based, robust, real-time
- DSLAM: Direct visual odometry
- COLMAP: Offline photogrammetry

### 1.4 Visual Odometry vs. Full SLAM

#### Visual Odometry (VO)
- Estimates robot motion frame-to-frame
- Accumulates drift over long paths
- Lightweight, works in real-time
- **Equation**: position(t) = position(t-1) + delta_motion(t)

#### Visual SLAM (full SLAM)
- Includes global optimization with loop closure
- Corrects drift when revisiting locations
- Heavier computation, but more accurate
- **Benefit**: Long-term accuracy without drift

**Real-world analogy**:
- **VO**: Taking steps and counting, useful for short paths
- **SLAM**: Remembering landmarks and correcting when you see them again

### 1.5 Monocular vs. Stereo VSLAM

#### Monocular SLAM
- **Input**: Single camera stream
- **Strengths**: Lightweight, single camera on most robots
- **Weakness**: No absolute scale (can't tell if object is big and far or small and close)
- **Solution**: Use wheel odometry to learn scale

```
Pixel difference ─→ Feature motion
                    (angle only, not distance)
```

#### Stereo SLAM
- **Input**: Two synchronized cameras
- **Strengths**: Absolute scale from stereo baseline
- **Weakness**: Requires stereo calibration, more computation
- **Benefit**: Scale known without odometry

```
Left image + Right image ─→ Stereo disparity ─→ Depth ─→ Scale information
```

**Trade-off**:
- Monocular: Fast, simple, needs odometry fusion
- Stereo: Accurate, complex setup, works standalone

## Layer 2: Collaboration (ROS 2 Integration)

### 2.1 SLAM in ROS 2

ORB-SLAM3 can run as a **ROS 2 node**:

```
┌─────────────────────────────────────────┐
│ Gazebo Simulator                        │
│  ├─ /gazebo/image_raw (camera topic)  │
│  ├─ /gazebo/odom (wheel odometry)     │
│  └─ /gazebo/tf (ground truth frames)  │
└──────────────┬──────────────────────────┘
               │ publishes camera images
               ▼
         ┌──────────────┐
         │ ORB-SLAM3    │ ROS 2 wrapper
         │ ROS 2 Node   │
         └──────┬───────┘
                │ publishes:
                ├─ /orb_slam3/pose (estimated position)
                ├─ /orb_slam3/pose_covariance
                ├─ /orb_slam3/map (feature map)
                └─ /orb_slam3/tracked_features
               │ subscribes:
               └─ /camera/image_raw (input)
```

### 2.2 Key Topics and Services

| Topic | Type | Direction | Content |
|-------|------|-----------|---------|
| `/camera/image_raw` | sensor_msgs/Image | In | Camera image from Gazebo |
| `/odom` | nav_msgs/Odometry | Out | Robot pose and velocity estimates |
| `/tf` | tf2_msgs/TFMessage | Out | Frame transformations (map → odom → base_link) |

### 2.3 Coordinate Frames

SLAM establishes several frames:

```
map  ─────────(T_map_odom)──────► odom  ─────────(T_odom_base)──────► base_link
  (global)                         (drift)                              (robot)
```

- **map**: Global frame (SLAM output, doesn't drift long-term)
- **odom**: Odometry frame (drifts over time, wheel encoder origin)
- **base_link**: Robot center (moves with robot)

**Loop closure effect**:
```
After loop closure detection:
T_map_odom gets updated to correct accumulated drift
```

## Layer 3: Intelligence (Tuning and Optimization)

### 3.1 Loop Closure Detection

**Problem**: Over long paths, odometry drifts. How do we know when we've returned to a known location?

**Solution**: Feature matching and place recognition.

**How it works**:
1. Store visual features and their 3D positions
2. When new features appear, compare to stored features
3. If match found with features from earlier position → loop closure!
4. Optimize global pose graph to correct drift

**Importance**:
- Without loop closure: drift unbounded
- With loop closure: long-term accuracy maintained

**Real-world example**:
- Robot explores a warehouse floor plan (200m path)
- After 200m, robot returns to start location
- Camera sees the starting room again
- Features matched with stored features from time t=0
- SLAM system corrects accumulated drift

### 3.2 Tuning SLAM Parameters

Key parameters for ORB-SLAM3:

| Parameter | Effect | Range | Typical |
|-----------|--------|-------|---------|
| `feature_threshold` | Number of features detected | 0.01-0.1 | 0.05 |
| `min_matches` | Minimum matches between frames | 5-50 | 20 |
| `scale_factor` | Image pyramid scale | 1.1-1.3 | 1.2 |
| `num_levels` | Pyramid levels | 1-8 | 4 |

**Tuning strategy**:
1. Start with defaults
2. If low odometry accuracy → increase `min_matches`
3. If features missed → decrease `feature_threshold`
4. If computational expensive → reduce `num_levels`

### 3.3 Accuracy Metrics

How do we measure SLAM performance?

**Absolute Trajectory Error (ATE)**:
```
ATE = sqrt(mean((estimated_pose - ground_truth_pose)²))
```
Measures overall localization accuracy. Target: &lt;5% of path length.

**Relative Pose Error (RPE)**:
```
RPE = measured between poses at t and t+Δt
```
Measures local odometry quality. Target: &lt;2% of distance traveled.

**Loop Closure Precision**:
```
% of correctly detected loop closures
```
Target: >95% true positives, &lt;5% false positives.

## Layer 4: Advanced (Research Frontiers)

### 4.1 Beyond ORB-SLAM3

**Deep Learning SLAM**:
- Neural networks to replace hand-crafted features
- Advantages: Works in challenging conditions (low light, motion blur)
- Disadvantages: Requires large training datasets

**LiDAR-based SLAM**:
- Uses laser range finder instead of camera
- Advantages: Range information, works in darkness
- Disadvantages: Sparse, requires rotation for full range

**Multi-sensor SLAM**:
- Combines camera + LiDAR + IMU
- Better robustness and accuracy
- More complex state management

### 4.2 Loop Closure in Real-World Systems

Real challenges:
- **Appearance change**: Same location looks different (time of day, weather, dynamic objects)
- **Perceptual aliasing**: Different locations look similar
- **Large rotations**: Detecting loops after 180° turn difficult

Modern solutions:
- Place recognition networks (e.g., NetVLAD)
- Geometric consistency checks
- Temporal filtering to reject transient loop closure detections

## Summary

| Concept | Key Insight |
|---------|-------------|
| **Autonomous Navigation** | Integration of localization, mapping, planning, and control |
| **SLAM** | Simultaneous localization and mapping solves drift problem |
| **Visual Odometry** | Monocular frame-to-frame motion, accumulates drift |
| **Visual SLAM** | Adds loop closure for long-term accuracy |
| **Monocular** | Fast, needs scale calibration |
| **Stereo** | Accurate, requires synchronization and calibration |
| **Loop Closure** | Critical for long-term accuracy in large environments |

## Self-Assessment

Check your understanding:

- [ ] Can you explain what SLAM stands for and what it solves?
- [ ] Do you understand the difference between localization and odometry?
- [ ] Can you explain monocular vs. stereo trade-offs?
- [ ] Do you know why loop closure detection is important?
- [ ] Can you describe the map → odom → base_link frame hierarchy?

**Full assessment**: See [SLAM Comprehension Checklist](../checklists/slam-comprehension.md)

## Code Examples

### Example 1: Launch ORB-SLAM3 on Gazebo

```python
# code_examples/orb_slam3_launch.py
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # Gazebo simulator
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['../../gazebo_worlds/flat_navigation_world.world'],
        ),
        # ORB-SLAM3 ROS 2 wrapper
        Node(
            package='orb_slam3_ros2',
            executable='mono_inertial_node',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'vocab_file': '/path/to/ORBvoc.txt',
                'settings_file': '/path/to/settings.yaml',
            }],
        ),
    ])
```

### Example 2: Verify Odometry Accuracy

```python
# code_examples/slam_accuracy_checker.py
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class SLAMValidator(rclpy.Node):
    def __init__(self):
        super().__init__('slam_validator')
        self.slam_poses = []
        self.ground_truth_poses = []

        # Subscribe to SLAM output
        self.create_subscription(
            Odometry, '/orb_slam3/pose', self.slam_callback, 10)

        # Subscribe to ground truth from Gazebo
        self.create_subscription(
            PoseStamped, '/gazebo/model_states', self.gt_callback, 10)

    def slam_callback(self, msg):
        pose = msg.pose.pose
        self.slam_poses.append([pose.position.x, pose.position.y])
        self.compute_ate()

    def compute_ate(self):
        # Absolute Trajectory Error
        if len(self.slam_poses) > 10:
            ate = np.sqrt(np.mean(
                (np.array(self.slam_poses) -
                 np.array(self.ground_truth_poses))**2
            ))
            self.get_logger().info(f'ATE: {ate:.4f}m')
```

## Practice Exercise

**Objective**: Run SLAM on Gazebo and verify odometry output

**Steps**:
1. Launch the provided Gazebo world: `gazebo gazebo_worlds/flat_navigation_world.world`
2. Start ORB-SLAM3: `ros2 launch code_examples/orb_slam3_launch.py`
3. Record the odometry: `ros2 bag record /orb_slam3/pose`
4. Move robot through environment using `/gazebo/cmd_vel` publisher
5. After 5 minutes, analyze odometry accuracy using `slam_accuracy_checker.py`
6. **Success criterion**: ATE < 5% of total path length

## References

- **ORB-SLAM3**: [GitHub Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **Visual SLAM Survey**: [Introduction to Visual SLAM (Cyrill Stachniss)](https://www.youtube.com/playlist?list=PLgnQpQmwLSnxIQmHKzIUM2Ijy6J3-Qt5T)
- **ROS 2 SLAM Setup**: [Nav2 SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

## Next Steps

Now that you understand SLAM fundamentals and localization:

1. **Advanced option**: Proceed to [Lesson 2: Visual SLAM Systems](02-visual-slam-systems.md) for deeper technical details on ORB-SLAM3
2. **Core path**: Jump to [Lesson 4: Nav2 Path Planning](04-nav2-path-planning-stack.md) to learn how to plan paths using SLAM for localization
3. **Optional**: Explore [Lesson 3: Isaac Sim](03-introduction-to-isaac-sim.md) for photorealistic simulation alternatives

---

**Lesson 1 Summary**: You now understand SLAM as the solution to autonomous navigation's fundamental challenge: simultaneously localizing and mapping. You know the differences between visual odometry and full SLAM, understand monocular vs. stereo trade-offs, and recognize loop closure detection's critical importance.

**Ready for the next lesson?** Choose your path:
- **→ [Lesson 2: Visual SLAM Systems](02-visual-slam-systems.md)** (detailed SLAM algorithms)
- **→ [Lesson 4: Nav2 Path Planning](04-nav2-path-planning-stack.md)** (core curriculum)
- **→ [Lesson 3: Isaac Sim](03-introduction-to-isaac-sim.md)** (advanced simulation)
