# Lesson 2: Visual SLAM Systems

**Duration**: 3 hours | **Level**: CEFR B1-C2 | **Priority**: P1 | **Prerequisite**: Lesson 1

## Learning Outcomes

By the end of this lesson, you will be able to:

1. **Understand** ORB-SLAM3 architecture and feature-based visual SLAM
2. **Explain** feature detection, matching, and pose estimation
3. **Configure** ORB-SLAM3 for monocular and stereo camera setups
4. **Run** VSLAM on simulated Gazebo data
5. **Measure** and optimize odometry accuracy
6. **Debug** SLAM failures (insufficient features, lighting issues)

## Layer 1: Foundation

### 2.1 ORB-SLAM3 Architecture

**ORB** = **O**riented **R**FAST and **B**RIEF

ORB-SLAM3 is a complete visual SLAM system using ORB features. Why ORB?

- **Fast Detection**: FAST corner detection (~1000 features/frame)
- **Rotation-Invariant**: BRIEF descriptors handle camera rotation
- **Efficient**: Low computational cost, real-time performance

**Complete system** includes:
1. **Tracking**: Feature detection and matching frame-to-frame
2. **Mapping**: Accumulating features into global structure
3. **Loop Closing**: Detecting and correcting revisited locations
4. **Bundle Adjustment**: Global optimization to minimize error

### 2.2 Feature Detection Pipeline

```
Raw image
    │
    ├─→ FAST Corner Detection
    │       (identify key points)
    │
    ├─→ BRIEF Descriptor Computation
    │       (binary feature description)
    │
    └─→ Feature List
            [pt1, pt2, ..., ptN]
            (locations + descriptors)
```

**FAST (Features from Accelerated Segment Test)**:
- Detect corners by comparing pixel intensities around candidate point
- Fast because it uses simple intensity comparisons
- ~1000-2000 features per frame in textured scenes

**BRIEF (Binary Robust Independent Elementary Features)**:
- Create binary descriptor (256 bits) for each feature
- Fast to compute and match
- Rotation variant (ORB variants fix this)

### 2.3 Feature Matching

```
Frame t:  features [f1, f2, f3, ..., fn]
               │
               ├─→ Descriptor matching with Frame t+1
               │
Frame t+1: features [f'1, f'2, f'3, ..., f'm]

Result: Matched pairs = [(f1, f'3), (f2, f'1), ...]
```

**Matching strategy**:
- Compare BRIEF descriptors using Hamming distance
- For each feature in frame t, find closest match in frame t+1
- Use bidirectional matching (must match both directions)
- Use outlier rejection (RANSAC) to remove false matches

### 2.4 Pose Estimation from Feature Matches

Given matched features between consecutive frames, estimate camera motion:

```
3D point P(world) ──projection──→ p1 (in frame t)
                                  p2 (in frame t+1)

From p1 ↔ p2 correspondence, recover:
- Rotation: R (3×3 matrix)
- Translation: t (3D vector)
```

**Method**: Essential matrix decomposition
- Essential matrix E encodes both R and t
- Computed from matched feature correspondences
- Decompose E to recover R and t

### 2.5 Local Mapping

**Keyframes**: Not every frame is stored; only "keyframes" with sufficient motion/features

```
Frame sequence:
[Keyframe1] [frame2] [frame3] [Keyframe4] [frame5]
    (stored)  (used)   (used)    (stored)   (used)
```

**Why keyframes?**
- Reduces memory usage
- Speeds up processing
- Filters out static frames

**Local mapping process**:
1. New keyframe arrives
2. Add new 3D points from features without depth
3. Perform local bundle adjustment (optimize pose + structure)
4. Culling: Remove redundant points/keyframes

## Layer 2: Collaboration (ROS 2 Integration)

### 2.1 ORB-SLAM3 ROS 2 Wrapper

**Input**: Camera calibration parameters
```yaml
# camera_calibration.yaml
Camera.fx: 500.0  # focal length x (pixels)
Camera.fy: 500.0  # focal length y (pixels)
Camera.cx: 320.0  # principal point x (pixels)
Camera.cy: 240.0  # principal point y (pixels)
Camera.k1: 0.0    # distortion coefficient
```

**Camera topic interface**:
```
ROS 2 Topic: /camera/image_raw
  ├─ Type: sensor_msgs/msg/Image
  ├─ Encoding: rgb8 or mono8
  └─ Frequency: 15-30 Hz (typical)
```

**ORB-SLAM3 output topics**:
```
/orb_slam3/pose           (nav_msgs/Odometry)
  ├─ pose.pose (position + orientation)
  ├─ twist.twist (velocity estimate)
  └─ header.frame_id = "map"

/orb_slam3/trajectory     (geometry_msgs/PoseStamped array)
  ├─ All poses in trajectory
  └─ For visualization/analysis

/orb_slam3/tracked_points (sensor_msgs/PointCloud2)
  ├─ 3D positions of tracked features
  └─ Used in RViz visualization
```

### 2.2 RViz Visualization

Monitor SLAM progress in real-time:

```
RViz GUI:
├─ Map frame (map)
│  ├─ Tracked feature points (colored by depth)
│  ├─ Estimated trajectory (green)
│  └─ Ground truth trajectory (red, if available)
├─ Robot frame (base_link)
│  ├─ Current pose
│  └─ Odometry frame
└─ Camera view
   ├─ Detected features (circles)
   └─ Tracked features (blue)
```

## Layer 3: Intelligence (Configuration & Tuning)

### 3.1 Camera Calibration

**Critical**: ORB-SLAM3 requires accurate camera calibration

**Calibration parameters** (from camera matrix K):
```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]

Where:
- fx, fy = focal length (pixels)
- cx, cy = principal point (pixels)
- Distortion coefficients: k1, k2, p1, p2, k3
```

**For Gazebo simulated cameras**:
- Use ideal parameters (no distortion)
- fx = fy = focal_length (from camera plugin)
- cx = image_width / 2
- cy = image_height / 2

### 3.2 Monocular VSLAM Configuration

```yaml
# mono_settings.yaml
Camera.type: "PinHole"
Camera.width: 640
Camera.height: 480
Camera.fx: 500.0
Camera.fy: 500.0
Camera.cx: 320.0
Camera.cy: 240.0

# Feature detection
ORBextractor.nFeatures: 2000  # features per frame
ORBextractor.scaleFactor: 1.2 # image pyramid scale
ORBextractor.nLevels: 8       # pyramid levels

# Thresholds
ThFAST: 20      # FAST corner threshold
ThFASTinit: 50  # initial frame threshold

# ORB-SLAM3 specific
System.useStereo: false
System.useRGBD: false
System.useIMU: true  # use IMU if available
```

### 3.3 Stereo VSLAM Configuration

```yaml
# stereo_settings.yaml
Camera.type: "PinHole"
Camera.width: 640
Camera.height: 480

# Left camera (reference)
Camera.fx: 500.0
Camera.fy: 500.0
Camera.cx: 320.0
Camera.cy: 240.0

# Stereo baseline and other parameters
Stereo.baseline: 0.1  # baseline between cameras (meters)
Stereo.ThDepth: 40    # close/far threshold (pixels)

# Feature detection (same as mono)
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2

System.useStereo: true
```

### 3.4 Performance Optimization

**Parameter tuning for different scenarios**:

| Scenario | nFeatures | scaleFactor | ThFAST | Goal |
|----------|-----------|-------------|--------|------|
| Textured indoors | 2000 | 1.2 | 20 | Balance accuracy and speed |
| Low texture | 4000 | 1.2 | 10 | More features, lenient detection |
| High speed | 1000 | 1.3 | 30 | Fast computation, sparse features |
| High accuracy | 3000 | 1.2 | 15 | More features, careful detection |

**Computational cost**:
- Feature detection: ~30-50ms
- Feature matching: ~20-40ms
- Pose estimation: ~10-20ms
- **Total per frame**: 60-110ms @ 15Hz = reasonable

## Layer 4: Advanced

### 4.1 Loop Closure Detection

**Problem**: After 10 minutes of navigation, how does SLAM know if it has returned to a known location?

**Solution**: Place recognition using visual similarity

```
Loop closure pipeline:
├─ Current image
│  ├─ Extract features
│  └─ Compute feature vector ("bag of words")
│
├─ Query database of past keyframe bags
│  └─ Find keyframes with similar bags
│
├─ Candidate matches found?
│  └─ YES → Perform geometric verification
│       ├─ Match features between current and candidate
│       ├─ Compute similarity transform
│       └─ If match strong → declare loop closure
│
└─ Loop closure correction:
   ├─ Optimize pose graph
   ├─ Correct all accumulated drift
   └─ Fuse with previous estimate
```

**Why geometric verification is crucial**:
- Perceptual aliasing: Different places look similar
- Need to verify with feature matches, not just appearance

### 4.2 Inertial Measurement Unit (IMU) Integration

ORB-SLAM3 can fuse camera + IMU:

**Benefits**:
- IMU provides immediate angular velocity information
- Helps when visual features insufficient (motion blur, low light)
- Enables scale recovery in monocular mode

**Challenge**: Temporal synchronization
- Camera and IMU have different timestamp frequencies
- Must align on common time reference

## Summary

| Concept | Key Insight |
|---------|------------|
| **ORB features** | Fast to detect/match, rotation-invariant, enable real-time SLAM |
| **Keyframes** | Not every frame is stored; only keyframes reduce computation |
| **Feature matching** | Descriptor matching + RANSAC outlier rejection |
| **Pose estimation** | Essential matrix decomposition from matched features |
| **Loop closure** | Place recognition + geometric verification to correct drift |
| **IMU fusion** | Improves robustness in challenging conditions |

## Code Examples

### Example 1: Configure ORB-SLAM3 for Gazebo

```python
# code_examples/orb_slam3_config_generator.py
import yaml

def generate_mono_config():
    """Generate ORB-SLAM3 config for monocular Gazebo camera"""
    config = {
        'Camera': {
            'type': 'PinHole',
            'width': 640,
            'height': 480,
            'fx': 500.0,
            'fy': 500.0,
            'cx': 320.0,
            'cy': 240.0,
        },
        'ORBextractor': {
            'nFeatures': 2000,
            'scaleFactor': 1.2,
            'nLevels': 8,
        },
        'ThFAST': 20,
        'System': {
            'useStereo': False,
            'useRGBD': False,
            'useIMU': False,
        }
    }

    with open('mono_settings.yaml', 'w') as f:
        yaml.dump(config, f)
    return config

if __name__ == '__main__':
    generate_mono_config()
    print("Config saved to mono_settings.yaml")
```

### Example 2: Analyze Loop Closure Performance

```python
# code_examples/analyze_loop_closure.py
import rclpy
from geometry_msgs.msg import PoseStamped
import numpy as np

class LoopClosureAnalyzer(rclpy.Node):
    def __init__(self):
        super().__init__('loop_closure_analyzer')
        self.poses = []
        self.loop_closure_count = 0

        # Subscribe to poses
        self.create_subscription(PoseStamped, '/orb_slam3/trajectory',
                                self.pose_callback, 10)
        self.create_timer(5.0, self.analyze_loop_closures)

    def pose_callback(self, msg):
        pose = msg.pose.position
        self.poses.append([pose.x, pose.y, pose.z])

    def analyze_loop_closures(self):
        """Detect loop closures by analyzing trajectory"""
        if len(self.poses) < 20:
            return

        poses_array = np.array(self.poses)
        current = poses_array[-1]

        # Find past poses within 0.5m distance
        distances = np.linalg.norm(poses_array[:-20] - current, axis=1)
        close_poses = np.where(distances < 0.5)[0]

        if len(close_poses) > 0:
            first_close = close_poses[0]
            time_since = len(poses_array) - first_close
            self.get_logger().info(
                f'Potential loop closure detected! '
                f'Revisited pose from {time_since} frames ago'
            )
            self.loop_closure_count += 1
```

## Practice Exercise

**Objective**: Configure ORB-SLAM3 and measure tracking performance

**Steps**:
1. Create Gazebo world with diverse features (walls, textures, objects)
2. Generate camera calibration config for your Gazebo camera
3. Launch ORB-SLAM3: `ros2 launch code_examples/orb_slam3_launch.py`
4. Record SLAM output to ROS bag
5. Analyze:
   - Number of tracked features per frame
   - Loop closure detections
   - Pose estimation accuracy

**Success criteria**:
- >500 features tracked continuously
- Smooth trajectory (no sudden jumps)
- Loop closures detected when revisiting areas
- ATE &lt;5% of path length

## References

- **ORB-SLAM3 Paper**: [ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM](https://arxiv.org/abs/2007.11898)
- **Feature Detection**: [FAST Corner Detection](https://www.edwardrosten.com/work/fast.html)
- **BRIEF Descriptors**: [BRIEF: Computing a Local Binary Descriptor Very Fast](https://www.cs.ubc.ca/~lowe/papers/calonder_eccv10.pdf)

## Next Steps

You now understand the architecture and tuning of ORB-SLAM3. Choose your next lesson:

- **→ [Lesson 4: Nav2 Path Planning](04-nav2-path-planning-stack.md)** (Core: use SLAM for navigation)
- **→ [Lesson 7: Sensor Fusion](07-multi-sensor-perception-and-fusion.md)** (Advanced: combine SLAM with LiDAR/IMU)

---

**Lesson 2 Summary**: ORB-SLAM3 provides complete visual SLAM using ORB features, keyframe tracking, and loop closure detection. Understanding feature detection, matching, pose estimation, and loop closure enables you to configure and debug VSLAM systems in diverse scenarios.
