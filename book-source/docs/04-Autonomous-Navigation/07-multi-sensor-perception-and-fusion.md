# Lesson 7: Multi-Sensor Perception and Fusion

**Duration**: 3 hours | **Level**: Unlimited | **Priority**: P3 | **Prerequisite**: Lesson 1 or 3

## Learning Outcomes

By the end of this lesson, you will be able to:

1. **Explain** sensor characteristics (camera, LiDAR, IMU) and their trade-offs
2. **Implement** temporal synchronization using ROS 2 message_filters
3. **Design** covariance-weighted sensor fusion strategies
4. **Measure** accuracy improvements from multi-sensor fusion (>20% target)
5. **Handle** sensor failures and gracefully degrade

## Layer 1: Foundation

### 7.1 Sensor Characteristics

#### Camera (Visual)
```
Strengths:
├─ Rich visual information (features, colors, textures)
├─ Works well in daylight/indoor with good lighting
└─ Scalable (multiple cameras cheap)

Weaknesses:
├─ Fails in darkness or low contrast
├─ Motion blur at high speed
├─ Computationally expensive feature extraction
└─ Sensitive to lighting changes
```

**Good for**: Feature-based SLAM, object detection

#### LiDAR (Light Detection and Ranging)
```
Strengths:
├─ Robust distance measurement (works day/night)
├─ 3D point cloud, good range (5-30m)
├─ Less affected by lighting
└─ Direct depth, no scale ambiguity

Weaknesses:
├─ Sparse point clouds (vs. dense images)
├─ Expensive hardware
├─ Doesn't capture color/texture
└─ Limited by range and reflectivity
```

**Good for**: Range sensing, obstacle detection

#### IMU (Accelerometer + Gyroscope)
```
Strengths:
├─ Immediate motion feedback (no latency)
├─ Works anywhere (indoors/outdoors, dark/light)
├─ Small, cheap, lightweight
└─ No external dependencies

Weaknesses:
├─ Accelerometer drifts (integration error accumulates)
├─ Gyroscope drifts (bias accumulates)
├─ Noisy measurements
└─ Can't estimate absolute position alone
```

**Good for**: Motion tracking, orientation

### 7.2 Sensor Fusion Motivation

**Single sensor limitations**:
```
Camera in dark room:
├─ Can't see features → SLAM fails
└─ Output: No pose estimate

LiDAR in featureless corridor:
├─ All similar point clouds → ambiguous
└─ Output: Multiple possible poses

IMU alone:
├─ Drifts over time
└─ Output: Rough estimate, diverges
```

**Fused sensors**:
```
Camera + LiDAR + IMU:
├─ Camera: rich features (when light available)
├─ LiDAR: robust range (works day/night)
├─ IMU: immediate motion feedback
└─ Output: Robust pose with 20%+ accuracy improvement
```

### 7.3 Fusion Architectures

**Tightly-coupled fusion**:
- All sensors feed into single optimizer
- Best accuracy, most complex
- Example: Visual-inertial SLAM

**Loosely-coupled fusion**:
- Each sensor produces independent pose estimate
- Estimates combined via weighted average
- Simpler, sufficient for most applications
- **Our approach**: Loosely-coupled with covariance weighting

```
Visual odometry → pose (with covariance)
                      │
                      ├→ Weighted fusion → final pose
                      │
LiDAR odometry → pose (with covariance)
                      │
IMU-based estimate → orientation (with covariance)
```

### 7.4 Covariance-Weighted Fusion

Each sensor produces:
- **State estimate**: Position and orientation
- **Covariance**: Uncertainty matrix (how confident?)

**Fusion rule** (Kalman filter-like):

```
Measurement 1: pose_1, cov_1 (high confidence, low covariance)
Measurement 2: pose_2, cov_2 (low confidence, high covariance)

Fused estimate = (cov_2*pose_1 + cov_1*pose_2) / (cov_1 + cov_2)

Intuition: Weight measurements by inverse covariance
```

## Layer 2: Collaboration (ROS 2 Integration)

### 2.1 Temporal Synchronization

**Challenge**: Sensors have different frequencies and latencies

```
Camera @ 30 Hz:    [    @0.1s   @0.2s   @0.3s   ]
LiDAR @ 10 Hz:     [   @0.0s            @0.2s  ]
IMU @ 100 Hz:      [@ @ @ @ @ @ @ @ @ @ @ @ @ @ ]

How do we combine at same time?
```

**Solution**: `message_filters` with exact/approximate time synchronization

### 2.2 Using message_filters for Synchronization

```python
# code_examples/temporal_sync_handler.py
import rclpy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu

class SensorFusionNode(rclpy.Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Create subscribers
        visual_sub = Subscriber(self, PoseStamped, '/camera/odometry')
        lidar_sub = Subscriber(self, PoseStamped, '/lidar/odometry')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        # Synchronize messages
        # ApproximateTimeSynchronizer allows time slew (0.1s tolerance)
        sync = ApproximateTimeSynchronizer(
            [visual_sub, lidar_sub, imu_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )

        sync.registerCallback(self.fusion_callback)

    def fusion_callback(self, visual, lidar, imu):
        """
        Called when all three messages are available
        within 100ms of each other
        """
        self.fuse_measurements(visual, lidar, imu)
```

### 2.3 Sensor Fusion Node Architecture

```yaml
# sensor_fusion_node_config.yaml
sensor_fusion:
  input_topics:
    visual_odometry: "/camera/odometry"  # from SLAM
    lidar_odometry: "/lidar/odometry"    # from LiDAR-SLAM
    imu_data: "/imu/data"               # raw IMU

  output_topics:
    fused_pose: "/fused/odometry"       # fused estimate
    diagnostics: "/fused/diagnostics"

  fusion_parameters:
    use_visual: true                     # enable camera odometry
    use_lidar: true                      # enable LiDAR odometry
    use_imu: true                        # enable IMU

    sync_tolerance: 0.1  # seconds (100ms)

    # Weighting (lower covariance = higher weight)
    visual_weight: 1.0   # reference weight
    lidar_weight: 1.5    # LiDAR slightly less trusted
    imu_weight: 2.0      # IMU least trusted for position

  failure_modes:
    visual_timeout: 2.0    # seconds until visual considered dead
    lidar_timeout: 2.0
    imu_timeout: 1.0
    fallback_sensor: "lidar"  # use this if others fail
```

## Layer 3: Intelligence (Fusion Tuning)

### 3.1 Covariance Tuning

Each sensor publishes odometry with covariance matrix:

```python
def compute_covariance(sensor_noise, measurement_error):
    """
    Covariance = measure of uncertainty

    Low covariance (0.01) = high confidence
    High covariance (1.0) = low confidence
    """
    return np.eye(3) * measurement_error

# Visual odometry covariance
# Usually low in well-lit scenes, high in darkness
visual_cov = 0.05  # confident in daylight

# LiDAR odometry covariance
# Usually low, except in featureless corridors
lidar_cov = 0.10   # medium confidence

# IMU covariance
# High initially, increases with time (drift)
imu_cov_initial = 0.5  # low confidence
imu_cov_over_time = imu_cov_initial + drift_rate * time
```

### 3.2 Failure Mode Handling

```python
# code_examples/fusion_failure_handling.py
class FusionWithFailover(rclpy.Node):
    def __init__(self):
        self.active_sensors = {'visual': True, 'lidar': True, 'imu': True}
        self.last_message_time = {
            'visual': None, 'lidar': None, 'imu': None
        }

    def check_sensor_health(self):
        """Check which sensors are producing data"""
        current_time = self.get_clock().now()

        for sensor in self.active_sensors.keys():
            if self.last_message_time[sensor] is None:
                continue

            time_since_message = (current_time -
                               self.last_message_time[sensor]).nanoseconds / 1e9

            if time_since_message > self.timeout[sensor]:
                self.get_logger().warn(f'{sensor} timeout - disabling')
                self.active_sensors[sensor] = False
            else:
                self.active_sensors[sensor] = True

    def select_fallback(self):
        """Use available sensor if others fail"""
        if self.active_sensors['visual']:
            return 'visual'
        elif self.active_sensors['lidar']:
            return 'lidar'
        elif self.active_sensors['imu']:
            return 'imu'
        else:
            return None  # all sensors failed!
```

### 3.3 Measuring Fusion Performance

**Ground truth comparison**:
```python
# Compare fused pose to ground truth
error_before = compute_ate(visual_only, ground_truth)
error_after = compute_ate(fused_pose, ground_truth)

improvement = (error_before - error_after) / error_before * 100
# Target: >20% improvement
```

## Layer 4: Advanced

### 4.1 Extended Kalman Filter (EKF) Fusion

More advanced than simple weighted averaging:

```
EKF maintains:
├─ State: [x, y, theta, vx, vy, vtheta]
├─ Covariance: Uncertainty in each state variable
└─ Update cycle:
    1. Predict: Use motion model
    2. Measure: Receive sensor measurements
    3. Correct: Weight update by sensor covariance
    4. Repeat
```

## Summary

| Concept | Key Insight |
|---------|------------|
| **Sensor fusion** | Combines strengths, mitigates weaknesses |
| **Covariance** | Measures uncertainty; inverse of confidence |
| **Synchronization** | Temporal alignment critical for fusion |
| **Loosely-coupled** | Simple, sufficient for most applications |
| **Failure handling** | Monitor sensor health, fallback gracefully |

## Code Examples

### Example 1: Multi-Sensor Fusion Node

```python
# code_examples/sensor_fusion_node.py
import numpy as np
import rclpy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class MultiSensorFusion(rclpy.Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Subscribers with synchronization
        visual_sub = Subscriber(self, PoseStamped, '/camera/odometry')
        lidar_sub = Subscriber(self, PoseStamped, '/lidar/odometry')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        sync = ApproximateTimeSynchronizer(
            [visual_sub, lidar_sub, imu_sub],
            queue_size=10, slop=0.1
        )
        sync.registerCallback(self.fusion_callback)

        self.fused_pub = self.create_publisher(
            Odometry, '/fused/odometry', 10)

    def fusion_callback(self, visual, lidar, imu):
        """Fuse three sensor modalities"""

        # Extract covariances
        visual_cov = 0.05  # high confidence
        lidar_cov = 0.10   # medium confidence
        imu_cov = 0.5      # low confidence for position

        # Weight by inverse covariance
        weights = np.array([1/visual_cov, 1/lidar_cov, 1/imu_cov])
        weights /= weights.sum()  # normalize

        # Weighted average of positions
        positions = np.array([
            [visual.pose.position.x, visual.pose.position.y],
            [lidar.pose.position.x, lidar.pose.position.y],
            [imu.orientation.x, imu.orientation.y]  # IMU limited
        ])

        fused_position = weights.reshape(-1, 1) * positions
        fused_position = fused_position.sum(axis=0)

        # Publish fused estimate
        odometry = Odometry()
        odometry.pose.pose.position.x = fused_position[0]
        odometry.pose.pose.position.y = fused_position[1]
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = 'map'

        self.fused_pub.publish(odometry)
```

### Example 2: Sensor Validation

```python
# code_examples/validate_sensor_fusion_accuracy.py
import numpy as np

class FusionValidator(rclpy.Node):
    def __init__(self):
        super().__init__('fusion_validator')
        self.single_sensor_errors = []
        self.fused_errors = []

    def evaluate_accuracy(self, single_sensor, fused, ground_truth):
        """
        Measure improvement from fusion
        """
        single_error = np.linalg.norm(single_sensor - ground_truth)
        fused_error = np.linalg.norm(fused - ground_truth)

        improvement = (single_error - fused_error) / single_error * 100

        self.get_logger().info(
            f'Single sensor error: {single_error:.3f}m\n'
            f'Fused error: {fused_error:.3f}m\n'
            f'Improvement: {improvement:.1f}%'
        )

        return improvement
```

## Practice Exercise

**Objective**: Implement multi-sensor fusion and verify >20% accuracy improvement

**Steps**:
1. Set up three odometry sources:
   - Visual odometry (camera-based SLAM)
   - LiDAR odometry (if available, or use simulated)
   - IMU (gyroscope-based orientation)
2. Implement temporal synchronization
3. Create fusion node with weighted averaging
4. Run test scenario in Gazebo:
   - Well-lit room (cameras work well)
   - Dark room (cameras fail, LiDAR works)
   - Feature-poor corridor (LiDAR ambiguous)
5. Measure accuracy improvement

**Success criteria**:
- Fusion node runs without errors
- Temporal synchronization working (topics synchronized)
- >20% accuracy improvement over best single sensor
- Graceful failure handling (fallback when sensor dies)

## References

- **Sensor Fusion Survey**: [Multi-Sensor Fusion Review](https://arxiv.org/abs/1902.01305)
- **Extended Kalman Filter**: [EKF Tutorial](https://www.kalmanfilter.net/multivariate_k.html)
- **ROS 2 message_filters**: [Message Filters Documentation](http://wiki.ros.org/message_filters)

## Next Steps

You now understand multi-sensor fusion for robust perception. Choose your path:

- **→ [Lesson 8: Capstone](08-capstone-mission.md)** (Integrate complete system)

---

**Lesson 7 Summary**: Multi-sensor fusion combines visual, range, and inertial information for >20% accuracy improvement. Temporal synchronization, covariance weighting, and failure handling enable robust autonomous navigation in diverse conditions.
