---
sidebar_position: 7
title: "Lesson 6: Processing Sensor Data"
description: "Write ROS 2 subscribers to process camera, LiDAR, and IMU data in real-time"
---

# Lesson 6: Processing Sensor Data (75 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Write ROS 2 subscriber nodes for camera, LiDAR, and IMU data
2. Convert ROS 2 Image messages to OpenCV format
3. Extract obstacle distances from LaserScan data
4. Interpret IMU acceleration and rotation values
5. Synchronize multi-sensor data using message_filters

---

## Prerequisites

- Lesson 5 completed (sensors added to robot)
- Chapter 1 understanding of ROS 2 subscribers
- Comfortable with Python 3.10+

**Install dependencies**:
```bash
sudo apt install python3-cv-bridge python3-opencv ros-humble-message-filters
```

---

## Core Concepts (8 Total)

### 1. Image Subscribers
ROS 2 Camera images are `sensor_msgs/Image`:
- **encoding**: Pixel format (`rgb8`, `bgr8`, `mono8`, etc.)
- **width, height**: Image dimensions in pixels
- **data**: Raw pixel bytes
- **header.frame_id**: Camera optical frame

Typical workflow:
1. Subscribe to `/camera/image_raw`
2. Convert Image → OpenCV Mat (using cv_bridge)
3. Process with OpenCV (edge detection, feature extraction)
4. Publish results back to ROS 2

### 2. LaserScan Subscribers
ROS 2 LiDAR scans are `sensor_msgs/LaserScan`:
- **angle_min, angle_max**: Scan coverage (radians, typically -π to π)
- **angle_increment**: Radians between consecutive beams
- **ranges**: Array of distances (in meters)
- **intensities**: Optional reflectivity per beam
- **header.stamp**: Measurement timestamp

Example interpretation:
```python
for i, range_val in enumerate(msg.ranges):
    angle = msg.angle_min + i * msg.angle_increment
    # (angle, range_val) = one measurement point
```

### 3. IMU Subscribers
ROS 2 IMU data is `sensor_msgs/Imu`:
- **linear_acceleration**: 3D acceleration vector (m/s²)
  - `x, y, z` components in IMU frame
  - Includes gravity (9.81 m/s² downward if stationary)
- **angular_velocity**: 3D rotation rate (rad/s)
  - `x, y, z` components (roll, pitch, yaw rates)
- **orientation**: Quaternion (if IMU can estimate attitude)

Example interpretation:
```python
accel_magnitude = sqrt(ax² + ay² + az²)  # Total acceleration
```

### 4. cv_bridge (Image Conversion)
Converts between ROS 2 Image and OpenCV Mat:
```python
from cv_bridge import CvBridge
bridge = CvBridge()

# Image → OpenCV
cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")

# OpenCV → Image (to publish)
image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
```

### 5. Message Filtering (Time Sync)
Synchronize multiple sensor streams (camera + LiDAR + IMU):
```python
from message_filters import Subscriber, ApproximateTimeSynchronizer

image_sub = Subscriber(node, Image, '/camera/image_raw')
scan_sub = Subscriber(node, LaserScan, '/scan')
imu_sub = Subscriber(node, Imu, '/imu/data')

# Synchronize messages within 100ms time window
sync = ApproximateTimeSynchronizer([image_sub, scan_sub, imu_sub], 10, 0.1)
sync.registerCallback(synchronized_callback)
```

### 6. Obstacle Detection
Extract obstacles from LaserScan:
```python
# Find closest obstacle
min_range = min(msg.ranges)
min_index = msg.ranges.index(min_range)
obstacle_angle = msg.angle_min + min_index * msg.angle_increment

# All obstacles within threshold
obstacles = [(i, r) for i, r in enumerate(msg.ranges) if r < 0.5]
```

### 7. Edge Detection (OpenCV)
Common image processing for camera data:
```python
import cv2
# Canny edge detection
edges = cv2.Canny(cv_image, 100, 200)
```

### 8. Visualization in RViz
Processed data can be published as:
- **Image topic**: Reprocess and republish edited images
- **MarkerArray**: 3D visual overlays in RViz
- **PointCloud2**: 3D point clouds from LiDAR
- **Twist**: Control commands based on sensor input

---

## Layer 1: Manual Exercise

### Exercise: Process Camera Image with Edge Detection

1. Launch Gazebo with sensor-equipped robot (from Lesson 5)
2. In RViz, add **Image** display for `/camera/image_raw`
3. Observe raw camera image
4. Run image processor script (see below)
5. Add second **Image** display for `/camera/edges`
6. See edge-detected image appear

---

## Code Examples: Sensor Subscribers

### Camera Image Subscriber with Edge Detection

**File**: `examples/chapter-2-gazebo/lesson-06/camera_subscriber.py`

```python
#!/usr/bin/env python3
"""
Camera image subscriber with edge detection
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publish processed edges
        self.edges_pub = self.create_publisher(Image, '/camera/edges', 10)

        self.get_logger().info('Image processor initialized')

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Apply Canny edge detection
            edges = cv2.Canny(gray, 100, 200)

            # Convert back to ROS Image
            edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            edge_msg.header = msg.header  # Preserve timestamp

            # Publish
            self.edges_pub.publish(edge_msg)

            self.get_logger().debug('Processed image, found edges')

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
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

### LaserScan Subscriber with Obstacle Detection

**File**: `examples/chapter-2-gazebo/lesson-06/lidar_subscriber.py`

```python
#!/usr/bin/env python3
"""
LiDAR LaserScan subscriber with obstacle detection
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.obstacle_threshold = 0.5  # meters

        # Subscribe to LiDAR scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publish minimum distance
        self.min_distance_pub = self.create_publisher(Float32, '/obstacle/min_distance', 10)

        self.get_logger().info('Obstacle detector initialized')

    def scan_callback(self, msg):
        """Process incoming LiDAR scan"""
        try:
            # Filter out infinity values (no return)
            valid_ranges = [r for r in msg.ranges if 0 < r < float('inf')]

            if not valid_ranges:
                self.get_logger().warn('No valid LiDAR readings')
                return

            # Find minimum range (closest obstacle)
            min_range = min(valid_ranges)
            min_index = msg.ranges.index(min_range)

            # Calculate angle to closest obstacle
            obstacle_angle = msg.angle_min + min_index * msg.angle_increment

            # Publish minimum distance
            min_dist_msg = Float32(data=float(min_range))
            self.min_distance_pub.publish(min_dist_msg)

            # Log obstacle warning if too close
            if min_range < self.obstacle_threshold:
                self.get_logger().warn(
                    f'Obstacle detected! Distance: {min_range:.2f}m at angle {obstacle_angle:.2f}rad')

        except Exception as e:
            self.get_logger().error(f'Failed to process scan: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
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

### IMU Subscriber

**File**: `examples/chapter-2-gazebo/lesson-06/imu_subscriber.py`

```python
#!/usr/bin/env python3
"""
IMU subscriber for motion monitoring
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUMonitorNode(Node):
    def __init__(self):
        super().__init__('imu_monitor')

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        self.get_logger().info('IMU monitor initialized')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        try:
            # Extract acceleration components
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # Extract angular velocity components
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z

            # Calculate magnitudes
            accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
            angular_mag = math.sqrt(wx**2 + wy**2 + wz**2)

            # Log (debug level to avoid spam)
            self.get_logger().debug(
                f'Accel: {accel_mag:.2f} m/s², Angular: {angular_mag:.2f} rad/s')

            # Detect significant acceleration
            if accel_mag > 15.0:  # More than 1.5G
                self.get_logger().warn(f'High acceleration detected: {accel_mag:.2f}m/s²')

            # Detect fast rotation
            if angular_mag > 3.0:  # More than ~170 degrees/second
                self.get_logger().warn(f'Fast rotation detected: {angular_mag:.2f} rad/s')

        except Exception as e:
            self.get_logger().error(f'Failed to process IMU data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Multi-Sensor Fusion with Time Synchronization

**File**: `examples/chapter-2-gazebo/lesson-06/sensor_fusion_node.py`

```python
#!/usr/bin/env python3
"""
Multi-sensor fusion with time synchronization
Simulation environment: Gazebo 11+
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Create subscribers using message_filters
        image_sub = Subscriber(self, Image, '/camera/image_raw')
        scan_sub = Subscriber(self, LaserScan, '/scan')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        # Synchronize messages within 100ms
        ts = ApproximateTimeSynchronizer(
            [image_sub, scan_sub, imu_sub],
            queue_size=10,
            slop=0.1)  # 100ms time window
        ts.registerCallback(self.fusion_callback)

        self.get_logger().info('Sensor fusion node initialized')

    def fusion_callback(self, image_msg, scan_msg, imu_msg):
        """Process synchronized sensor data"""
        try:
            # Get image timestamp
            image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec / 1e9

            # Get LiDAR data
            min_range = min([r for r in scan_msg.ranges if 0 < r < float('inf')])

            # Get IMU acceleration
            accel = imu_msg.linear_acceleration

            self.get_logger().info(
                f'Fused data - Time: {image_time:.3f}, MinRange: {min_range:.2f}m, '
                f'Accel: {accel.x:.2f},{accel.y:.2f},{accel.z:.2f}')

        except Exception as e:
            self.get_logger().error(f'Fusion callback error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
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

💬 **Prompt 1**: "The LaserScan ranges include 'inf' values. What do they mean?"

Claude explains:
- `inf` (infinity) = no laser return at that angle
- Object too far, light absorbed, or no object
- Always filter with: `if 0 < r < float('inf')`
- Simplifies to: `if r > 0 and not math.isinf(r)`

💬 **Prompt 2**: "My synchronized callback never fires. What's wrong?"

Claude explains:
- Time difference between messages > slop parameter
- Check message update rates match approximately
- If camera is 30 Hz and LiDAR is 10 Hz, timing is hard to sync
- Increase slop: `slop=0.5` (500ms window) for slower sensors

---

## Troubleshooting

### Problem: "cv_bridge not found"
**Cause**: cv_bridge package not installed
**Solution**:
```bash
sudo apt install python3-cv-bridge
```

### Problem: Image callback receives empty image
**Cause**: Camera plugin not publishing or wrong topic name
**Solution**:
```bash
# Verify topic exists and has messages
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

### Problem: LaserScan contains all inf values
**Cause**: No objects in LiDAR range or beam angle wrong
**Solution**:
- Add objects to Gazebo world
- Check ray sensor beam count (should be > 1)

### Problem: IMU data looks wrong (huge random values)
**Cause**: Noise model too aggressive
**Solution**: Reduce noise stddev in sensor plugin

### Problem: Synchronized callback never fires
**Cause**: Message timestamps too far apart
**Solution**:
- Increase slop: `ApproximateTimeSynchronizer(..., slop=0.5)`
- Verify all sensors publish at reasonable rates
- Check ROS 2 clock synchronization (`ros2 topic echo /clock`)

---

## Self-Assessment Checklist

- [ ] I've written an image subscriber that receives camera frames
- [ ] I can convert ROS 2 Image to OpenCV and back
- [ ] I've applied edge detection or another image filter
- [ ] I've written a LaserScan subscriber and extracted distance data
- [ ] I can identify obstacles from LiDAR data
- [ ] I've written an IMU subscriber and interpret acceleration/rotation
- [ ] I understand message_filters and can synchronize multiple sensor streams

---

## Key Takeaways

1. **ROS 2 subscribers are the primary interface**: Get data via topics
2. **cv_bridge handles image conversion**: Image ↔ OpenCV seamlessly
3. **Always filter invalid data**: inf values, negative ranges, etc.
4. **Message filtering synchronizes sensors**: Keep multi-sensor pipelines aligned
5. **Processing happens in callbacks**: Minimize compute per message

---

## Next Lesson

In Lesson 7, you'll close the loop by commanding robot motion based on processed sensor data.

[Go to Lesson 7: Gazebo-ROS 2 Integration →](./lesson-07-gazebo-ros2-integration.md)

---

**Lesson Status**: Complete with four sensor processor examples (camera, LiDAR, IMU, fusion) and troubleshooting.

**Duration**: 75 minutes (including RViz sensor visualization)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-06/camera_subscriber.py`
- `examples/chapter-2-gazebo/lesson-06/lidar_subscriber.py`
- `examples/chapter-2-gazebo/lesson-06/imu_subscriber.py`
- `examples/chapter-2-gazebo/lesson-06/sensor_fusion_node.py`

**Concepts Covered**: 8 (Image subscribers, LaserScan subscribers, IMU subscribers, cv_bridge, message filtering, obstacle detection, edge detection, RViz visualization)
