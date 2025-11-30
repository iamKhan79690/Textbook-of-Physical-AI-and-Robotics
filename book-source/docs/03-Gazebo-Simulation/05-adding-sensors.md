---
sidebar_position: 6
title: "Lesson 5: Adding Sensors"
description: "Equip robots with cameras, LiDAR, and IMU sensors for perception"
---

# Lesson 5: Adding Sensors (75 minutes)

## Learning Outcomes

By the end of this lesson, you will:
1. Understand sensor plugin architecture in Gazebo
2. Add camera, LiDAR, and IMU sensors to robot URDFs
3. Configure sensor intrinsics (focal length, field of view, noise)
4. Verify sensors publish to correct ROS 2 topics
5. Visualize sensor data in RViz

---

## Prerequisites

- Lesson 4 completed (physics-tuned robot)
- ROS 2 sensor_msgs understanding

---

## Core Concepts (8 Total)

### 1. Gazebo Sensor Plugins
Plugins bridge Gazebo simulation and ROS 2:
- **gazebo_ros_camera**: Simulates camera with optical distortion
- **gazebo_ros_lidar**: Simulates 2D/3D LiDAR with ray casting
- **gazebo_ros_imu**: Simulates inertial measurement unit

Each plugin:
- Reads simulation state at update_rate
- Publishes ROS 2 messages
- Can add noise models for realism

### 2. Camera Intrinsics
Camera parameters that define how images are formed:
- **Focal length** (fx, fy): Lens zoom (pixels)
- **Principal point** (cx, cy): Center of image (pixels)
- **Distortion**: Barrel/pincushion distortion coefficients
- **Field of view**: Horizontal and vertical angle (radians)
- **Image resolution**: Width × height in pixels

Typical values:
```
fx = fy = 500 pixels  (moderate zoom)
cx = width/2, cy = height/2
fov = 60-90 degrees
resolution = 640×480 or 1280×960
```

### 3. LiDAR Beam Model
LiDAR simulates laser rangefinder:
- **Beam count**: 1 (single beam) to 360+ (spinning laser)
- **Angular resolution**: Degrees between consecutive beams
- **Range**: Min/max distance scannable (0.1 m to 100+ m)
- **Noise model**: Gaussian noise on range measurements
- **Update rate**: Scans per second (typically 10-40 Hz)

### 4. IMU (Inertial Measurement Unit)
Measures acceleration and angular velocity:
- **Accelerometer**: Linear acceleration in X, Y, Z
- **Gyroscope**: Angular velocity around X, Y, Z axes
- **Noise models**: Gaussian noise on each sensor
- **Update rate**: Measurements per second (typically 100+ Hz)

### 5. Sensor Placement
Sensor frame attachment:
- **Fixed joints** connect sensor links to robot body
- **Frame ID**: ROS 2 TF frame (e.g., `camera_link`, `lidar_link`)
- **Origin**: Offset from parent link (xyz and rotation)

Example:
```xml
<joint name="camera_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>  <!-- 10cm forward, 5cm up -->
</joint>
```

### 6. ROS 2 Topic Mapping
Plugins map sensor data to ROS 2 topics:
- **Camera**: `/camera/image_raw` (sensor_msgs/Image)
- **LiDAR**: `/scan` (sensor_msgs/LaserScan)
- **IMU**: `/imu/data` (sensor_msgs/Imu)

Custom namespacing:
```xml
<ros>
  <namespace>/robot/sensors</namespace>  <!-- Changes to /robot/sensors/camera/image_raw -->
</ros>
```

### 7. Noise Models
Realistic sensors have noise:
- **Gaussian**: Random noise with mean 0, standard deviation σ
- **Update rate effects**: Lower update rates = less frequent measurements
- **Quantization**: Measurement precision (e.g., 1 mm for LiDAR)

### 8. Sensor Refresh Rates
Critical for timing:
- **Camera**: 10-30 Hz (human visual perception)
- **LiDAR**: 10-40 Hz (navigation planning)
- **IMU**: 100-200 Hz (high-speed control feedback)
- Must coordinate in ROS 2 subscribers (message_filters for synchronization)

---

## Layer 1: Manual Exercise - Add Sensors to URDF

### Exercise Steps:

1. **Create camera link** (small sphere at front):
```xml
<link name="camera_link">
  <inertial><mass value="0.05"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
  <visual><geometry><sphere radius="0.02"/></geometry></visual>
  <collision><geometry><sphere radius="0.02"/></geometry></collision>
</link>
```

2. **Mount camera on base**:
```xml
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>  <!-- 10cm forward, 5cm up -->
</joint>
```

3. **Add camera plugin**:
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image><width>640</width><height>480</height></image>
      <clip><near>0.1</near><far>100</far></clip>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros><namespace>/camera</namespace></ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## Code Example: Robot with All Sensors

**File**: `examples/chapter-2-gazebo/lesson-05/robot-with-all-sensors.urdf`

```xml
<?xml version="1.0"?>
<robot name="sensor_equipped_robot">
  <!-- Simulation environment: Gazebo 11+ -->

  <!-- Base Link (from previous lessons) -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.4"/>
    </inertial>
    <visual>
      <geometry><box size="0.2 0.15 0.1"/></geometry>
      <material name="base_color"><color rgba="0.7 0.7 0.7 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.15 0.1"/></geometry>
    </collision>
  </link>

  <!-- Wheels (omitted for brevity, same as lesson 3) -->

  <!-- Camera Link -->
  <link name="camera_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry><box size="0.05 0.05 0.03"/></geometry>
      <material name="camera_material"><color rgba="0.3 0.3 0.3 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.03"/></geometry>
    </collision>
  </link>

  <!-- LiDAR Link -->
  <link name="lidar_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry><cylinder radius="0.03" length="0.05"/></geometry>
      <material name="lidar_material"><color rgba="0.2 0.2 0.2 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.03" length="0.05"/></geometry>
    </collision>
  </link>

  <!-- IMU Link -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <geometry><box size="0.02 0.02 0.02"/></geometry>
      <material name="imu_material"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.02 0.02 0.02"/></geometry>
    </collision>
  </link>

  <!-- Camera Joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR Joint -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- IMU Joint -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo Sensor Plugins -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image><width>640</width><height>480</height></image>
        <clip><near>0.1</near><far>100</far></clip>
        <distortion>
          <k1>0.0</k1>
          <k2>0.0</k2>
          <k3>0.0</k3>
          <p1>0.0</p1>
          <p2>0.0</p2>
        </distortion>
      </camera>
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <ros><namespace>/camera</namespace></ros>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.141592654</min_angle>
            <max_angle>3.141592654</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_lidar.so">
        <ros><namespace>/lidar</namespace></ros>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor type="imu" name="imu_sensor">
      <update_rate>100</update_rate>
      <imu>
        <noise>
          <type>gaussian</type>
          <rate>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </accel>
        </noise>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros><namespace>/imu</namespace></ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

---

## Layer 2: AI Collaboration Notes

💬 **Prompt 1**: "Why do I need both camera and LiDAR? What's the difference?"

Claude explains:
- **Camera**: Sees color/texture (needs lighting, detailed visual processing)
- **LiDAR**: Measures distance to objects (works in dark, gives 3D point cloud)
- **Camera best for**: Object recognition, visual features, human-readable imagery
- **LiDAR best for**: Obstacle detection, navigation, range measurements

💬 **Prompt 2**: "My sensor is publishing but the topic appears in 'rostopic list' but no messages arrive. What's wrong?"

Claude explains:
- Plugin loaded but not publishing messages
- Check topic name matches between URDF and subscriber
- Verify update_rate > 0 (not paused)
- Check ROS_DOMAIN_ID matches between publisher and subscriber
- Use `ros2 topic hz /topic_name` to verify message rate

---

## Troubleshooting

### Problem: "Plugin not found: libgazebo_ros_camera.so"
**Cause**: gazebo-ros plugins not installed
**Solution**:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Problem: Sensor topics don't appear in `ros2 topic list`
**Cause**: Topic namespace wrong or plugin not initialized
**Solution**:
- Check `<namespace>` in plugin configuration
- Verify Gazebo runs with ROS 2 support: `gazebo -s libgazebo_ros_init.so`

### Problem: Camera image is black/empty
**Cause**: Camera position wrong or object too close/far from camera
**Solution**:
```xml
<!-- Adjust focal length and clip planes -->
<horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees FOV -->
<clip><near>0.1</near><far>100</far></clip>
```

### Problem: LiDAR returns all zeros (no range data)
**Cause**: No objects in range or ray direction wrong
**Solution**:
- Verify objects exist in simulation
- Check horizontal_fov covers 360 degrees
- Ensure min/max range reasonable

### Problem: IMU data looks unrealistic (huge accelerations)
**Cause**: Noise model too aggressive or gravity not applied
**Solution**:
```xml
<stddev>0.1</stddev>  <!-- Reduce from 1.0 to 0.1 -->
```

---

## Self-Assessment Checklist

- [ ] I understand the role of Gazebo sensor plugins
- [ ] I've added camera, LiDAR, and IMU sensors to a robot URDF
- [ ] My sensors appear as links/joints in the kinematic tree
- [ ] Each sensor publishes to the correct ROS 2 topic
- [ ] I can configure sensor intrinsics (FOV, resolution, noise)
- [ ] I've verified sensor data in RViz (camera image, point cloud, IMU)

---

## Key Takeaways

1. **Sensors are links + plugins**: Physical part (link) + simulation (plugin)
2. **Fixed joints attach sensors**: Immobilize sensors relative to robot
3. **Topic namespacing prevents collisions**: Multiple robots can have `/camera/image_raw`
4. **Update rates matter**: Sync sensors with ROS 2 message_filters
5. **Noise models add realism**: Gazebo can simulate sensor imperfections

---

## Next Lesson

In Lesson 6, you'll write ROS 2 nodes to process sensor data and extract meaningful information.

[Go to Lesson 6: Processing Sensor Data →](./lesson-06-processing-sensors.md)

---

**Lesson Status**: Complete with sensor plugin overview, full multi-sensor robot URDF, and sensor configuration guide.

**Duration**: 75 minutes (including RViz sensor visualization)

**Files Created**:
- `examples/chapter-2-gazebo/lesson-05/robot-with-camera.urdf`
- `examples/chapter-2-gazebo/lesson-05/robot-with-lidar.urdf`
- `examples/chapter-2-gazebo/lesson-05/robot-with-imu.urdf`
- `examples/chapter-2-gazebo/lesson-05/robot-with-all-sensors.urdf`

**Concepts Covered**: 8 (sensor plugins, camera intrinsics, LiDAR beam model, IMU, sensor placement, ROS 2 topic mapping, noise models, update rates)
