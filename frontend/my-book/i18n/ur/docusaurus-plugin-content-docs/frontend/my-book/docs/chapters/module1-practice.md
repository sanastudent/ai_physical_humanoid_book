<!--
  Urdu Translation Placeholder
  English source: ..\frontend\my-book\docs\chapters\module1-practice.md

  TODO: Add Urdu translation below this comment.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

# ROS 2 Practical Implementation

## Building a Complete Robot System

Let's integrate everything we've learned into a practical mobile robot system.

## Project: Autonomous Explorer Robot

### System Requirements

Our robot will:
1. Read from a LiDAR sensor
2. Detect obstacles
3. Navigate autonomously
4. Provide status updates

### Architecture

```
┌─────────────┐
│  LiDAR Node │──scan──>┌──────────────┐
└─────────────┘         │ Obstacle Det.│──obstacles──>┌──────────┐
                        └──────────────┘              │ Navigator│──cmd_vel──>┌───────┐
┌─────────────┐                                       └──────────┘            │ Motors│
│  Odom Node  │──odometry──>──────────────────────────>    ^                 └───────┘
└─────────────┘                                            |
                                                           |
                                      ┌─────────────┐      |
                                      │ User Goals  │──────┘
                                      └─────────────┘
```

## Implementation

### 1. LiDAR Node

```python
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_scan)

        # Configure LiDAR parameters
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi / 180
        self.range_min = 0.1
        self.range_max = 10.0

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Simulate or read actual LiDAR data
        num_readings = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        )
        scan.ranges = [self.read_distance(i) for i in range(num_readings)]

        self.publisher.publish(scan)
```

### 2. Obstacle Detection Node

```python
class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.obstacle_pub = self.create_publisher(
            Bool,
            '/obstacle_detected',
            10
        )

        # Obstacle detection threshold (meters)
        self.declare_parameter('obstacle_threshold', 0.5)

    def scan_callback(self, scan: LaserScan):
        threshold = self.get_parameter(
            'obstacle_threshold'
        ).get_parameter_value().double_value

        # Check if any reading is below threshold
        obstacle_detected = any(
            r < threshold for r in scan.ranges
            if self.range_min < r < self.range_max
        )

        msg = Bool()
        msg.data = obstacle_detected
        self.obstacle_pub.publish(msg)

        if obstacle_detected:
            self.get_logger().warn('Obstacle detected!')
```

### 3. Navigation Node (Action)

```python
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from my_interfaces.action import NavigateToGoal

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # Velocity command publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Obstacle subscription
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/obstacle_detected',
            self.obstacle_callback,
            10
        )

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.navigate_callback
        )

        self.obstacle_present = False

    def obstacle_callback(self, msg: Bool):
        self.obstacle_present = msg.data

    def navigate_callback(self, goal_handle):
        self.get_logger().info('Starting navigation...')

        target = goal_handle.request.target_pose
        feedback = NavigateToGoal.Feedback()

        rate = self.create_rate(10)  # 10 Hz

        while not self.reached_goal(target):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                return NavigateToGoal.Result()

            # Handle obstacles
            if self.obstacle_present:
                self.get_logger().warn('Obstacle! Stopping.')
                self.stop_robot()
                rate.sleep()
                continue

            # Compute and publish velocity
            cmd_vel = self.compute_velocity(target)
            self.cmd_pub.publish(cmd_vel)

            # Publish feedback
            feedback.current_pose = self.get_current_pose()
            feedback.distance_remaining = self.compute_distance(target)
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Goal reached
        self.stop_robot()
        goal_handle.succeed()

        result = NavigateToGoal.Result()
        result.success = True
        result.final_distance = 0.0
        return result

    def stop_robot(self):
        stop_cmd = Twist()  # All zeros
        self.cmd_pub.publish(stop_cmd)
```

## Launch File

Create `launch/robot_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR node
        Node(
            package='my_robot',
            executable='lidar_node',
            name='lidar',
            parameters=[{'frame_id': 'laser_frame'}]
        ),

        # Obstacle detector
        Node(
            package='my_robot',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[{'obstacle_threshold': 0.5}]
        ),

        # Navigator
        Node(
            package='my_robot',
            executable='navigator',
            name='navigator',
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/robot.rviz']
        ),
    ])
```

## Testing the System

### 1. Build and Source

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
```

### 2. Launch the System

```bash
ros2 launch my_robot robot_system.launch.py
```

### 3. Send Navigation Goal

```bash
ros2 action send_goal /navigate_to_goal my_interfaces/action/NavigateToGoal \
  "{target_pose: {pose: {position: {x: 2.0, y: 1.0}}}}"
```

### 4. Monitor Topics

```bash
# Watch velocity commands
ros2 topic echo /cmd_vel

# Monitor obstacles
ros2 topic echo /obstacle_detected

# View LiDAR scan
ros2 topic echo /scan
```

## Debugging Tools

### RQt Graph

Visualize node connections:
```bash
rqt_graph
```

### Topic Monitor

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic hz /cmd_vel  # Check publishing rate
```

### Logger Level

Change logging verbosity:
```bash
ros2 run my_robot navigator --ros-args --log-level debug
```

## Common Issues & Solutions

### Issue: Nodes can't discover each other

**Solution**: Check DDS domain ID
```bash
export ROS_DOMAIN_ID=42
```

### Issue: Topics not receiving data

**Solution**: Verify QoS compatibility
```python
# Use sensor QoS profile
from rclpy.qos import qos_profile_sensor_data

subscription = node.create_subscription(
    LaserScan,
    '/scan',
    callback,
    qos_profile_sensor_data
)
```

## Exercise: Add Camera Integration

**Task**: Extend the system with a camera node

**Requirements**:
1. Publish camera images at 30 Hz
2. Create object detection node
3. Integrate detections into navigation decisions

## Summary

- ✓ Built complete multi-node robot system
- ✓ Integrated sensors, processing, and control
- ✓ Used launch files for system orchestration
- ✓ Applied debugging tools
- ✓ Handled real-world challenges

Congratulations on completing Module 1!

---

**Image Placeholder**: [Complete system architecture diagram]

**Citation**: Macenski, S. "Navigation2." [navigation.ros.org](https://navigation.ros.org)
