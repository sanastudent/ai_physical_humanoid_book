<!--
  Urdu Translation Placeholder
  English source: ..\frontend\my-book\docs\chapters\module1-architecture.md

  TODO: Add Urdu translation below this comment.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

# ROS 2 Architecture and Design Patterns

## System Architecture

ROS 2 follows a distributed, modular architecture that enables flexibility and scalability in robotic systems.

## Communication Layers

### 1. Application Layer
Your custom nodes and packages that implement robot behaviors.

### 2. ROS 2 Client Library (RCL)
Language-agnostic core functionality:
- Node lifecycle management
- Communication patterns
- Parameter handling

### 3. ROS Middleware (RMW)
Abstraction layer over DDS implementations:
- Fast DDS (default)
- Cyclone DDS
- RTI Connext DDS

### 4. DDS Layer
Underlying communication protocol handling discovery, serialization, and transport.

## Design Patterns

### Separation of Concerns

Break complex functionality into focused nodes:

```python
# Bad: Monolithic node
class RobotNode:
    def __init__(self):
        self.camera = Camera()
        self.vision = VisionProcessor()
        self.planner = PathPlanner()
        self.controller = MotorController()
    # Everything in one place!

# Good: Modular design
class CameraNode:  # Only handles camera
class VisionNode:  # Only processes images
class PlannerNode:  # Only plans paths
class ControlNode:  # Only controls motors
```

### Composition

Combine multiple nodes in a single process for efficiency:

```python
from rclpy.node import Node

class SensorFusion(Node):
    """Combines IMU and wheel odometry"""
    def __init__(self):
        super().__init__('sensor_fusion')
        self.imu_sub = self.create_subscription(...)
        self.odom_sub = self.create_subscription(...)
        self.fused_pub = self.create_publisher(...)
```

## Lifecycle Management

ROS 2 provides managed nodes with defined states:

```
[Unconfigured] --configure--> [Inactive] --activate--> [Active]
                                   ^                      |
                                   |                      |
                                   +----- deactivate -----+
```

**Use Cases**:
- Controlled startup sequences
- Error recovery
- Resource management

## Example: Temperature Monitor

```python
from lifecycle_msgs.msg import Transition
from rclpy.lifecycle import LifecycleNode, LifecycleState

class TempMonitor(LifecycleNode):
    def __init__(self):
        super().__init__('temp_monitor')

    def on_configure(self, state: LifecycleState):
        self.get_logger().info('Configuring sensor...')
        self.sensor = self.init_sensor()
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('Starting monitoring...')
        self.timer = self.create_timer(1.0, self.check_temp)
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def check_temp(self):
        temp = self.sensor.read()
        if temp > 80:
            self.get_logger().warn(f'High temperature: {temp}C')
```

## Package Organization

### Standard Package Structure

```
my_robot_pkg/
├── package.xml           # Package metadata
├── setup.py             # Python package setup
├── my_robot_pkg/
│   ├── __init__.py
│   ├── nodes/           # Node implementations
│   ├── utils/           # Helper functions
│   └── config/          # Configuration files
├── launch/              # Launch files
├── config/              # YAML configs
├── rviz/               # RViz configs
└── test/               # Unit tests
```

## Best Practices

### 1. Use Namespaces

```python
# Organize topics hierarchically
/robot1/camera/image
/robot1/lidar/scan
/robot2/camera/image
```

### 2. Parameter Files

```yaml
# config/robot_params.yaml
robot_node:
  ros__parameters:
    max_speed: 1.0
    update_rate: 50
    use_sim_time: false
```

### 3. Logging Levels

```python
self.get_logger().debug('Detailed info')
self.get_logger().info('General info')
self.get_logger().warn('Warning condition')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Critical failure')
```

## Exercise: Lifecycle Node

**Task**: Create a lifecycle node for a robot arm controller.

**Requirements**:
- Configure: Initialize motor drivers
- Activate: Enable motors and start control loop
- Deactivate: Stop control loop, keep motors enabled
- Cleanup: Shutdown motor drivers

## Summary

- ✓ Understood ROS 2 layered architecture
- ✓ Learned design patterns for modular systems
- ✓ Explored lifecycle management
- ✓ Best practices for package organization

Next, we'll explore communication patterns in detail.

---

**Image Placeholder**: [Architecture diagram showing ROS 2 layers]

**Citation**: ROS 2 Design Documentation. [https://design.ros2.org/](https://design.ros2.org/)
