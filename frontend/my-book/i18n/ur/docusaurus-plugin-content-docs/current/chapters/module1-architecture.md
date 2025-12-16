# ROS 2 Architecture اور Design Patterns

## سسٹم Architecture

ROS 2 ایک distributed، modular architecture کی پیروی کرتا ہے جو robotic systems میں لچک اور scalability کو قابل بناتا ہے۔

## Communication Layers

### 1. Application Layer
آپ کے custom nodes اور packages جو robot کی رویات کو لاگو کرتے ہیں۔

### 2. ROS 2 Client Library (RCL)
زبان سے آزاد بنیادی فعالیت:
- Node lifecycle management
- Communication patterns
- Parameter handling

### 3. ROS Middleware (RMW)
DDS implementations پر abstraction layer:
- Fast DDS (default)
- Cyclone DDS
- RTI Connext DDS

### 4. DDS Layer
بنیادی communication protocol جو discovery، serialization، اور transport کو سنبھالتا ہے۔

## Design Patterns

### Separation of Concerns

پیچیدہ فعالیت کو مرکوز nodes میں تقسیم کریں:

```python
# خراب: Monolithic node
class RobotNode:
    def __init__(self):
        self.camera = Camera()
        self.vision = VisionProcessor()
        self.planner = PathPlanner()
        self.controller = MotorController()
    # سب کچھ ایک جگہ!

# اچھا: Modular design
class CameraNode:  # صرف camera کو سنبھالتا ہے
class VisionNode:  # صرف images کو process کرتا ہے
class PlannerNode:  # صرف paths کی منصوبہ بندی کرتا ہے
class ControlNode:  # صرف motors کو control کرتا ہے
```

### Composition

کارکردگی کے لیے ایک واحد process میں متعدد nodes کو یکجا کریں:

```python
from rclpy.node import Node

class SensorFusion(Node):
    """IMU اور wheel odometry کو یکجا کرتا ہے"""
    def __init__(self):
        super().__init__('sensor_fusion')
        self.imu_sub = self.create_subscription(...)
        self.odom_sub = self.create_subscription(...)
        self.fused_pub = self.create_publisher(...)
```

## Lifecycle Management

ROS 2 متعین states کے ساتھ managed nodes فراہم کرتا ہے:

```
[Unconfigured] --configure--> [Inactive] --activate--> [Active]
                                   ^                      |
                                   |                      |
                                   +----- deactivate -----+
```

**استعمال کی صورتیں**:
- کنٹرول شدہ startup sequences
- Error recovery
- Resource management

## مثال: Temperature Monitor

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

### معیاری Package Structure

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

## بہترین طریقے

### 1. Namespaces استعمال کریں

```python
# Topics کو درجہ بندی سے منظم کریں
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

## مشق: Lifecycle Node

**کام**: ایک robot arm controller کے لیے lifecycle node بنائیں۔

**تقاضے**:
- Configure: Motor drivers کو شروع کریں
- Activate: Motors کو فعال کریں اور control loop شروع کریں
- Deactivate: Control loop بند کریں، motors کو فعال رکھیں
- Cleanup: Motor drivers کو بند کریں

## خلاصہ

- ✓ ROS 2 layered architecture کو سمجھا
- ✓ Modular systems کے لیے design patterns سیکھے
- ✓ Lifecycle management کو دریافت کیا
- ✓ Package organization کے لیے بہترین طریقے

اگلے، ہم communication patterns کو تفصیل سے دریافت کریں گے۔

---

**تصویر کی جگہ**: [ROS 2 layers دکھانے والا architecture diagram]

**حوالہ**: ROS 2 Design Documentation. [https://design.ros2.org/](https://design.ros2.org/)
