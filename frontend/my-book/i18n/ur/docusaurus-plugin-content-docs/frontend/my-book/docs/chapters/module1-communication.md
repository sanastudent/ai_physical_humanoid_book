<!--
  Urdu Translation Placeholder
  English source: ..\frontend\my-book\docs\chapters\module1-communication.md

  TODO: Add Urdu translation below this comment.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

# ROS 2 Communication Patterns

## Overview

ROS 2 provides three primary communication patterns, each suited for different use cases.

## 1. Topics (Publish-Subscribe)

### When to Use
- Continuous data streams (sensor data, state updates)
- One-to-many communication
- Fire-and-forget messaging

### Example: Publishing Camera Images

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Create publisher with QoS
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10  # Queue size
        )

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.publish_frame)
        self.cap = cv2.VideoCapture(0)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical_frame"
            self.publisher.publish(msg)
```

### Subscribing to Topics

```python
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        self.get_logger().info(
            f'Received image: {msg.width}x{msg.height}'
        )
        # Process the image...
```

## Quality of Service (QoS)

QoS policies control message delivery behavior:

### Reliability
- **Reliable**: Guarantees delivery (TCP-like)
- **Best Effort**: May drop messages (UDP-like)

### Durability
- **Transient Local**: Cache last message for late joiners
- **Volatile**: Only send to current subscribers

### History
- **Keep Last**: Store last N messages
- **Keep All**: Store all messages (until limit)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Define custom QoS
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

publisher = node.create_publisher(
    Image,
    '/camera/image',
    qos_profile
)
```

## 2. Services (Request-Response)

### When to Use
- Occasional requests with responses
- Synchronous operations
- Configuration or command execution

### Example: Add Two Numbers

**Define Service** (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

**Service Server**:
```python
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'{request.a} + {request.b} = {response.sum}'
        )
        return response
```

**Service Client**:
```python
class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')

        self.client = self.create_client(
            AddTwoInts,
            'add_two_ints'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        return future
```

## 3. Actions (Goal-Based)

### When to Use
- Long-running tasks with feedback
- Cancelable operations
- Progress monitoring

### Example: Navigate to Goal

**Action Definition** (`NavigateToGoal.action`):
```
# Goal
geometry_msgs/PoseStamped target_pose
---
# Result
bool success
float32 final_distance
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

**Action Server**:
```python
from nav_msgs.action import NavigateToGoal
from rclpy.action import ActionServer

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation...')

        feedback_msg = NavigateToGoal.Feedback()

        # Simulate navigation
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToGoal.Result()

            # Update feedback
            feedback_msg.distance_remaining = 10.0 - i
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = NavigateToGoal.Result()
        result.success = True
        result.final_distance = 0.0
        return result
```

## Choosing the Right Pattern

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topic** | Continuous data | Sensor readings, robot state |
| **Service** | Quick request/response | Get configuration, toggle mode |
| **Action** | Long task with feedback | Navigation, grasping, planning |

## Exercise: Temperature Alert System

**Task**: Build a system with all three patterns:

1. **Topic**: Publish temperature readings
2. **Service**: Set temperature threshold
3. **Action**: Run cooling sequence when threshold exceeded

## Summary

- ✓ Mastered pub-sub pattern with topics
- ✓ Understood QoS policies
- ✓ Implemented services for request-response
- ✓ Created actions for long-running tasks
- ✓ Learned when to use each pattern

Next chapter covers practical implementations!

---

**Image Placeholder**: [Communication patterns diagram]

**Citation**: ROS 2 Documentation. "About Quality of Service Settings." [docs.ros.org](https://docs.ros.org)
