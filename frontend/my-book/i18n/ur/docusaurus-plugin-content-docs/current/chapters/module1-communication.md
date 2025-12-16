# ROS 2 Communication Patterns

## جائزہ

ROS 2 تین بنیادی communication patterns فراہم کرتا ہے، ہر ایک مختلف استعمال کی صورتوں کے لیے موزوں ہے۔

## 1. Topics (Publish-Subscribe)

### کب استعمال کریں
- مسلسل data streams (sensor data، state updates)
- One-to-many communication
- Fire-and-forget messaging

### مثال: Camera Images کو Publish کرنا

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # QoS کے ساتھ publisher بنائیں
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

### Topics کو Subscribe کرنا

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
        # Image کو process کریں...
```

## Quality of Service (QoS)

QoS policies message delivery کی رویہ کو کنٹرول کرتی ہیں:

### Reliability
- **Reliable**: ترسیل کی ضمانت دیتا ہے (TCP کی طرح)
- **Best Effort**: Messages کو drop کر سکتا ہے (UDP کی طرح)

### Durability
- **Transient Local**: دیر سے شامل ہونے والوں کے لیے آخری message کو cache کریں
- **Volatile**: صرف موجودہ subscribers کو بھیجیں

### History
- **Keep Last**: آخری N messages کو محفوظ کریں
- **Keep All**: تمام messages کو محفوظ کریں (حد تک)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Custom QoS کی وضاحت کریں
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

### کب استعمال کریں
- کبھی کبھار جوابات کے ساتھ درخواستیں
- Synchronous operations
- Configuration یا command execution

### مثال: دو اعداد کو جمع کریں

**Service کی تعریف** (`AddTwoInts.srv`):
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

### کب استعمال کریں
- Feedback کے ساتھ طویل المیعاد کام
- منسوخ کیے جا سکنے والے operations
- ترقی کی نگرانی

### مثال: Goal تک Navigate کریں

**Action کی تعریف** (`NavigateToGoal.action`):
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

        # Navigation کو simulate کریں
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToGoal.Result()

            # Feedback کو update کریں
            feedback_msg.distance_remaining = 10.0 - i
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = NavigateToGoal.Result()
        result.success = True
        result.final_distance = 0.0
        return result
```

## صحیح Pattern کا انتخاب

| Pattern | استعمال کی صورت | مثال |
|---------|----------|---------|
| **Topic** | مسلسل data | Sensor readings، robot state |
| **Service** | فوری request/response | Configuration حاصل کریں، mode toggle کریں |
| **Action** | Feedback کے ساتھ طویل کام | Navigation، grasping، planning |

## مشق: Temperature Alert System

**کام**: تینوں patterns کے ساتھ ایک system بنائیں:

1. **Topic**: Temperature readings کو publish کریں
2. **Service**: Temperature threshold سیٹ کریں
3. **Action**: جب threshold سے تجاوز ہو تو cooling sequence چلائیں

## خلاصہ

- ✓ Topics کے ساتھ pub-sub pattern میں مہارت حاصل کی
- ✓ QoS policies کو سمجھا
- ✓ Request-response کے لیے services کو لاگو کیا
- ✓ طویل المیعاد کاموں کے لیے actions بنائے
- ✓ سیکھا کہ ہر pattern کب استعمال کریں

اگلا باب عملی implementations کو cover کرتا ہے!

---

**تصویر کی جگہ**: [Communication patterns diagram]

**حوالہ**: ROS 2 Documentation. "About Quality of Service Settings." [docs.ros.org](https://docs.ros.org)
