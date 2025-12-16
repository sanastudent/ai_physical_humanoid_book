# ماڈیول 1: ROS 2 کا تعارف

## جائزہ

ROS 2 (Robot Operating System 2) روبوٹکس کے سب سے زیادہ استعمال ہونے والے middleware framework کی اگلی نسل ہے۔ اپنے پیشرو ROS 1 کے برعکس، ROS 2 کو production systems کے لیے بنیاد سے تیار کیا گیا ہے، جو real-time صلاحیتوں، بہتر security، اور multi-robot support فراہم کرتا ہے۔

## ROS 2 کیوں؟

### صنعتی قبولیت

ROS 2 کو دنیا بھر کی معروف روبوٹکس کمپنیوں اور تحقیقی اداروں نے اپنایا ہے:

- **Autonomous Vehicles**: کمپنیاں جیسے Cruise اور Apex.AI ROS 2 استعمال کرتی ہیں
- **Industrial Robotics**: ABB، Universal Robots کی integration
- **Service Robots**: صفائی، ترسیل، اور مہمان نوازی کے robots
- **Research**: دنیا بھر میں یونیورسٹیاں اور labs

### ROS 1 پر اہم بہتریاں

1. **Real-Time Performance**: RTOS support اور deterministic communication
2. **Security**: encrypted، authenticated communication کے لیے DDS-Security
3. **Multi-Platform**: Windows، macOS، Linux، embedded systems
4. **Production-Ready**: Quality of Service (QoS) policies
5. **Multi-Robot Systems**: robot teams کے لیے native support

## DDS Middleware

ROS 2 **Data Distribution Service (DDS)** معیار کی بنیاد پر بنایا گیا ہے۔ DDS فراہم کرتا ہے:

- **Discovery**: مرکزی master کے بغیر خودکار node کی شناخت
- **Quality of Service**: قابل تشکیل reliability، durability، اور latency
- **Scalability**: بڑے distributed systems میں موثر communication

```python
# مثال: ایک سادہ ROS 2 node بنانا
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node has been started!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## بنیادی تصورات

### Nodes

ایک **node** ایک آزاد process ہے جو computation انجام دیتا ہے۔ ایک robotic system میں:

- **Sensor Node**: sensor data کو پڑھتا اور publish کرتا ہے
- **Processing Node**: data کو تبدیل کرتا ہے (مثلاً، filters، algorithms)
- **Actuator Node**: motors کو commands بھیجتا ہے
- **UI Node**: visualization یا user interface فراہم کرتا ہے

### Computation Graph

Nodes ایک **computation graph** کے ذریعے communicate کرتے ہیں:

```
[Camera Node] --image--> [Vision Node] --detections--> [Control Node] --commands--> [Motor Node]
```

### Workspace

ایک ROS 2 workspace میں شامل ہے:

- `src/`: packages کے لیے source code
- `build/`: compiled code
- `install/`: installed packages
- `log/`: execution logs

## مشق: آپ کا پہلا ROS 2 Node

**کام**: ایک node بنائیں جو ہر سیکنڈ "Hello, ROS 2!" print کرے۔

**اقدامات**:

1. ایک نیا package بنائیں:
   ```bash
   ros2 pkg create --build-type ament_python my_first_package
   ```

2. اپنا node لکھیں (اوپر code کی مثال دیکھیں)

3. Build اور source کریں:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. اپنا node چلائیں:
   ```bash
   ros2 run my_first_package minimal_node
   ```

## خلاصہ

اس باب میں، آپ نے سیکھا:

- ✓ ROS 2 کیا ہے اور یہ کیوں اہم ہے
- ✓ ROS 1 پر اہم بہتریاں
- ✓ DDS middleware کا کردار
- ✓ بنیادی تصورات: nodes، computation graphs
- ✓ اپنا پہلا node کیسے بنائیں

## اگلے اقدامات

اگلے باب میں، ہم ROS 2 architecture میں مزید گہرائی سے جائیں گے اور دستیاب مختلف communication patterns کو تلاش کریں گے۔

---

**تصویر کی جگہ**: [DDS layer کے ساتھ ROS 2 architecture کا خاکہ]

**حوالہ**: Macenski, S., et al. (2020). "Robot Operating System 2: Design, architecture, and uses in the wild." Science Robotics.
