# Module 1: Introduction to ROS 2

## Overview

ROS 2 (Robot Operating System 2) is the next generation of the most widely used robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is built from the ground up for production systems, offering real-time capabilities, improved security, and multi-robot support.

## Why ROS 2?

### Industry Adoption

ROS 2 has been adopted by leading robotics companies and research institutions worldwide:

- **Autonomous Vehicles**: Companies like Cruise and Apex.AI use ROS 2
- **Industrial Robotics**: ABB, Universal Robots integration
- **Service Robots**: Cleaning, delivery, and hospitality robots
- **Research**: Universities and labs globally

### Key Improvements Over ROS 1

1. **Real-Time Performance**: RTOS support and deterministic communication
2. **Security**: DDS-Security for encrypted, authenticated communication
3. **Multi-Platform**: Windows, macOS, Linux, embedded systems
4. **Production-Ready**: Quality of Service (QoS) policies
5. **Multi-Robot Systems**: Native support for robot teams

## The DDS Middleware

ROS 2 is built on top of the **Data Distribution Service (DDS)** standard. DDS provides:

- **Discovery**: Automatic node detection without a central master
- **Quality of Service**: Configurable reliability, durability, and latency
- **Scalability**: Efficient communication in large distributed systems

```python
# Example: Creating a simple ROS 2 node
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

## Core Concepts

### Nodes

A **node** is an independent process that performs computation. In a robotic system:

- **Sensor Node**: Reads and publishes sensor data
- **Processing Node**: Transforms data (e.g., filters, algorithms)
- **Actuator Node**: Sends commands to motors
- **UI Node**: Provides visualization or user interface

### Computation Graph

Nodes communicate through a **computation graph**:

```
[Camera Node] --image--> [Vision Node] --detections--> [Control Node] --commands--> [Motor Node]
```

### Workspace

A ROS 2 workspace contains:

- `src/`: Source code for packages
- `build/`: Compiled code
- `install/`: Installed packages
- `log/`: Execution logs

## Exercise: Your First ROS 2 Node

**Task**: Create a node that prints "Hello, ROS 2!" every second.

**Steps**:

1. Create a new package:
   ```bash
   ros2 pkg create --build-type ament_python my_first_package
   ```

2. Write your node (see code example above)

3. Build and source:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Run your node:
   ```bash
   ros2 run my_first_package minimal_node
   ```

## Summary

In this chapter, you learned:

- ✓ What ROS 2 is and why it matters
- ✓ Key improvements over ROS 1
- ✓ The role of DDS middleware
- ✓ Basic concepts: nodes, computation graphs
- ✓ How to create your first node

## Next Steps

In the next chapter, we'll dive deeper into ROS 2 architecture and explore the different communication patterns available.

---

**Image Placeholder**: [Diagram showing ROS 2 architecture with DDS layer]

**Citation**: Macenski, S., et al. (2020). "Robot Operating System 2: Design, architecture, and uses in the wild." Science Robotics.
