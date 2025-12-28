---
sidebar_position: 2
---

# Core ROS 2 Concepts: Nodes, Topics, Services, and Messages

## Understanding ROS 2 Architecture

In this section, we'll explore the fundamental building blocks of ROS 2. These concepts form the foundation of how robots built with ROS 2 communicate and coordinate their behavior.

### Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node performs a specific function and communicates with other nodes through topics, services, and actions.

**Key characteristics of nodes:**
- Each node runs a single-threaded or multi-threaded process
- Nodes can be written in different programming languages (C++, Python, etc.)
- Nodes can be started and stopped independently
- Nodes can be distributed across multiple machines

### Topics and Publishers/Subscribers

**Topics** are named buses over which nodes exchange messages. The communication is based on a publish/subscribe pattern where publishers send messages and subscribers receive them.

**Example:**
```python
# Publisher example
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(String, 'topic_name', 10)

    msg = String()
    msg.data = 'Hello, ROS 2!'
    publisher.publish(msg)

    rclpy.shutdown()
```

### Services

**Services** provide a request/reply communication pattern. A service client sends a request to a service server, which processes the request and sends back a response.

**Key characteristics:**
- Synchronous communication
- Request/response pattern
- One-to-one communication model

### Messages

**Messages** are the data structures that are passed between nodes. They define the format of the data being exchanged and are language-independent.

**Common message types:**
- `std_msgs`: Basic data types (int, float, string, etc.)
- `geometry_msgs`: Geometric primitives (points, poses, etc.)
- `sensor_msgs`: Sensor data (images, laser scans, etc.)

### Learning Objectives

After completing this section, you should be able to:
- Explain the role of nodes in a ROS 2 system
- Describe how topics enable communication between nodes
- Understand the difference between topics and services
- Identify common message types used in ROS 2