---
sidebar_position: 2
---

# rclpy Basics: Creating Nodes, Publishers, and Subscribers

## Getting Started with rclpy

`rclpy` is the Python client library for ROS 2. It provides the Python API for developing ROS 2 applications. In this section, we'll cover the fundamentals of creating ROS 2 nodes using Python.

### Creating Your First Node

Here's how to create a basic ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber

Subscribers in ROS 2 Python nodes are created similarly:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating Services

Services allow for request/response communication:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Best Practices

When working with `rclpy`, keep these best practices in mind:

1. **Always call `rclpy.init()`** before creating any nodes
2. **Use `rclpy.spin()`** to keep nodes running and processing callbacks
3. **Properly handle node destruction** in the shutdown sequence
4. **Use appropriate QoS settings** for your application's needs
5. **Log important events** using `self.get_logger().info()`

### Learning Objectives

After completing this section, you should be able to:
- Create basic ROS 2 nodes using rclpy
- Implement publishers to send messages
- Create subscribers to receive messages
- Build services for request/response communication
- Follow best practices for Python ROS 2 development