# A Simple Publisher and Subscriber

## 1. Create a package

in the ros2 ws/src Run the package creation

```
ros2 pkg create --build-type ament_python py_pubsub
```
 terminal will display a message confirming the creation of package py pubsub and all of its necessary files and folders.
 
 ## 2. Write the publisher node
 
 we can download the example talker code by going to ros2 ws/src/py pubsub/py pubsub and typing the following command:
 
 ```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

 Open the file in your preferred text editor. The __init.py file will now be followed by a new one called publisher member function.py.
 
 ```
 import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    ```
    
    
    ## 2.1 Add dependencies
    Setup.py, setup.cfg, and package.xml files already produced for you in ros2 ws/src/py pubsub will be returned.
    
    Be sure to fill in all the tags in package.xml, including description>, maintainer>, and license>.
    
   ```
   <description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
we need to add the following dependencies after the import declarations for your node

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

To run the package's code, rclpy and standard messages are required.
make sure to save the file 
