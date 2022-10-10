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
wgethttps://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
![image](https://user-images.githubusercontent.com/92859942/194820798-ebd90a9c-5f13-46cb-b3a9-35fa0644946b.png)


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
![image](https://user-images.githubusercontent.com/92859942/194821468-308e1455-b9fd-42e2-9f6c-72132ecc902e.png)

To run the package's code, rclpy and standard messages are required.
make sure to save the file 

## 2.2 Add an entry point

Check setup.py.Double-check package.xml to be sure its maintainer, maintainer email, description, and license columns are correct.

```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
Add the next line to the entry points field between the console scripts brackets:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
![image](https://user-images.githubusercontent.com/92859942/194822682-d2169521-7ba9-49d9-9853-cade3406f1b8.png)

we need to save here as well 


## 2.3 Check setup.cfg

the following command should use first 

```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```

So that ros2 run knows where to find your executables, tell setuptools to place them in the lib directory.If you wanted to see the full system in operation, you could create your package and source the local setup files right now.The subscriber node needs to be established first, however.

## 3 Write the subscriber node

we should create node using following command 

```
wgethttps://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
![image](https://user-images.githubusercontent.com/92859942/194822885-ca202986-9f67-4f33-addf-5132a3d63121.png)

below mentioned lines should be in directory 

```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

in the text editor navigate to subscriber member function.py.

```
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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
    
    
## 3.1 Add an entry point

Delete the entry point for the publisher and reopen setup.py with an entry point for the subscriber below it.You should now see the following entry points in the entry points field:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
![image](https://user-images.githubusercontent.com/92859942/194823251-a29af71b-984f-4f13-be9c-207eb35d4dba.png)

after saving the system should function

# 4 Build and Run

after rclpy and std mags already installed, we need first to check the any indepencies are missing or not 

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Still in the root of your workspace, ros2_ws, build your new package:

```
colcon build --packages-select py_pubsub
```
![image](https://user-images.githubusercontent.com/92859942/194831334-30912a35-4222-4503-95da-64f6a0b27871.png)


Navigate to ros2 ws in a new terminal, then source the setup files and run the talker mode 

```
. install/setup.bash
```
```
ros2 run py_pubsub talker
```

we can see the terminal sending the following message 

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```
![image](https://user-images.githubusercontent.com/92859942/194831442-81b91ccf-496c-4675-8bfb-5b3c08752c45.png)


in short time, the terminal starts the informational messages as below and listener begins writing message to the console starting at the publisher's current message count as follows:

```
ros2 run py_pubsub listener
```

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```
in this cace , ctrl+c will stop the nodes progress.



# A Simple Service and Client

## 1. create package

in terminal we need to run package creation command 

```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```
we will see the conformation 

![image](https://user-images.githubusercontent.com/92859942/194831723-14c1d311-ba4b-42ae-8a20-9668b390675c.png)

## 1.1 update(package.xml) 

we should always remember to include the description, license information, and the name and email of the maintainer in package.xml. but we dont need to manually add dependencies to package.xml.

```
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
![image](https://user-images.githubusercontent.com/92859942/194832328-009297dc-0f7f-432f-a1f5-fd0af93e63db.png)


## 1.2 Update (setup.py)


details should be added to the setup.py file's description, maintainer, maintainer email, and license fields

```
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
![image](https://user-images.githubusercontent.com/92859942/194832769-09ef8dcf-d327-4dcd-81a9-7538533b961b.png)

# 2 Write the service node

we need to make new file called function.py, and run the following code. 

```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


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

## 2.1 Add an entry point

the entry point must be added to be able to run node located in the ros2 ws/src/py srvcli directory

insert the code between the "console scripts" brackets

```
'service = py_srvcli.service_member_function:main',
```
![image](https://user-images.githubusercontent.com/92859942/194835145-5bbfd63b-8a31-4a22-a966-e19c06c8241b.png)

## 3 Write the client node

![image](https://user-images.githubusercontent.com/92859942/194834297-f7060b85-519d-4226-8945-16f92a5b1ad1.png)

again we need to make new file called client member  function.py  and use the code mentioned below .

```
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3.1 Add an entry point


In the same way that the service node requires an entry point, the client node also does.It is important to format your setup.py file's entry points column the following way:


```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```
![image](https://user-images.githubusercontent.com/92859942/194835869-1e8769e6-78d4-4991-97aa-c202e0f99867.png)


## 4 Build and Run
we need to check any dependencies are missing or not 

```
rosdep install -i --from-path src --rosdistro foxy -y
```

now we able to create own package 

```
colcon build --packages-select py_srvcli
```

in the new terminal source the setup files and run the service node right after 

```
. install/setup.bash
```

```
ros2 run py_srvcli service
```

![image](https://user-images.githubusercontent.com/92859942/194839726-e1ab4c3d-89c7-44d5-8dc7-66072ae7343f.png)

Until a request is made by the client, the node will hold off.Open a new terminal and re-source the setup files from ros2 ws.Two integers, separated by a space, and the client node.


```
ros2 run py_srvcli client 2 3
```
![image](https://user-images.githubusercontent.com/92859942/194839816-d0790efd-8156-4660-ab6e-87da06bd0928.png)

the customer will receive the response if we choose 2 or 3 option 

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```

in servive node running terminal it will published the following log statements after receiving the request

```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

to stop the nodes from rotating ctrl+c command is used. 


# Creating custom msg and srv files

## 1 Create a new package

Navigate into ros2_ws/src and run the package creation command:

```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

Upon establishing your package tutorial interfaces and all necessary files and folders, a message will appear on your terminal.It should be noted that this is a CMake package since Python packages are unable to generate .msg or .srv files.A Python node can use a custom interface you create in a CMake package, which will be discussed in the last section.

The directories should be made in the ros2 ws/src/tutorial interfaces.

```
mkdir msg

mkdir srv
```
![image](https://user-images.githubusercontent.com/92859942/194840895-69308228-2cd5-4356-a9ce-7685bb1ae08f.png)

## 2 Create custom definitions

## 2.1 msg definition

in new file named Num.msg add single line of code which describe the data structure 

```
int64 num
```

This custom message sends a single value, the 64-bit integer num.

In the directory we just created for the tutorial interfaces/msg, make a new file called Sphere.msg and use command :

```
geometry_msgs/Point center
float64 radius
```

A message from a different message package—in this case, geometry msgs/Point—is used in this custom message.

## 2.2 srv definition

In the instructional interfaces/srv directory, new file should created named 
AddThreeInts.srv with following structure 

```
int64 a
int64 b
int64 c
---
int64 sum
```
This custom service accepts the three integers a, b, and c and provides the answer's integer sum.

![image](https://user-images.githubusercontent.com/92859942/194843782-cf0f9aa2-ba12-4161-99b6-6b0a983cf65d.png)

## 3 CMakeLists.txt

The following lines in CMakeLists.txt will translate the interfaces you created into code that can be used in C++ and Python:

```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```
![image](https://user-images.githubusercontent.com/92859942/194844391-17978213-b921-437b-9307-7e3ded38d936.png)

## 4 package.xml

```
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```


here in package.xml, added a above command .

## 5 Build the tutorial_interfaces package


we may now construct our own interfaces package as all of its components are in place. In the workspace's root directory (/ros2 ws), issue the following command:
and other ROS2 application will be able to locate the interface 

```
colcon build --packages-select tutorial_interfaces
```

![image](https://user-images.githubusercontent.com/92859942/194845223-490653a2-c8f6-4b92-8d80-d4b6b9c84f20.png)

## 6 Confirm msg and srv creation

Use the following command from within your workspace (ros2 ws) to source it in a new terminal, after that next command helps to check your interface creation was successful .

```
. install/setup.bash
```

```
ros2 interface show tutorial_interfaces/msg/Num
```

use the command one by one which should return and so on . 

```
int64 num
```

```
ros2 interface show tutorial_interfaces/msg/Sphere
```

```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
```
int64 a
int64 b
int64 c
---
int64 sum
```
![image](https://user-images.githubusercontent.com/92859942/194845600-f4f03c68-4b68-4112-a8bd-8ea2f00477f1.png)


## 7 Test the new interfaces

here we can make use of package created earlier, the process to enable package file to use new interface is by adjustments to the nodes, CMakeLists .

# 7.1 Testing Num.msg with pub/sub
we  can see Num.msg in action by making a few minor adjustments to the publisher/subscriber package developed in a prior course (in C++ or Python). our decision to switch from the default textual message to a numerical one will somewhat alter the outcome.

publisher : 

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

subscriber :

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
CMakeLists.txt:

Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
![image](https://user-images.githubusercontent.com/92859942/194846662-082ca9de-f5dd-481d-b6c7-3d09aa018aee.png)

package.xml:
after adding the below mentioned line we can create the package after making adjustment and save the work.

```
<exec_depend>tutorial_interfaces</exec_depend>
```

```
colcon build --packages-select py_pubsub
```

in the new window we can add following command followed by opening the two terminal and run the ros2 ws in each by using given command 

```
colcon build --merge-install --packages-select py_pubsub
```

```
ros2 run py_pubsub talker
```
![image](https://user-images.githubusercontent.com/92859942/194846960-86ce7059-12e0-4168-81cd-f773d82e3707.png)

```
ros2 run py_pubsub listener

```

The talker should only be transmitting integer values, as opposed to the text it previously broadcast as Num.msg only transmits an integer:

![image](https://user-images.githubusercontent.com/92859942/194846859-adb6cb76-ad08-4ffb-8a55-43d1f0dbc0f3.png)

```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

## 7.2 Testing AddThreeInts.srv with service/client

By making a few minor adjustments to the service/client package developed in a prior lesson (in C++ or Python), AddThreeInts.srv may be utilized. The outcome will be very different because you'll be switching from the first two integer request srv to a three integer request srv.

service : 

```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

client: 

```
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
CMakeLists.txt:

Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml:
Add the following command :

```
<exec_depend>tutorial_interfaces</exec_depend>
```
After making the aforementioned alterations and saving your work, create the package .

```
colcon build --packages-select py_srvcli
```
![image](https://user-images.githubusercontent.com/92859942/194849463-af6ff6fc-4e55-4b22-a9c1-a535f25f381f.png)

on windows: 

```
colcon build --merge-install --packages-select py_srvcli
```

now we able to luvch two terminal and excute the work 

```
ros2 run py_srvcli service
```
![image](https://user-images.githubusercontent.com/92859942/194849406-373e8c42-7434-42b1-a090-e0cdbcd62fac.png)

```
ros2 run py_srvcli client 2 3 1
```
![image](https://user-images.githubusercontent.com/92859942/194849177-572c320e-ff9e-4ea8-96e2-85592506777c.png)





