# Creating ROS Publisher and Subscriber Nodes

In this activity, we will create 2 simple ROS nodes.
In ROS terminology, a node is a small program which fulfills a specific purpose.
A robot typically has many ROS nodes running simulaneously.

This activity is based on the tutorial [here](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/).

## Background on ROS terminology

ROS nodes communicate by sending and receiving messages.
Messages are organized by topics.
When a ROS node sends messages to a topic, it is called a publisher, and when it listens to a topic for messages it is called a subcriber.

You can read more about nodes and topics on these pages:
- https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Nodes/
- https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/

## Create the publisher node

In your preferred text editor, open the file `spear-turtlebot/publisher_subscriber/publisher_subscriber/publisher_member_function.py`.

The file should just have a single line:
```python
# TODO put publisher code here
```

Replace that line with the following code:

```python
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

## Explanation of publisher code

Let's take a closer look at the publisher code.

First, these lines import the ROS client libraries neccessary to run this python file.
We need to import the `Node` class since we will be extending it.
We also need to import the `String` message type.
There are many different message types built into ROS and you can also create your own.
For this tutorial, our publisher will just publish `String` type messages.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

This declares a new python class called `MinimalPublisher` which extends the base `Node` class from ROS.

```python
class MinimalPublisher(Node):
```

This calls the `Node` class contructor with the argument `'minimal_publisher'`. This argument is the name of the node.

```python
        super().__init__('minimal_publisher')
```

This creates a publisher which will publish `String` type messages on a topic called `topic` and a queue size of 10.
Queue size is the maximum number of messages to save if subscribers are reading messages fast enough.

```python
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

This creates a timer that will call the method `timer_callback` every 0.5 seconds.
`self.i` is a counter which we will initialize at zero.

```python
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
```

This is the implementation of the `timer_callback` method which will be called every 0.5 seconds.
This method creates a new `String` message and fills in the message data with `Hello World: ` and the current value of the counter.
It then asks the publisher to publish the message.

The call to `self.get_logger().info()` prints out a message in the terminal where this node is running.
We will see this message later when we run the node.

Finally, this method increments the counter.

```python
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

All of this code is required to set up the node and register it with ROS.
The call to `rclpy.spin()` starts running the node and its timer.

```python
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

## Create the subscriber node

In your preferred text editor, open the file `spear-turtlebot/publisher_subscriber/publisher_subscriber/subscriber_member_function.py`.

The file should just have a single line:
```python
# TODO put subscriber code here
```

Replace that line with the following code:

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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Explanation of subscriber code

Again we are going to be creating a class that extends the `Node` class and uses `String` type messages:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

The contructor for our `MinimalSubscriber` class does the following:
- Call the `Node` contructor and name the node `minimal_subscriber`
- Create a new subscription with the following arguments:
  - The messages we expect to receive are of `String` type
  - The name of the topic to subscribe to it `topic` (this matches the topic name in the publisher)
  - The method we want to call every time a message is received is `self.listener_callback`
  - The queue size is 10

```python
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
```

Then, we define the callback method which will be called every time a message is received.
In this case, the callback just prints out the message that was received.


```python
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

The rest of the code is the same as the publisher.

## Add dependencies and entry points

We need to inform the ROS build system which libraries our nodes depend on.
In ROS, nodes are grouped into packages and we declare dependencies per package.
The name of the package that these nodes where created in is `publisher_subscriber`.
The file where we specify dependencies for this package is `spear-turtlebot/publisher_subscriber/package.xml`.

If you open that file in a text editor, you should see this line:
```xml
  <!-- TODO add dependencies here -->
```

Replace that line with these lines:
```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

This tells the ROS build system that executing the `publisher_subscriber` package depends 2 other packages called `rclpy` and `std_msgs`.
The `rclpy` package contains the base `Node` class and the `std_msgs` package contains the `String` message type.

Now we need to add entrypoints.
Open the file `spear-turtlebot/publisher_subscriber/setup.py`.

Find this line:
```python
            # TODO add entry points for publisher and subcriber nodes
```

Replace it with the following:


```python
            'talker = publisher_subscriber.publisher_member_function:main',
            'listener = publisher_subscriber.subscriber_member_function:main',
```

## Running everything

First navigate to the ROS workspace in a terminal.

If you are using docker, first run a docker container:
```bash
docker-compose run spear-turtlebot
```

Then, inside the docker container navigate the the workspace:
```bash
cd /opt/ros/overlay_ws
```

Once you have navigated to the workspace, run this command to build the `publisher_subscriber` package:

```bash
colcon build --packages-select publisher_subscriber --symlink-install
```

Now that we have built the package, we are ready to run each node.
We need a separate terminal to run each of the 2 nodes.
We will use tmux to open 2 terminals within the docker image.

To start a new tmux session, simply run this command:

```bash
tmux
```

After you do that, you should see a yellow bar appear at the bottom of your terminal.

Here is a crash course on tmux:
* In tmux, we can create new terminal windows by hitting "Ctrl-b" then "c".
* To switch between windows, hit "Ctrl-b", then the number of the window.
  * For example, to go to the first window (usually numbered 0), hit "Ctrl-b" then "0".
* To close a tmux window, simply hit "Ctrl-d"

Feel free to google tmux tutorials if you would like to learn more.
Also see the "Further Resources" section of the main new member activity page.

In this tmux window, you can run the publisher as follows:
```bash
ros2 run publisher_subscriber talker
```

You should see messages like this appear every 0.5 seconds:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```

Once the publisher is running, open a new tmux window and run this:
```bash
ros2 run publisher_subscriber listener
```

You should see messages like this appear every 0.5 seconds:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

To stop the nodes, press "Ctrl-c" in each of the tmux windows.

When you have finished and stopped the nodes, press "Ctrl-d" to close each tmux window and again to exit the docker container.

