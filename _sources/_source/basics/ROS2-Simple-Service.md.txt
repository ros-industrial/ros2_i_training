# Simple Service in Python

## Introduction

ROS 2 services implement request-response communication between nodes. A
client sends a request, a server processes it, and the server returns one
response. Services are useful for operations that should be performed on
demand rather than continuously.

This exercise creates a Python service server and an asynchronous client for
the `example_interfaces/srv/AddTwoInts` service. The request contains two
integers:

```text
int64 a
int64 b
---
int64 sum
```

The fields above `---` belong to the request. The field below it belongs to
the response.

## 1. Create the package

Source ROS 2 and create a workspace if one does not already exist:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

Create an `ament_python` package:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 \
  py_srvcli --dependencies rclpy example_interfaces
```

The `--dependencies` option adds `rclpy` and `example_interfaces` to
`package.xml`.

The generated package has this structure:

```text
py_srvcli/
├── package.xml
├── py_srvcli/
│   └── __init__.py
├── resource/
│   └── py_srvcli
├── setup.cfg
└── setup.py
```

## 2. Write the server

Create `service.py` in the Python module directory:

```bash
cd ~/dev_ws/src/py_srvcli/py_srvcli
touch service.py
```

Add the following code:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b}'
        )

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.1 Create the service

```python
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_two_ints_callback
)
```

`create_service()` specifies:

1. the service type: `AddTwoInts`
2. the service name: `add_two_ints`
3. the callback that handles incoming requests

The server stores the returned service object in `self.srv` so that it remains
available for the lifetime of the node.

### 2.2 Process the request

```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b

    self.get_logger().info(
        f'Incoming request: {request.a} + {request.b}'
    )

    return response
```

The callback receives an `AddTwoInts.Request` and an empty
`AddTwoInts.Response`. It adds `request.a` and `request.b`, stores the
result in `response.sum`, and returns the completed response to the client.

### 2.3 Spin the server

```python
rclpy.spin(minimal_service)
```

The server must keep spinning so that its executor can receive requests and
invoke `add_two_ints_callback`. Stop it with `Ctrl+C`.

## 3. Write the client

Create `client.py` in the Python module directory:

```bash
cd ~/dev_ws/src/py_srvcli/py_srvcli
touch client.py
```

Add the following code:

```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'service not available, waiting again...'
            )

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b

        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Result of add_two_ints: {response.sum}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Service call failed: {e}'
            )


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    minimal_client.send_request(a, b)

    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4. Understand the client

### 4.1 Create the node and client

```python
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

`MinimalClientAsync` inherits from `Node`. The call to `create_client`
specifies:

1. the service type: `AddTwoInts`
2. the service name: `add_two_ints`

Both must match the server.

### 4.2 Wait for the server

```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info(
        'service not available, waiting again...'
    )
```

The client checks once per second until a matching service is available. This
prevents it from sending a request before the server is ready.

### 4.3 Send an asynchronous request

```python
req = AddTwoInts.Request()
req.a = a
req.b = b

future = self.cli.call_async(req)
future.add_done_callback(self.response_callback)
```

`call_async()` returns immediately with a future representing the pending
result. Registering `response_callback` tells ROS 2 what to execute when that
future finishes.

### 4.4 Process the response

```python
def response_callback(self, future):
    try:
        response = future.result()
        self.get_logger().info(
            f'Result of add_two_ints: {response.sum}'
        )
    except Exception as e:
        self.get_logger().error(
            f'Service call failed: {e}'
        )
```

`future.result()` returns the response or raises an exception if the request
failed.

### 4.5 Spin the node

```python
rclpy.spin(minimal_client)
```

Spinning lets the executor process the response callback. This client continues
spinning after it prints the result; stop it with `Ctrl+C`.

The program expects exactly two integer command-line arguments. Running it
without both arguments, or with non-integer values, causes an error.

## 5. Register the executables

Open `~/dev_ws/src/py_srvcli/setup.py` and add entry points for both nodes:

```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service:main',
        'client = py_srvcli.client:main',
    ],
},
```

The name before `=` is the executable passed to `ros2 run`. The value after
`=` identifies the Python module and its `main` function.

## 6. Build the package

From the workspace root, install dependencies and build:

```bash
cd ~/dev_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --packages-select py_srvcli
source install/setup.bash
```

## 7. Run the server and client

In the first terminal, source ROS 2 and the workspace, then start the server:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/dev_ws
source install/setup.bash
ros2 run py_srvcli service
```

In a second terminal, source ROS 2 and the workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/dev_ws
source install/setup.bash
```

Run the client with two integers:

```bash
ros2 run py_srvcli client 2 3
```

The client prints:

```text
[INFO] [minimal_client_async]: Result of add_two_ints: 5
```

Press `Ctrl+C` to stop the client after receiving the response. Use
`Ctrl+C` in the first terminal to stop the server.

## 8. Inspect the service

With the server running, inspect the available service and its interface:

```bash
ros2 service list
ros2 service type /add_two_ints
ros2 interface show example_interfaces/srv/AddTwoInts
```

The service can also be tested directly from the command line:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \
  "{a: 2, b: 3}"
```

The response contains `sum: 5`.
