# ros_2cli

Brand-compliant `ROS 2` annd `ROS 1`/`ROS` command aliases.
See [shouldiputaspacebetweenrosand2.com](https://shouldiputaspacebetweenrosand2.com/).

| Command... | ...calls | Example... | ...calls |
|:--------|:------|:--------|:--------------|
| `ROS 2 ...` | `ros2 ...` | `ROS 2 topic list` | `ros2 topic list` |
| `ROS 1 ...` | `ros...`   | `ROS 1 topic list` | `rostopic list`   |
| `ROS ...`   | `ros...`   | `ROS topic list`   | `rostopic list`   |

Lowercase `ros` is also supported.

## How to use

### From source

1. Clone package into your workspace
   ```sh
   $ cd ~/ws/src
   $ git clone https://github.com/christophebedard/ros-2cli.git
   ```
1. Build package
   ```sh
   $ cd ~/ws
   $ colcon build --packages-up-to ros_2cli
   ```
1. Source
   ```sh
   $ source install/setup.bash
   ```
1. Use commands
   ```sh
   $ ROS 2 run pkg exe
   $ ROS 1 topic list
   $ # etc.
   ```

### Binary install

Coming soon!
