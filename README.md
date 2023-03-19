# ros_2cli

Brand-compliant `ROS 2` and `ROS 1` command aliases.
See [shouldiputaspacebetweenrosand2.com](https://shouldiputaspacebetweenrosand2.com/).

| Command... | ...calls | Example... | ...calls |
|:--------|:------|:--------|:--------------|
| `ROS 2 ...` | `ros2 ...` | `ROS 2 topic list` | `ros2 topic list` |
| `ROS 1 ...` | `ros...`   | `ROS 1 topic list` | `rostopic list`   |

The `ROS 2` command alias supports autocompletion:

```sh
$ ROS <tab>
1 2 -h --help
$ ROS 2 <tab>
action                          interface                       run
bag                             launch                          security
component                       lifecycle                       service
daemon                          multicast                       topic
doctor                          node                            --use-python-default-buffering
extension_points                param                           wtf
extensions                      pkg
$ ROS 2 topic <tab>
bw                       hz                       pub
delay                    --include-hidden-topics  type
echo                     info                     
find                     list
$ ROS 2 topic list
/email
/parameter_events
/rosout
```

## How to use

### Build from source

1. Clone package into your workspace
   ```sh
   $ cd ~/ws/src
   $ git clone https://github.com/christophebedard/ros_2cli.git
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

### Install binary

For ROS 2 Rolling on Ubuntu 22.04:

1. Add this GitHub repository as an `apt` and `rosdep` repository and update
   ```sh
   $ echo "deb [trusted=yes] https://raw.githubusercontent.com/christophebedard/ros_2cli/jammy-rolling/ ./" | sudo tee /etc/apt/sources.list.d/christophebedard_ros_2cli.list
   $ echo "yaml https://raw.githubusercontent.com/christophebedard/ros_2cli/jammy-rolling/local.yaml rolling" | sudo tee /etc/ros/rosdep/sources.list.d/1-christophebedard_ros_2cli.list
   $ sudo apt-get update
   $ rosdep update
   ```
1. Install `ros_2cli`
   ```sh
   $ sudo apt-get install -y ros-rolling-ros-2cli
   ```
1. Source
   ```sh
   $ source /opt/ros/rolling/setup.bash
   ```
1. Use commands
   ```sh
   $ ROS 2 run pkg exe
   $ ROS 1 topic list
   $ # etc.
   ```
