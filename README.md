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

### From source

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

### Binary install

Coming soon!
