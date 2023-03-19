```bash
echo "deb [trusted=yes] https://raw.githubusercontent.com/christophebedard/ros_2cli/jammy-rolling/ ./" | sudo tee /etc/apt/sources.list.d/christophebedard_ros_2cli.list
echo "yaml https://raw.githubusercontent.com/christophebedard/ros_2cli/jammy-rolling/local.yaml rolling" | sudo tee /etc/ros/rosdep/sources.list.d/1-christophebedard_ros_2cli.list
```
