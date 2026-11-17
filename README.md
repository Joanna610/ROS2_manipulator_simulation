# ROS2_manipulator_simulation

To start the simulation we have to use the below commands:

```bash
source install/setup.bash
colcon build --packages-select mani_gz_sim
ros2 launch mani_gz_sim ros_gz_sim_launch.py
```

View of the camera (in another terminal)
```bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

The example of the manipulator's motion
```bash
source install/setup.bash
ros2 run mani_gz_sim move_joint
```
