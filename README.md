# 2026a_nav

ROS 2 Jazzy / Gazebo Harmonicを使った2026NHKロボコンのシミュレーション環境。

## 自己位置推定

```bash
ros2 launch ./launch/localization_launch.py
```

## SLAM

```bash
ros2 launch /home/yamana/workspace/y_robo/ros2_ws/2026a_nav/launch/slam_launch.py
```

## シミュレーションのみ

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch /home/yamana/workspace/y_robo/ros2_ws/2026a_nav/launch/sim_launch.py
```

## topic
| ROS側 | Gazebo側 | 方向 |
| --- | --- | --- |
| `/clock` | `/world/2026_nhk_field/clock` | Gazebo → ROS |
| `/scan` | `/scan` | Gazebo → ROS |
| `/odom` | `/odom` | Gazebo → ROS |
| `/tf` | `/odom_tf` | Gazebo → ROS |
| `/cmd_vel` | `/cmd_vel` | ROS → Gazebo |


### cmd_vel

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
