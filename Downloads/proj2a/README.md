# Running ROS2 Project 2a

## 1. Starting Application Order
The general order to begin your work is to open three terminals. In each
terminal, and in this order, run the following:

### 1.1 Start Gazebo on ros domain 67 to avoid cross talk with robot :
```bash
ROS_DOMAIN_ID=67  \
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py \
  world_path:=/home/you/274/workspace/worlds/lab.world \
  spawn_dock:=false \
  x:=0 y:=0

```
  - If others are working at the same time in the lab, you may all need to
    agree to different numbers and replace 67 with that number.

### 1.2 Start the script you are modifying :
```bash
colcon build --symlink-install
source install/setup.bash
ROS_DOMAIN_ID=67  \
ros2 run boustrophedon main
```
---

## 2. Simulate button-presses for the robot:
  This is how you send message to the robot. When we move to the actual bot, we
  will literally press the buttons.
```bash
ROS_DOMAIN_ID=67  \
ros2 topic pub --once  \
/interface_buttons  \
irobot_create_msgs/msg/InterfaceButtons "{  \
  button_1: {is_pressed: false},  \
  button_power: {is_pressed: true},  \
  button_2: {is_pressed: false}  \
}"
```

---

## 3. Expected responses before editing main.py:
[INFO] [1762207321.800925321] [main]: ROS_DOMAIN_ID: 0
[INFO] [1762207321.801170906] [main]: ROS_NAMESPACE: 
[INFO] [1762207321.801391253] [main]: RMW_IMPLEMENTATION: rmw_fastrtps_cpp
[INFO] [1762207321.901287012] [odom_node]: OdomNode ready (listening to /odom).
[INFO] [1762207321.904381306] [ir_node]: IrNode ready (using native sensor order).
[INFO] [1762207321.906092500] [button_node]: ButtonNode ready.
[INFO] [1762207321.908039482] [hazard_node]: HazardNode ready.
[INFO] [1762207321.909710249] [pulse_node]: PulseNode started, calling pulse() every 1.0s.
[INFO] [1762207322.384548479] [ir_node]: IR intensity messages are coming in!
[INFO] [1762207336.310028932] [Controller]: Hazard Detected
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 183.0   90.0   37.0  316.0  201.0  384.0  320.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 178.0   92.0   30.0  316.0  199.0  387.0  317.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 180.0   92.0   32.0  316.0  200.0  386.0  319.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 176.0   90.0   31.0  318.0  198.0  386.0  317.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 183.0   90.0   32.0  315.0  201.0  386.0  320.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 182.0   92.0   34.0  314.0  201.0  383.0  322.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 179.0   90.0   32.0  318.0  198.0  387.0  318.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 182.0   86.0   34.0  311.0  202.0  381.0  322.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 177.0   90.0   32.0  317.0  197.0  386.0  316.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 179.0   90.0   31.0  317.0  198.0  387.0  316.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 180.0   92.0   32.0  317.0  197.0  385.0  318.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 184.0   88.0   36.0  314.0  201.0  383.0  323.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 184.0   90.0   36.0  317.0  203.0  387.0  324.0
x=-0.254, y=0.234, theta=1.566rad
Left->Right IR readings: 180.0   91.0   32.0  317.0  198.0  388.0  325.0
