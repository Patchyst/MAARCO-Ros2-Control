# Terrain ID and Behavior Tree 
Behavior tree for changing locomotion methods based on terrain identification. For Multi-terrain Amphibious ARCtic explOrer (MAARCO) at NCSU.
<img width="734" height="295" alt="image" src="https://github.com/user-attachments/assets/461238c6-6bd0-4604-ad16-1f8148069f0f" />

# Behavior Tree
<img width="1328" height="345" alt="image" src="https://github.com/user-attachments/assets/5c764ccd-6459-4c0a-9cf3-3f4670d723af" />

# Build Instructions
Clone the repo, place it in the `src` folder of a ros2 workspace. Then:
```bash
colcon build --symlink-install
source install/setup.bash
```

Packages can be listed via
```bash
ros2 pkg list
```

and nodes in that package:

```bash
ros2 pkg executables <pkg name>
```

# Run Instructions
To run the behavior tree node:
```bash
ros2 run maarco_bt heading_bt_node
```
To run the pd controller, motor driver, and serial reader node. Run this launch file:
```
ros2 launch maarco_bt maarco_control.launch.py
```

### To run them individually:
To run the pd controller:
```bash
ros2 run heading_controller heading_controller
```

To run the motor driver:
```bash
ros2 run motor_driver motor_driver
```

To run the serial reader node:
```bash
ros2 run serial_reader serial_node
```


