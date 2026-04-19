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

```
ros2 pkg executables <pkg name>
```

# Run Instructions
To run the behavior tree node:
```bash
ros2 run maarco_bt heading_bt_node
```

