load process:

```text
lib_imports_2      lib_imports
     |                  |
     |                  v
     |          env_config_setting
     |                  |
     |                  v
     |               convert
     |                  |
     v                  v
ros_* modules        control
     |                  |
     |                  v
     +---------------> etc
                        |
                        v
                       main

ros_* modules:
- ros_get_cube_pose
- ros_get_robot_pose
- ros_set_robot_pose
```
