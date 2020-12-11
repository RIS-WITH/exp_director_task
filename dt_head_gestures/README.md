# Director's Task Head Gestures Package

A ROS package regrouping all high level head motions. Meant to use with the pr2_head_manager package.

## Dependencies

 - **pr2_head_manager_msgs** : *Compilation & Execution*
 - **resource_management_msgs** : *Compilation & Execution*
 - **resource_management**     : *Execution*
 - **pr2_head_manager**     : *Execution*

# Install

```
git clone https://gitlab.com/laas-hri/resource_management.git
git clone https://gitlab.com/laas-hri/pr2_resources_management_demo/pr2_head_manager.git
git clone https://gitlab.com/laas-hri/pr2_resources_management_demo/pr2_head_manager_msgs.git
```

# Usage

```
rosrun pr2_head_manager pr2_head_manager
rosrun dt_head_gestures head_scan_node
```

Here is an example with the following request:


```bash
rosservice call /dt_head_gestures/head_scan "central_point:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: 'base_footprint'
  point: {x: 2.0, y: 0.0, z: 1.5}
height: 1.0
width: 1.0
step_length: 0.3
duration_per_point:
  data: {secs: 2, nsecs: 0}"
```

[![You should see the Pr2 scanning the area](https://img.youtube.com/vi/IBR9xku5Jrc/0.jpg)](https://www.youtube.com/watch?v=IBR9xku5Jrc)
