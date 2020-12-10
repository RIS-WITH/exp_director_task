# Director's Task Head Gestures Package

A ROS package regrouping all high level head motions. Meant to use with the pr2_head_manager package.

Here is an example with the following request:

```bash
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