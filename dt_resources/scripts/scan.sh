rosservice call /head_scan/head_scan "central_point:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: 'base_footprint'
  point: {x: 1.2, y: 0.0, z: 0.5}
height: 0.5
width: 2
step_length: 0.3
duration_per_point:
  data: {secs: 2, nsecs: 0}"

