rosservice call /head_scan/head_scan "central_point:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: 'base_footprint'
  point: {x: 1.2, y: 0.0, z: 0.6}
height: 0.3
width: 1.5
step_length: 0.2
duration_per_point:
  data: {secs: 1, nsecs: 500000000}"

