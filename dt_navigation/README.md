# dt_navigation
A small action server handling short base displacements for tabletop scenarios.

## Usage
Use the launch file to start a full navigation stack (with map server, ...)
Send a request to the action server.
Request:
* `move_type`: `GO_IN_FRONT` or `GO_NEXT` (not supported yet). `GO_IN_FRONT` move the robot in front of `frame_id` while staying parallel to the y axis of the `table_frame` parameter.

Response:
* `error_code`: The return error code (`SUCCESS = 0`, `NO_TABLE_TO_OBJ_TRANSFORM = -1`, `NO_TABLE_TO_FOOTPRINT_TRANSFORM = -2`, `NO_MAP_TO_TABLE_TRANSFORM = -3`, `PREEMPTED = -4`
* `action_end`: Timestamp when the navigation action ended (rostime)

Feedback:
* `distance_to_goal`: Distance left between the robot and the computed point to reach (in meters)
* `action_start`: Timestamp when the navigation actually started (rostime, 0 if not started yet)

## Parameters
Parameters are read at each control loop (i.e. you can change it while the robot is moving)
* `goal_tolerance`: Position tolerence for goal acceptance (in meters)
* `max_speed`: Maximum speed allowed for the robot (in meters/seconds)
* `P`: Proportional gain of the position controller
* `I`: Integral gain of the position controller
* `max_integral`: Maximum value (saturation) for the integral component of the position controller (kind of anti windup)
* `control_period`: The position control period (in seconds)
* `table_frame`: The frame id of the table to stay parallel to 
