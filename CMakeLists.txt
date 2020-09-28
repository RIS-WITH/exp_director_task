
  cmake_minimum_required(VERSION 3.0.2)
  project(exp_director_task)

  find_package(catkin
    REQUIRED
    COMPONENTS
    rospy

    pr2_motion_tasks_msgs
    )

    catkin_package(

    )

    catkin_install_python(PROGRAMS scripts/ar_perception_node.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
