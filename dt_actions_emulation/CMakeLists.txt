cmake_minimum_required(VERSION 3.0.2)
project(dt_actions_emulation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  ontologenius
  roscpp
  cmake_modules
  std_msgs
  genmsg
  message_generation
)

find_package(ontologenius 0.2.6 REQUIRED)
find_package(TinyXML REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MementarAction.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS ontologenius roscpp std_msgs
   DEPENDS TinyXML
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ontologenius_lib_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(dt_actions_emulation_lib
  src/ActionFileReader.cpp
  src/Menu.cpp
)
add_dependencies(dt_actions_emulation_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(dt_actions_emulation_lib ${catkin_LIBRARIES} ontologenius_lib ${TinyXML_LIBRARIES})

add_executable(actions_emulation src/main.cpp)
add_dependencies(actions_emulation ${catkin_EXPORTED_TARGETS} dt_actions_emulation_gencpp)
target_link_libraries(actions_emulation dt_actions_emulation_lib)
