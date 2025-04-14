cmake_minimum_required(VERSION 3.0.2)
project(cw2_team_35) # change your team name here!

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  pcl_ros
  cw2_world_spawner
  message_generation  # for your .srv files
)

# find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io features visualization registration)
###################################
## Generate services and messages ##
###################################
add_service_files(
  FILES
  example.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_cw2_class_lib
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    moveit_ros_planning
    moveit_ros_planning_interface
    cw2_world_spawner
    message_runtime   # <-- important if generating msgs or srvs
  DEPENDS
    PCL
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ---- Library: cw2_class_lib --------------------------------------------
add_library(${PROJECT_NAME}_cw2_class_lib
  src/cw2_class.cpp
)

add_dependencies(${PROJECT_NAME}_cw2_class_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_cw2_class_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---- Executable: cw2_solution_node -------------------------------------
add_executable(cw2_solution_node src/cw2_node.cpp)

add_dependencies(cw2_solution_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cw2_solution_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_cw2_class_lib
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############
# (uncomment and adjust if you need to install targets/headers)

# install(TARGETS cw2_solution_node ${PROJECT_NAME}_cw2_class_lib
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

#############
## Testing ##
#############
# (uncomment if/when you have tests)
#
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(${PROJECT_NAME}_test test/test_cw2.cpp)
#   target_link_libraries(${PROJECT_NAME}_test
#       ${PROJECT_NAME}_cw2_class_lib
#       ${catkin_LIBRARIES}
#       ${PCL_LIBRARIES}
#   )
# endif()
