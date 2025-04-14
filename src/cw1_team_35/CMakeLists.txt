cmake_minimum_required(VERSION 3.0.2)
project(cw1_team_35)

## find_package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  pcl_ros          # <-- 包含 pcl_ros
  pcl_conversions
  cw1_world_spawner
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  io
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    cw1_world_spawner
    cv_bridge
    image_transport
    pcl_ros
    pcl_conversions
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # <-- PCL 头文件
)

## 1) 生成库
add_library(${PROJECT_NAME}_cw1_class_lib
  src/cw1_class.cpp
)

## 让这个库链接 catkin 及 PCL
target_link_libraries(${PROJECT_NAME}_cw1_class_lib
  ${catkin_LIBRARIES}  # <-- 关键：包含了 pcl_ros 等
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_cw1_class_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## 2) 生成可执行文件
add_executable(cw1_solution_node
  src/cw1_node.cpp
)

target_link_libraries(cw1_solution_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_cw1_class_lib  # <-- 链接到自己生成的库
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  # 还可以加上 ${Boost_LIBRARIES}、${cv_bridge_LIBRARIES} 等
)

add_dependencies(cw1_solution_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
