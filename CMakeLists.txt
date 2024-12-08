cmake_minimum_required(VERSION 3.0.2)
project(yaskawa_cross_modal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  rviz_visual_tools
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  sun_robot_ros
  sensor_msgs
  pcl_ros
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
find_package(rosbag REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  grid.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  Tactile.action
  Visual.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   trajectory_msgs
   sensor_msgs
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yaskawa_cross_modal
  CATKIN_DEPENDS 
  actionlib 
  actionlib_msgs 
  geometry_msgs 
  roscpp 
  rospy 
  rviz_visual_tools 
  std_msgs 
  tf2 
  tf2_eigen 
  tf2_ros 
  sun_robot_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/yaskawa_cross_modal.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/yaskawa_cross_modal_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# catkin_install_python(PROGRAMS 
# scripts/dataset.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# catkin_install_python(PROGRAMS 
# scripts/model.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${rosbag_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable(task src/task.cpp)
# add_dependencies(task ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(task ${catkin_LIBRARIES})

add_executable(joint_state src/ros_network/joint_state.cpp)
add_dependencies(joint_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_state ${catkin_LIBRARIES})

add_executable(grid_srv src/ros_network/grid_srv.cpp)
add_dependencies(grid_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_srv ${catkin_LIBRARIES})

# add_executable(task_camera src/task_camera.cpp)
# add_dependencies(task_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(task_camera ${catkin_LIBRARIES})
# target_link_libraries(task_camera ${rosbag_LIBRARIES})

# add_executable(visualize_pcl src/visualize_pcl.cpp)
# add_dependencies(visualize_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(visualize_pcl ${catkin_LIBRARIES})
# target_link_libraries(visualize_pcl ${rosbag_LIBRARIES})

# add_executable(downsampling src/downsampling.cpp)
# add_dependencies(downsampling ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(downsampling ${catkin_LIBRARIES})
# target_link_libraries(downsampling ${rosbag_LIBRARIES})
# target_link_libraries(downsampling ${PCL_LIBRARIES})

# add_executable(icp src/icp.cpp)
# add_dependencies(icp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(icp ${catkin_LIBRARIES})
# target_link_libraries(icp ${rosbag_LIBRARIES})
# target_link_libraries(icp ${PCL_LIBRARIES})

# add_executable(interpolation src/interpolation.cpp)
# add_dependencies(interpolation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(interpolation ${catkin_LIBRARIES})
# target_link_libraries(interpolation ${rosbag_LIBRARIES})
# target_link_libraries(interpolation ${PCL_LIBRARIES})

# add_executable(post_process src/post_process.cpp)
# add_dependencies(post_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(post_process ${catkin_LIBRARIES})
# target_link_libraries(post_process ${rosbag_LIBRARIES})
# target_link_libraries(post_process ${PCL_LIBRARIES})

# add_executable(esf src/esf.cpp)
# add_dependencies(esf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(esf ${catkin_LIBRARIES})
# target_link_libraries(esf ${rosbag_LIBRARIES})
# target_link_libraries(esf ${PCL_LIBRARIES})

# add_executable(shot src/shot.cpp)
# add_dependencies(shot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(shot ${catkin_LIBRARIES})
# target_link_libraries(shot ${rosbag_LIBRARIES})
# target_link_libraries(shot ${PCL_LIBRARIES})

# add_executable(vfh src/vfh.cpp)
# add_dependencies(vfh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vfh ${catkin_LIBRARIES})
# target_link_libraries(vfh ${rosbag_LIBRARIES})
# target_link_libraries(vfh ${PCL_LIBRARIES})

# add_executable(pfh src/pfh.cpp)
# add_dependencies(pfh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pfh ${catkin_LIBRARIES})
# target_link_libraries(pfh ${rosbag_LIBRARIES})
# target_link_libraries(pfh ${PCL_LIBRARIES})

# add_executable(gasd src/gasd.cpp)
# add_dependencies(gasd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gasd ${catkin_LIBRARIES})
# target_link_libraries(gasd ${rosbag_LIBRARIES})
# target_link_libraries(gasd ${PCL_LIBRARIES})

# add_executable(search_initial_points src/search_initial_points.cpp)
# add_dependencies(search_initial_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(search_initial_points ${catkin_LIBRARIES})
# target_link_libraries(search_initial_points ${rosbag_LIBRARIES})
# target_link_libraries(search_initial_points ${PCL_LIBRARIES})

# add_executable(direction src/direction.cpp)
# add_dependencies(direction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(direction ${catkin_LIBRARIES})
# target_link_libraries(direction ${rosbag_LIBRARIES})

# add_executable(tactile_map src/tactile_map.cpp)
# add_dependencies(tactile_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(tactile_map ${catkin_LIBRARIES})
# target_link_libraries(tactile_map ${rosbag_LIBRARIES})

# add_executable(tactile_hessian src/tactile_hessian.cpp)
# add_dependencies(tactile_hessian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(tactile_hessian ${catkin_LIBRARIES})
# target_link_libraries(tactile_hessian ${rosbag_LIBRARIES})
# target_link_libraries(tactile_hessian ${PCL_LIBRARIES})

# add_executable(convert_PCL src/convert_PCL.cpp)
# add_dependencies(convert_PCL ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(convert_PCL ${catkin_LIBRARIES})
# target_link_libraries(convert_PCL ${rosbag_LIBRARIES})
# target_link_libraries(convert_PCL ${PCL_LIBRARIES})

# add_executable(segmentation src/segmentation.cpp)
# add_dependencies(segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(segmentation ${catkin_LIBRARIES})
# target_link_libraries(segmentation ${rosbag_LIBRARIES})
# target_link_libraries(segmentation ${PCL_LIBRARIES})

# add_executable(recognize src/recognize.cpp)
# add_dependencies(recognize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(recognize ${catkin_LIBRARIES})
# target_link_libraries(recognize ${rosbag_LIBRARIES})
# target_link_libraries(recognize ${PCL_LIBRARIES})

add_executable(tactile_exploration src/ros_network/tactile_exploration.cpp)
add_dependencies(tactile_exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tactile_exploration ${catkin_LIBRARIES})
target_link_libraries(tactile_exploration ${rosbag_LIBRARIES})
target_link_libraries(tactile_exploration ${PCL_LIBRARIES})

add_executable(task src/ros_network/task.cpp)
add_dependencies(task ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task ${catkin_LIBRARIES})
target_link_libraries(task ${rosbag_LIBRARIES})
target_link_libraries(task ${PCL_LIBRARIES})

add_executable(visual_exploration src/ros_network/visual_exploration.cpp)
add_dependencies(visual_exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_exploration ${catkin_LIBRARIES})
target_link_libraries(visual_exploration ${rosbag_LIBRARIES})
target_link_libraries(visual_exploration ${PCL_LIBRARIES})

add_executable(pre_processing src/ros_network/pre_processing.cpp)
add_dependencies(pre_processing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pre_processing ${catkin_LIBRARIES})
target_link_libraries(pre_processing ${rosbag_LIBRARIES})
target_link_libraries(pre_processing ${PCL_LIBRARIES})

# add_executable(pre_process src/pre_process.cpp)
# add_dependencies(pre_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pre_process ${catkin_LIBRARIES})
# target_link_libraries(pre_process ${rosbag_LIBRARIES})
# target_link_libraries(pre_process ${PCL_LIBRARIES})

# add_executable(script src/script.cpp)
# add_dependencies(script ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(script ${catkin_LIBRARIES})
# target_link_libraries(script ${rosbag_LIBRARIES})
# target_link_libraries(script ${PCL_LIBRARIES})

add_executable(segmentator src/ros_network/segmentator.cpp)
add_dependencies(segmentator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentator ${catkin_LIBRARIES})
target_link_libraries(segmentator ${rosbag_LIBRARIES})
target_link_libraries(segmentator ${PCL_LIBRARIES})

add_executable(downsample src/ros_network/downsample.cpp)
add_dependencies(downsample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(downsample ${catkin_LIBRARIES})
target_link_libraries(downsample ${rosbag_LIBRARIES})
target_link_libraries(downsample ${PCL_LIBRARIES})

add_executable(post_processing src/ros_network/post_processing.cpp)
add_dependencies(post_processing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(post_processing ${catkin_LIBRARIES})
target_link_libraries(post_processing ${rosbag_LIBRARIES})
target_link_libraries(post_processing ${PCL_LIBRARIES})

add_executable(interpolator src/ros_network/interpolator.cpp)
add_dependencies(interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(interpolator ${catkin_LIBRARIES})
target_link_libraries(interpolator ${rosbag_LIBRARIES})
target_link_libraries(interpolator ${PCL_LIBRARIES})

add_executable(cross_exploration src/ros_network/cross_exploration.cpp)
add_dependencies(cross_exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cross_exploration ${catkin_LIBRARIES})
target_link_libraries(cross_exploration ${rosbag_LIBRARIES})
target_link_libraries(cross_exploration ${PCL_LIBRARIES})

add_executable(reconstruct src/ros_network/reconstruct.cpp)
add_dependencies(reconstruct ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reconstruct ${catkin_LIBRARIES})
target_link_libraries(reconstruct ${rosbag_LIBRARIES})
target_link_libraries(reconstruct ${PCL_LIBRARIES})

# add_executable(division src/division.cpp)
# add_dependencies(division ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(division ${catkin_LIBRARIES})
# target_link_libraries(division ${rosbag_LIBRARIES})
# target_link_libraries(division ${PCL_LIBRARIES})

add_executable(euclidean_segmentation src/ros_network/euclidean_segmentation.cpp)
add_dependencies(euclidean_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(euclidean_segmentation ${catkin_LIBRARIES})
target_link_libraries(euclidean_segmentation ${rosbag_LIBRARIES})
target_link_libraries(euclidean_segmentation ${PCL_LIBRARIES})

add_executable(check_occlusion src/ros_network/check_occlusion.cpp)
add_dependencies(check_occlusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(check_occlusion ${catkin_LIBRARIES})
target_link_libraries(check_occlusion ${rosbag_LIBRARIES})
target_link_libraries(check_occlusion ${PCL_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yaskawa_cross_modal.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
