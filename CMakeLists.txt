cmake_minimum_required(VERSION 2.8.3)
project(abb_libegm_samples)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(abb_libegm REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS abb_libegm
)

###########
## Build ##
###########

include_directories(
 ${catkin_INCLUDE_DIRS}
)

#################################
# Build the samples #
#################################


add_executable(${PROJECT_NAME}_a1_joint_trajectory_node src/a1_joint_trajectory_node.cpp)
target_include_directories(${PROJECT_NAME}_a1_joint_trajectory_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_a1_joint_trajectory_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_a2_pose_trajectory_node src/a2_pose_trajectory_node.cpp)
target_include_directories(${PROJECT_NAME}_a2_pose_trajectory_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_a2_pose_trajectory_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)



add_executable(${PROJECT_NAME}_c1_joint_controller_node src/c1_joint_controller_node.cpp)
target_include_directories(${PROJECT_NAME}_c1_joint_controller_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_c1_joint_controller_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_c2_pose_controller_node src/c2_pose_controller_node.cpp)
target_include_directories(${PROJECT_NAME}_c2_pose_controller_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_c2_pose_controller_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_a2_test src/a2_test.cpp)
target_include_directories(${PROJECT_NAME}_a2_test PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_a2_test PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)
