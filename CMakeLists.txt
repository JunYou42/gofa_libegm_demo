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
# Build the basic level samples #
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


# set_target_properties(${PROJECT_NAME}_joint_trajectory_node PROPERTIES OUTPUT_NAME a1_joint_trajectory_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_joint_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_joint_trajectory_node ${catkin_LIBRARIES})



# add_executable(${PROJECT_NAME}_pose_trajectory_node src/a2_pose_trajectory_node.cpp)
# set_target_properties(${PROJECT_NAME}_pose_trajectory_node PROPERTIES OUTPUT_NAME a2_pose_trajectory_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_pose_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_pose_trajectory_node ${catkin_LIBRARIES})
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
# ########################################
# # Build the intermediate level samples #
# ########################################

# add_executable(${PROJECT_NAME}_joint_static_goal_node src/b1_joint_static_goal_node.cpp)
# set_target_properties(${PROJECT_NAME}_joint_static_goal_node PROPERTIES OUTPUT_NAME b1_joint_static_goal_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_joint_static_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_joint_static_goal_node ${catkin_LIBRARIES})
add_executable(${PROJECT_NAME}_b1_joint_static_goal_node src/b1_joint_static_goal_node.cpp)
target_include_directories(${PROJECT_NAME}_b1_joint_static_goal_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_b1_joint_static_goal_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)
# add_executable(${PROJECT_NAME}_pose_static_goal_node src/b2_pose_static_goal_node.cpp)
# set_target_properties(${PROJECT_NAME}_pose_static_goal_node PROPERTIES OUTPUT_NAME b2_pose_static_goal_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_pose_static_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_pose_static_goal_node ${catkin_LIBRARIES})
add_executable(${PROJECT_NAME}_b2_pose_static_goal_node src/b2_pose_static_goal_node.cpp)
target_include_directories(${PROJECT_NAME}_b2_pose_static_goal_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_b2_pose_static_goal_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)
# ####################################
# # Build the advanced level samples #
# ####################################

# add_executable(${PROJECT_NAME}_joint_controller_node src/c1_joint_controller_node.cpp)
# set_target_properties(${PROJECT_NAME}_joint_controller_node PROPERTIES OUTPUT_NAME c1_joint_controller_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_joint_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_joint_controller_node ${catkin_LIBRARIES})
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
# add_executable(${PROJECT_NAME}_pose_controller_node src/c2_pose_controller_node.cpp)
# set_target_properties(${PROJECT_NAME}_pose_controller_node PROPERTIES OUTPUT_NAME c2_pose_controller_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_pose_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_pose_controller_node ${catkin_LIBRARIES})
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
# add_executable(${PROJECT_NAME}_joint_velocity_controller_node src/c3_joint_velocity_controller_node.cpp)
# set_target_properties(${PROJECT_NAME}_joint_velocity_controller_node PROPERTIES OUTPUT_NAME c3_joint_velocity_controller_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_joint_velocity_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_joint_velocity_controller_node ${catkin_LIBRARIES})
add_executable(${PROJECT_NAME}_c3_joint_velocity_controller_node src/c3_joint_velocity_controller_node.cpp)
target_include_directories(${PROJECT_NAME}_c3_joint_velocity_controller_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_c3_joint_velocity_controller_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)
# add_executable(${PROJECT_NAME}_pose_velocity_controller_node src/c4_pose_velocity_controller_node.cpp)
# set_target_properties(${PROJECT_NAME}_pose_velocity_controller_node PROPERTIES OUTPUT_NAME c4_pose_velocity_controller_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_pose_velocity_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_pose_velocity_controller_node ${catkin_LIBRARIES})
add_executable(${PROJECT_NAME}_c4_pose_velocity_controller_node src/c4_pose_velocity_controller_node.cpp)
target_include_directories(${PROJECT_NAME}_c4_pose_velocity_controller_node PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_c4_pose_velocity_controller_node PRIVATE
  abb_libegm::abb_libegm
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_gofa_test_joint_trajectory src/gofa_test_joint_trajectory.cpp)
target_include_directories(${PROJECT_NAME}_gofa_test_joint_trajectory PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
   "${catkin_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}_gofa_test_joint_trajectory PRIVATE
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
