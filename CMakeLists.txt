cmake_minimum_required(VERSION 2.8.3)
project(hex_final_urdf_description)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hex_controller ${PROJECT_NAME}
  CATKIN_DEPENDS rospy roscpp std_msgs
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

# Stare biblioteki i executable - zachowane ale zakomentowane
# add_library(hex_controller
#   src/gait_controller.cpp
#   src/bipod_gait.cpp
#   src/tripod_gait.cpp
#   src/single_leg_gait.cpp
#   src/gait_manager.cpp
# )

# target_link_libraries(hex_controller
#   ${catkin_LIBRARIES}
# )

# add_library(hexapod_teleop
#   src/hexapod_teleop_node.cpp
# )

# target_link_libraries(hexapod_teleop
#   ${catkin_LIBRARIES}
#   hex_controller
# )

# Nowa implementacja - biblioteka base_gait
add_library(${PROJECT_NAME}
  src/base_gait.cpp
  src/one_leg_gait.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Stare executable - zakomentowane
# add_executable(stand_up_node src/stand_up.cpp)
# add_executable(walk_two_leg_gait_node src/walk_two_leg_gait.cpp)
# add_executable(two_leg_walk_gait_node src/two_leg_walk_gait_node.cpp)
# add_executable(hexapod_controller_node src/hexapod_controller_node.cpp)
# add_executable(hexapod_teleop_node src/hexapod_teleop_node.cpp)

# Nowy executable
add_executable(tests_node src/tests_node.cpp)
add_dependencies(tests_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(tests_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Stare target_link_libraries - zakomentowane
# target_link_libraries(hexapod_teleop_node
#   ${catkin_LIBRARIES}
#   hexapod_teleop
# )

# target_link_libraries(hexapod_controller_node
#   ${catkin_LIBRARIES}
#   hex_controller
# )

# target_link_libraries(stand_up_node 
#   ${catkin_LIBRARIES}
#   hex_controller
# )

# target_link_libraries(walk_two_leg_gait_node 
#   ${catkin_LIBRARIES}
#   hex_controller
# )

# target_link_libraries(two_leg_walk_gait_node 
#   ${catkin_LIBRARIES}
#   hex_controller
# )

install(TARGETS ${PROJECT_NAME} tests_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)