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
  LIBRARIES ${PROJECT_NAME} hex_controller
  CATKIN_DEPENDS rospy roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Stare pliki, które zostały przeniesione do głównego katalogu src
add_executable(stand_up_node src/stand_up.cpp)
add_executable(walk_two_leg_gait_node src/walk_two_leg_gait.cpp)

# Legacy biblioteka z pozostałymi starymi plikami
add_library(hex_controller
  src/legacy/gait_controller.cpp
  src/legacy/bipod_gait.cpp
  src/legacy/tripod_gait.cpp
  src/legacy/single_leg_gait.cpp
  src/legacy/gait_manager.cpp
)

# Linkowanie hex_controller
target_link_libraries(hex_controller
  ${catkin_LIBRARIES}
)

# Linkowanie starych node'ów
target_link_libraries(stand_up_node
  hex_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(walk_two_leg_gait_node
  hex_controller
  ${catkin_LIBRARIES}
)

# Legacy nodes pozostałe w katalogu legacy
set(LEGACY_NODES
  hexapod_controller_node
  hexapod_teleop_node
  two_leg_walk_gait_node
)

# Tworzenie i linkowanie pozostałych legacy nodes
foreach(node ${LEGACY_NODES})
  add_executable(${node} src/legacy/${node}.cpp)
  target_link_libraries(${node}
    hex_controller
    ${catkin_LIBRARIES}
  )
endforeach()

# Komentujemy nową implementację do czasu utworzenia plików
 add_library(${PROJECT_NAME}
   src/base_gait.cpp
   src/one_leg_gait.cpp
 )

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

 add_executable(tests_node src/tests_node.cpp)
 target_link_libraries(tests_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


# add_executable(tests_node src/tests_node.cpp)
#target_link_libraries(tests_node
#  hex_controller  # Na razie linkujemy ze starą biblioteką
#  ${catkin_LIBRARIES}
#)


# Install targets
install(TARGETS hex_controller stand_up_node walk_two_leg_gait_node ${LEGACY_NODES} tests_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)