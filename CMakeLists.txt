cmake_minimum_required(VERSION 2.8.3)
project(stopping_trajectory)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      control_arch
      geometry_utils
      local_map_generator
      parameter_utils
      collision_checker
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
            roscpp
            std_msgs
            control_arch
            geometry_utils
            local_map_generator
            parameter_utils
            PCL
            collision_checker
)

include_directories(include
      ${catkin_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
)

link_directories(
      ${catkin_LIBRARY_DIRS}
      ${PCL_INCLUDE_DIRS}
)

add_definitions(
      ${catkin_DEFINITIONS}
)

add_library(${PROJECT_NAME}
      src/StoppingTrajectory.cpp
      src/ComputeTrajectory.cpp
      src/EscapePoints.cpp
      src/Visualization.cpp
      src/SamplePoints.cpp
      src/CommandStop.cpp
      src/Initialization.cpp
)

target_link_libraries(${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
      src/stopping_trajectory.cpp
)

set_target_properties(${PROJECT_NAME}_node
    PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
      ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
)
