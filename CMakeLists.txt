cmake_minimum_required(VERSION 3.8)
project(gazebo_classic_sfm_plugin_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

add_library(PedestrianSFMRandomPlugin SHARED src/PedestrianSFMRandomPlugin.cpp)
target_include_directories(PedestrianSFMRandomPlugin PUBLIC include)
target_link_libraries(PedestrianSFMRandomPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

install(TARGETS PedestrianSFMRandomPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libAttachModelPlugin.so library.
set(attach_model_plugin_name AttachModelPlugin)
add_library(${attach_model_plugin_name} SHARED
  src/AttachModelPlugin.cc
)
target_link_libraries(${attach_model_plugin_name}
  ${GAZEBO_LIBRARIES}
)
#add_dependencies(${attach_model_plugin_name}
#  ${PROJECT_NAME}_generate_messages_cpp
#)
install(TARGETS ${attach_model_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
