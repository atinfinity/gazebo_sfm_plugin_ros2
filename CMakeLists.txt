cmake_minimum_required(VERSION 3.8)
project(gazebo_sfm_plugin_ros2)

# gz-sim8 / Harmonic requires C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Pedestrian Social Force Model plugin (gz-sim System)
add_library(PedestrianSFMRandomPlugin SHARED src/PedestrianSFMRandomPlugin.cpp)
target_include_directories(PedestrianSFMRandomPlugin PUBLIC include)
target_link_libraries(PedestrianSFMRandomPlugin
  gz-sim8::gz-sim8
  gz-math7::gz-math7
)

# Plugin that makes a collision model follow an actor (gz-sim System)
add_library(AttachModelPlugin SHARED src/AttachModelPlugin.cc)
target_include_directories(AttachModelPlugin PUBLIC include)
target_link_libraries(AttachModelPlugin
  gz-sim8::gz-sim8
  gz-math7::gz-math7
)

install(TARGETS
  PedestrianSFMRandomPlugin
  AttachModelPlugin
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

ament_package()
