cmake_minimum_required(VERSION 3.8)
project(turtlesim_catch_them_all)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  spawner 
  src/spawner.cpp
)

target_include_directories(
  spawner PUBLIC include
)

add_executable(
  controller 
  src/turtle_controller.cpp
)
target_include_directories(
  controller PUBLIC include
)

ament_target_dependencies(
  controller
  turtlesim
  rclcpp
  custom_interfaces
  geometry_msgs

)

ament_target_dependencies(
  spawner
  turtlesim
  rclcpp
  custom_interfaces

)

install( TARGETS 
  controller
  spawner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
