cmake_minimum_required(VERSION 3.5)
project(map_with_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ds4_driver_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  ds4_driver_msgs
)

include_directories(include)

add_executable(controller
  src/controller_cpp/ControllerNode.cpp
  src/controller_main.cpp
)
ament_target_dependencies(controller ${dependencies})

install(TARGETS
  controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


