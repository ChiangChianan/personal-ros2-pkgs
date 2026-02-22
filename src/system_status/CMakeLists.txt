cmake_minimum_required(VERSION 3.8)
project(system_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(status_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(system_status_pub src/system_status_pub.cpp)

# 启用自动 moc
set(CMAKE_AUTOMOC ON)
add_executable(system_status_display 
  src/status_display_main.cpp 
  src/main_window.cpp
  include/system_status/main_window.hpp   # 添加头文件以触发 moc
)


target_include_directories(system_status_display PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(system_status_display Qt5::Widgets)

ament_target_dependencies(system_status_pub 
  rclcpp
  status_interfaces
)

ament_target_dependencies(system_status_display 
  rclcpp
  status_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS system_status_pub system_status_display DESTINATION lib/${PROJECT_NAME})

ament_package()
