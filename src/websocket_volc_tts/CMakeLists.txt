cmake_minimum_required(VERSION 3.8)
project(websocket_volc_tts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(text_provider_node src/text_provider_node.cpp)
add_executable(websocket_client_node src/websocket_client_node.cpp)

target_include_directories(text_provider_node PRIVATE  include)
target_include_directories(websocket_client_node PRIVATE  include)

ament_target_dependencies(text_provider_node
  rclcpp
  std_msgs
  nlohmann_json
)

ament_target_dependencies(websocket_client_node
  rclcpp
  std_msgs
  nlohmann_json
)

target_link_libraries(websocket_client_node
  OpenSSL::SSL
  OpenSSL::Crypto
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


install(TARGETS
  websocket_client_node
  text_provider_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
