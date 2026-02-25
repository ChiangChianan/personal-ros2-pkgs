cmake_minimum_required(VERSION 3.8)
project(face_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local")
find_package(dlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RegisterFace.srv"
  "srv/RecognizeFace.srv"
  DEPENDENCIES sensor_msgs
)



add_executable(face_recognition_server src/face_recognition_server.cpp)
ament_target_dependencies(face_recognition_server
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(face_recognition_server PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(face_recognition_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp   # 关键：链接生成的接口库
  ${OpenCV_LIBRARIES}
  dlib::dlib
)


add_executable(face_recognition_client src/face_recognition_client.cpp)
ament_target_dependencies(face_recognition_client
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(face_recognition_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp   # 同样需要链接
  ${OpenCV_LIBRARIES}
)

add_executable(register_client src/register_client.cpp)
target_link_libraries(register_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(register_client rclcpp sensor_msgs cv_bridge)

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
  face_recognition_server
  face_recognition_client
  register_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
