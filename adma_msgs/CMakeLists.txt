cmake_minimum_required(VERSION 3.5)
project(adma_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_include_directories(include)

rosidl_generate_interfaces(adma_msgs
  "msg/AdmaDataDelta.msg"
  "msg/AdmaData.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
