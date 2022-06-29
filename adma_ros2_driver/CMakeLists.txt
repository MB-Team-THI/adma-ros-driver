cmake_minimum_required(VERSION 3.5)
project(adma_ros2_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options() #(-Wall -Wextra) #-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adma_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time)

include_directories(
  include
  ${adma_msgs_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
)

add_library(adma_connect_core
  src/adma_parse.cpp
)

ament_target_dependencies(adma_connect_core
  adma_msgs
  std_msgs
  sensor_msgs
)



target_link_libraries(adma_connect_core
  ${Boost_LIBRARIES}
  ${adma_msgs_LIBRARIES}
)

target_include_directories(adma_connect_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


add_executable(${PROJECT_NAME} src/adma_driver.cpp)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  adma_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  adma_connect_core
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  adma_connect_core
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

