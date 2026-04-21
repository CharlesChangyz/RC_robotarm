cmake_minimum_required(VERSION 3.8)
project(dmbot_serial)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arm_msgs REQUIRED)

## 查找 libusb（使用 pkg-config）
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)



## 包含头文件路径
include_directories(
  include
  ${LIBUSB_INCLUDE_DIRS}
)


## 编译 motor 库
add_library(motor 
  src/protocol/damiao.cpp
)
set_target_properties(motor PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(dmbot_serial_driver SHARED
  src/dm_motor_driver.cpp
)

## motor 库链接其他依赖库
target_link_libraries(motor
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libu2canfd.a
  ${LIBUSB_LIBRARIES}
  Threads::Threads
)

target_link_libraries(dmbot_serial_driver
  motor
)
ament_target_dependencies(dmbot_serial_driver
  rclcpp
)



## 编译主程序
add_executable(test_motor
  src/test.cpp
)

target_link_libraries(test_motor
  motor
)

add_executable(dev_sn
    src/dev_sn.cpp
)
# 链接库到可执行文件
target_link_libraries(dev_sn PRIVATE
    ${LIBUSB_LIBRARIES}
)

add_executable(usb2canfd_dm_node_cpp
  src/usb2canfd_dm_node.cpp
)
target_link_libraries(usb2canfd_dm_node_cpp
  motor
)
ament_target_dependencies(usb2canfd_dm_node_cpp
  rclcpp
  sensor_msgs
  arm_msgs
)

add_executable(debugger_node_cpp
  src/debugger_node.cpp
)
ament_target_dependencies(debugger_node_cpp
  rclcpp
  arm_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS motor dmbot_serial_driver test_motor dev_sn usb2canfd_dm_node_cpp debugger_node_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(motor dmbot_serial_driver)

ament_package()
