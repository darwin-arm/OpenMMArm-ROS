cmake_minimum_required(VERSION 3.8)
project(openmmarm_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW QUIET glfw3)

# MuJoCo 仿真库
file(GLOB MUJOCO_HOME_DIRS "$ENV{HOME}/.mujoco/mujoco-*")
file(GLOB MUJOCO_CONDA_DIRS "$ENV{HOME}/anaconda3/lib/python*/site-packages/mujoco")
set(MUJOCO_ROOT_HINTS
  "$ENV{MUJOCO_DIR}"
  "$ENV{MUJOCO_HOME}"
  ${MUJOCO_HOME_DIRS}
  ${MUJOCO_CONDA_DIRS}
)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${MUJOCO_ROOT_HINTS}
  PATH_SUFFIXES include
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco libmujoco
  HINTS ${MUJOCO_ROOT_HINTS}
  PATH_SUFFIXES lib
)
if(NOT MUJOCO_LIBRARY)
  find_library(MUJOCO_LIBRARY
    NAMES mujoco libmujoco
    HINTS ${MUJOCO_ROOT_HINTS}
  )
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo not found. Please set MUJOCO_DIR or MUJOCO_HOME to MuJoCo root "
    "(must contain include/mujoco/mujoco.h and lib/libmujoco.so, or libmujoco.so in root)."
  )
endif()

# 包含目录
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
)

# 硬件接口库
add_library(${PROJECT_NAME} SHARED
  src/openmmarm_hw.cpp
  src/io_mujoco.cpp
  src/io_udp.cpp
  src/arm_dynamics.cpp
)

if(GLFW_FOUND)
  target_include_directories(${PROJECT_NAME} PRIVATE ${GLFW_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${GLFW_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PRIVATE OPENMMARM_HAS_GLFW=1)
  message(STATUS "GLFW found, MuJoCo viewer support enabled")
else()
  message(WARNING "GLFW not found, MuJoCo viewer support disabled")
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rclcpp_action
  control_msgs
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  ${MUJOCO_LIBRARY}
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(hardware_interface openmmarm_hw.xml)

# 安装
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rclcpp_action
  control_msgs
  ament_index_cpp
)

ament_package()
