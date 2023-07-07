cmake_minimum_required(VERSION 3.5)
project(trylo_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/n_control.py
  scripts/n_refgen.py
  scripts/n_tst_ref.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Reference.msg"
)

ament_package()
