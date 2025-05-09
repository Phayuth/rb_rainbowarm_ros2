cmake_minimum_required(VERSION 3.5)
project(rb_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)


# ADD THIS FOR CUSTOM SRV AND MSG FILE GENERATOR ---------------------
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReqTcp.srv"
  "srv/ReqJnt.srv"
  "action/ReqTcpTraj.action"
  "action/ReqJntTraj.action"
  DEPENDENCIES geometry_msgs sensor_msgs trajectory_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
# --------------------------------------------------------------------

ament_package()
