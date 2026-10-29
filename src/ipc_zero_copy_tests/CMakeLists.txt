cmake_minimum_required(VERSION 3.8)
project(ipc_zero_copy_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPS
  rclcpp
  rmw
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# ----------------------------------------------------------------------------
# Supported message types. Adding one is a single line here; its registration
# translation unit is generated from src/register_type.cpp.in. Types with a
# variable-length field also need fill_payload()/payload_buffer() overloads in
# include/ipc_zero_copy_tests/payload_traits.hpp, otherwise payload_bytes is
# ignored and the message object's address is used for the zero-copy check.
# ----------------------------------------------------------------------------
set(SUPPORTED_TYPES
  sensor_msgs/PointCloud2
  sensor_msgs/Image
  sensor_msgs/Imu
  sensor_msgs/LaserScan
  sensor_msgs/JointState
  sensor_msgs/PointCloud
  sensor_msgs/Range
  sensor_msgs/Temperature
  sensor_msgs/FluidPressure
  nav_msgs/Odometry
  nav_msgs/Path
  geometry_msgs/PointStamped
  geometry_msgs/PoseStamped
  geometry_msgs/TwistStamped
  geometry_msgs/WrenchStamped
  geometry_msgs/TransformStamped
  std_msgs/Header
  std_msgs/String
  std_msgs/Bool
  std_msgs/Byte
  std_msgs/Char
  std_msgs/ColorRGBA
  std_msgs/Empty
  std_msgs/Float32
  std_msgs/Float64
  std_msgs/Int8
  std_msgs/Int16
  std_msgs/Int32
  std_msgs/Int64
  std_msgs/UInt8
  std_msgs/UInt16
  std_msgs/UInt32
  std_msgs/UInt64
  std_msgs/MultiArrayDimension
  std_msgs/MultiArrayLayout
  std_msgs/ByteMultiArray
  std_msgs/Float32MultiArray
  std_msgs/Float64MultiArray
  std_msgs/Int8MultiArray
  std_msgs/Int16MultiArray
  std_msgs/Int32MultiArray
  std_msgs/Int64MultiArray
  std_msgs/UInt8MultiArray
  std_msgs/UInt16MultiArray
  std_msgs/UInt32MultiArray
  std_msgs/UInt64MultiArray
)

set(TYPE_SOURCES)
foreach(IPC_TYPE ${SUPPORTED_TYPES})
  string(REPLACE "/" ";" parts "${IPC_TYPE}")
  list(GET parts 0 pkg)
  list(GET parts 1 name)
  # CamelCase -> snake_case, same rule as rosidl (UInt8MultiArray -> u_int8_multi_array).
  string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" snake "${name}")
  string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" snake "${snake}")
  string(TOLOWER "${snake}" snake)
  set(IPC_HEADER "${pkg}/msg/${snake}.hpp")
  set(IPC_CPP_TYPE "${pkg}::msg::${name}")
  set(out "${CMAKE_CURRENT_BINARY_DIR}/types/${pkg}_${snake}.cpp")
  configure_file(src/register_type.cpp.in "${out}" @ONLY)
  list(APPEND TYPE_SOURCES "${out}")
endforeach()

# ----------------------------------------------------------------------------
# Case runner. operator new is replaced in alloc_counter.cpp, so this must be
# an executable (not a component) for the override to see every allocation.
# ----------------------------------------------------------------------------
add_executable(ipc_case_runner
  src/main.cpp
  src/alloc_counter.cpp
  src/cases.cpp
  src/case_runner.cpp
  ${TYPE_SOURCES}
)
target_include_directories(ipc_case_runner PRIVATE include)
target_link_libraries(ipc_case_runner
  rclcpp::rclcpp
  rmw::rmw
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(ipc_external_subscriber src/external_subscriber.cpp)
target_link_libraries(ipc_external_subscriber rclcpp::rclcpp)

install(TARGETS ipc_case_runner ipc_external_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
