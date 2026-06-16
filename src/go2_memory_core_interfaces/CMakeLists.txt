cmake_minimum_required(VERSION 3.8)
project(go2_memory_core_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WriteCheckpoint.srv"
  "srv/WritePlace.srv"
  "srv/WriteSpawn.srv"
  "srv/QueryMemory.srv"
  "srv/QueryGraph.srv"
  "srv/ConnectLocations.srv"
  "srv/PromoteMemory.srv"
  "srv/ForgetMemory.srv"
  "srv/GetResumeContext.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
