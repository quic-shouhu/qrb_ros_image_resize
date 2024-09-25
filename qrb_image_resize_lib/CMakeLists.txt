cmake_minimum_required(VERSION 3.8)
project(qrb_image_resize_lib VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# add targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/eva_utils_lib.cpp
)

set_target_properties(qrb_image_resize_lib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

target_link_libraries(${PROJECT_NAME}
  EGL
  GLESv2
)

ament_auto_package()
