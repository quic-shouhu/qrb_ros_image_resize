cmake_minimum_required(VERSION 3.8)
project(qrb_ros_image_resize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# check whether the eva lib existed
find_library(BUILD_EVA_ENABLE
  NAMES eva
  PATHS /usr/lib
  NO_DEFAULT_PATH
)

# add targets
# Add targets resize library. if HW support EVA that EVA as default.
# BUILD_EVA_DISABLE_FORCE != ON and BUILD_EVA_ENABLE is not empty then choice EVA.
if(NOT BUILD_EVA_DISABLE_FORCE STREQUAL "ON" AND BUILD_EVA_ENABLE)
  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/resize_accelerated.cpp
  )
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE EVA_ENABLE=True
  )
else()
  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/resize.cpp
  )
endif()

rclcpp_components_register_nodes(${PROJECT_NAME}
  "qrb_ros::resize::ResizeNode"
)

ament_auto_package(INSTALL_TO_SHARE launch)