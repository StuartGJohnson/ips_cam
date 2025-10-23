cmake_minimum_required(VERSION 3.5)
project(ips_cam)

unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if($ENV{ROS_VERSION} EQUAL 2)
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  endif()
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Bullet REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Snapshot.srv"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(include)

ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

if(EXISTS ${avcodec})
  message(STATUS "Found libavcodec: ${avcodec}")
endif()

if(EXISTS ${avutil})
  message(STATUS "Found libavutil: ${avutil}")
endif()

if(EXISTS ${swscale})
  message(STATUS "Found libswscale: ${swscale}")
endif()

## Build the USB camera library
## Do not use ament_auto here so as to not link to rclcpp
add_library(${PROJECT_NAME}_lib SHARED
  src/usb_cam.cpp
  src/image_processor.cpp
  src/ros2/cam_node.cpp
  src/ros2/ips_cam_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>              # becomes install/<pkg>/include with ament_auto
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  tf2
  tf2_geometry_msgs
  yaml-cpp
  ament_index_cpp
  camera_info_manager
  std_msgs
  image_transport
  std_srvs
)

add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}__rosidl_generator_cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(cpp_typesupport_target)
  target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})
endif()


target_link_libraries(${PROJECT_NAME}_lib
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ament_index_cpp::ament_index_cpp)

ament_export_libraries(${PROJECT_NAME})
    
add_executable(cam_node
  src/ros2/cam_node_exe.cpp)

target_link_libraries(cam_node
  ${PROJECT_NAME}_lib
)

add_executable(ips_cam_node
  src/ros2/ips_cam_node_exe.cpp)

target_link_libraries(ips_cam_node
  ${PROJECT_NAME}_lib
)

if(SANITIZE)
    target_compile_options(${PROJECT_NAME} PUBLIC -fsanitize=address -fsanitize=leak)
    target_link_libraries(${PROJECT_NAME} -fsanitize=address -fsanitize=leak)
    target_compile_options(${PROJECT_NAME}_node PUBLIC -fsanitize=address -fsanitize=leak)
    target_link_libraries(${PROJECT_NAME}_node -fsanitize=address -fsanitize=leak)
    target_link_libraries(${PROJECT_NAME}_node_exe -fsanitize=address -fsanitize=leak)
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    set(AMENT_LINT_AUTO_FILE_EXCLUDE "build/*" "build/*/*" "build/*/*/*" "build/*/*/*/*" "build/*/*/*/*/*")

    # linting and friends really does not seem to work very well. Mostly due to the fact
    # that linting (etc) check in artifact directories. Sigh. I did address this for cpplint via the above
    # hacky setting of AMENT_LINT_AUTO_FILE_EXCLUDE. It still doesn't work for lint_cmake, so I have
    # turned that off here with this:
    set(ament_cmake_lint_cmake_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)

    # Unit tests
    ament_add_gtest(test_usb_cam_utils
      test/test_usb_cam_utils.cpp)
    target_link_libraries(test_usb_cam_utils
      ${PROJECT_NAME}_lib)
    ament_add_gtest(test_pixel_formats
      test/test_pixel_formats.cpp)
    target_link_libraries(test_pixel_formats
      ${PROJECT_NAME}_lib)
    # TODO(flynneva): rewrite this test in another PR
    # Integration tests
    #ament_add_gtest(test_usb_cam_lib
    #  test/test_usb_cam_lib.cpp)
    #target_link_libraries(test_usb_cam_lib
    #   ${PROJECT_NAME})
    ament_add_gtest(test_infra test/test_infra.cpp)
    target_link_libraries(test_infra ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES} ament_index_cpp::ament_index_cpp)
    ament_add_gtest(test_image_processing test/test_image_processing.cpp)
    target_link_libraries(test_image_processing ${PROJECT_NAME}_lib ${OpenCV_LIBRARIES})
    ament_add_gtest(test_node test/test_node.cpp)
    target_link_libraries(test_node ${PROJECT_NAME}_lib yaml-cpp)
endif()

install(
    PROGRAMS scripts/show_image.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY test/data/
    DESTINATION share/${PROJECT_NAME}/test/data)

install(TARGETS
    ${PROJECT_NAME}_lib
    cam_node
    ips_cam_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME} )

if(DEFINED ENV{ROS_DISTRO} AND "$ENV{ROS_DISTRO}" STREQUAL "humble")
  # Humble needs the opt-in
  ament_auto_package(
    INSTALL_TO_SHARE
     launch
     config
    USE_SCOPED_HEADER_INSTALL_DIR)
else()
  # Kilted/Rolling: default is scoped
  ament_auto_package(
    INSTALL_TO_SHARE
     launch
     config) 
endif()



#ament_package()

