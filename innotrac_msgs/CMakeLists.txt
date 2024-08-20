cmake_minimum_required(VERSION 3.8)
project(innotrac_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify the custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LifterStatus.msg"
  "msg/NSCmd1.msg"
  "msg/NSCmd2.msg"
  "msg/NSCmd3.msg"
  "msg/SCmd1.msg"
  "msg/NSError1.msg"
  "msg/NSError2.msg"
  "msg/NSStatus1.msg"
  "msg/NSStatus2.msg"
  "msg/NSStatus3.msg"
  "msg/NSStatus4.msg"
  "msg/NSStatus5.msg"
  "msg/SError1.msg"
  "msg/SStatus1.msg"
  DEPENDENCIES std_msgs  # dependencies of custom message depends on other packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
