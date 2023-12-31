cmake_minimum_required(VERSION 3.8)
project(cpp_democeres)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(helloworld src/helloworld.cpp)
add_executable(demo src/ceres_solver.cpp)

target_link_libraries(helloworld ${CERES_LIBRARIES})
target_link_libraries(demo ${CERES_LIBRARIES})

ament_target_dependencies(helloworld rclcpp Ceres)
ament_target_dependencies(demo rclcpp Ceres std_msgs)

install(TARGETS 
	helloworld
	demo
	DESTINATION lib/${PROJECT_NAME})

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
