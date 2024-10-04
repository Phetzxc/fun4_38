cmake_minimum_required(VERSION 3.8)
project(lab1_interfaces)

# Enable warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)          # Standard ROS messages
find_package(rosidl_default_generators REQUIRED)  # For service generation

# Uncomment and modify the following line if you have additional dependencies
# find_package(geometry_msgs REQUIRED)    # Add this if you're using geometry_msgs

# Generate interfaces for services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
  "srv/SetNoise.srv"
  DEPENDENCIES std_msgs  # Add additional dependencies here if necessary
)

# If you have tests, include linting and testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip linter checks for licenses (only for testing, comment out for production)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Export the package
ament_package()
