/* Containes the build files programmatically generated for the Robot description and simulation below */

export function GeneratePackageXMLFile(title) {
    return `<?xml version="1.0"?>
<package format="3">
  <name>${title}_description</name>
  <version>0.0.0</version>
  <description>The ${title}_description package</description>

  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>robot_state_publisher</build_depend>
  <build_depend>rviz2</build_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
`;
};


export function GenerateCMakelistsFile(title) {
    // Generate the CMakeLists.txt file for the ROS package
    return `cmake_minimum_required(VERSION 3.5)
project(${title}_description)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/\${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/\${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/\${PROJECT_NAME}
)

ament_package()
`;
};

export function GenerateSimPackageXMLFile(title) {
  return `<?xml version="1.0"?>
<package format="3">
<name>${title}_description</name>
<version>0.0.0</version>
<description>The ${title}_description package</description>

<maintainer email="your_email@example.com">Your Name</maintainer>
<license>Apache-2.0</license>

<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>robot_state_publisher</build_depend>
<build_depend>rviz2</build_depend>
<build_depend>gazebo_ros</build_depend>

<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>gazebo_ros</exec_depend>

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>

<export>
  <build_type>ament_cmake</build_type>
</export>
</package>
`;
};


export function GenerateSimCMakelistsFile(title) {
  // Generate the CMakeLists.txt file for the ROS package
  return `cmake_minimum_required(VERSION 3.5)
project(${title}_description)

# Default to C11
if(NOT CMAKE_C_STANDARD)
set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY model
DESTINATION share/\${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/\${PROJECT_NAME}
)

ament_package()
`;
};