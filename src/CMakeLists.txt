
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

find_package(RST-RT REQUIRED)
IF (RST-RT_FOUND)
    message(STATUS "RST-RT found. Version: ${RST-RT_VERSION}")
    include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
    add_definitions(${RST-RT_CFLAGS})
    link_directories(${RST-RT_LIBRARY_DIRS})
ENDIF()

add_subdirectory( orocos_kinematics_dynamics )
# find_package(orocos_kdl REQUIRED)
# if(orocos_kdl_FOUND)
#     message(STATUS "orocos_kdl found. Version: ${orocos_kdl_VERSION}")
#     include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
#     add_definitions(${orocos_kdl_CFLAGS})
#     link_directories(${orocos_kdl_LIBRARY_DIRS})
# endif()

find_package(urdfdom REQUIRED)
if(urdfdom_FOUND)
    message(STATUS "urdfdom found. Version: ${urdfdom_VERSION}")
    include_directories(BEFORE SYSTEM ${urdfdom_INCLUDE_DIRS})
    add_definitions(${urdfdom_CFLAGS})
    link_directories(${urdfdom_LIBRARY_DIRS})
endif()

find_package(kdl_parser REQUIRED)
if(kdl_parser_FOUND)
  message(STATUS "kdl_parser found. Version: ${kdl_parser_VERSION}")
  include_directories(BEFORE SYSTEM ${kdl_parser_INCLUDE_DIRS})
  add_definitions(${kdl_parser_CFLAGS})
  link_directories(${kdl_parser_LIBRARY_DIRS})
endif()

find_package(urdf REQUIRED)
if(urdf_FOUND)
    message(STATUS "urdf found. Version: ${urdf_VERSION}")
    include_directories(BEFORE SYSTEM ${urdf_INCLUDE_DIRS})
    add_definitions(${urdf_CFLAGS})
    link_directories(${urdf_LIBRARY_DIRS})
endif()

# Creates a component library libhelloworld-<target>.so
# and installs in the directory lib/orocos/helloworld/
#
orocos_component(robot_arm_svd robot_arm_svd-component.hpp robot_arm_svd-component.cpp) # ...you may add multiple source files
# orocos_component(robot_arm_svd robot_arm_svd-component.hpp robot_arm_svd-component.cpp ManipulabilityVisulizer.cpp  ManipulabilityVisulizer.hpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.
target_link_libraries(${CMAKE_PROJECT_NAME}
    ${RST-RT_LIBRARIES}
    # ${orocos_kdl_LIBRARIES}
    orocos-kdl
    ${urdfdom_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${urdf_LIBRARIES})

#
# Additional headers:
#
# Installs in the include/orocos/helloworld/ directory
#
orocos_install_headers(robot_arm_svd-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

