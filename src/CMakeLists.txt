
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libhelloworld-<target>.so
# and installs in the directory lib/orocos/helloworld/
#
orocos_component(robot_arm_svd robot_arm_svd-component.hpp robot_arm_svd-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/helloworld/ directory
#
orocos_install_headers(robot_arm_svd-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

