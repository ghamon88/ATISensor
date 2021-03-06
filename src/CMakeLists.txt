
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libATISensor-<target>.so
# and installs in the directory lib/orocos/ATISensor/
#
include_directories(../tinyxml2)
include_directories(../happyhttp)
include_directories(/usr/xenomai/include)
include_directories(../../../directKinematics/include/directKinematics)
include_directories(/usr/local/rtnet/include)
include_directories(/home/kuka/src/groovy_workspace/eigen_lgsm/include/eigen3)

link_directories(/usr/xenomai/lib)

add_library(tinyxml2 ../tinyxml2/tinyxml2.cpp)
add_library(happyhttp ../happyhttp/happyhttp.cpp)

orocos_component(ATISensor ATISensor-component.hpp ATISensor-component.cpp) # ...you may add multiple source files

target_link_libraries(ATISensor tinyxml2 happyhttp rtdm)



#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/ATISensor/ directory
#
orocos_install_headers(ATISensor-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libATISensor-plugin-<target>.so
# and installs in the directory lib/orocos/ATISensor/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(ATISensor-plugin ATISensor-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libATISensor-service-<target>.so
# and installs in the directory lib/orocos/ATISensor/plugins/
#
orocos_service(ATISensor-service ATISensor-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


