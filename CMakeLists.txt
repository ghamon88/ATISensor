#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)
#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#


project(ATISensor)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()


# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories (/home/kuka/src/groovy_workspace/eigen_lgsm/include/eigen3)


# Rest of the configuration is in src/
add_subdirectory( src )



#LINK_DIRECTORIES(/home/kuka/src/groovy_workspace/orocos/ATISensor/lib)

#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libATISensor-types-<target>.so
# and installs in the directory lib/orocos/@target@/ATISensor/types/
#
# The header will go in include/orocos/ATISensor/types/ATISensor/ATISensor-types.hpp
# So you can #include <ATISensor/ATISensor-types.hpp>
#
orocos_typegen_headers(include/ATISensor/ATISensor-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
#orocos_generate_package()
