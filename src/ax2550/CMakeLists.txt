cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# # ----------------------------------------------------------------------
# # join_arguments(var)
# #
# # Join the arguments in the (semi-colon separated) list VAR into one
# # space separated string. The string will be returned in the variable
# # VAR. This command is the opposite of the built-in command
# # separate_arguments().
# # ----------------------------------------------------------------------
# macro(join_arguments var)
#   set(_var)
#   foreach(_v ${${var}})
#     set(_var "${_var} ${_v}")
#   endforeach(_v ${${var}})
#   string(STRIP ${_var} _var)
#   set(${var} ${_var})
# endmacro(join_arguments)

# # Enable warnings
# # Assuming unix means a gcc style compiler, eg. g++ or clang++.
# IF(UNIX)
#   set(CMAKE_CXX_FLAGS "-Wall -Weffc++ -pedantic -pedantic-errors -Wextra "
#                       "-Wall -Waggregate-return -Wcast-align -Wcast-qual "
#                       "-Wchar-subscripts -Wcomment -Wconversion "
#                       "-Wdisabled-optimization -Wfloat-equal -Wformat "
#                       "-Wformat=2 -Wformat-nonliteral -Wformat-security "
#                       "-Wformat-y2k -Wimplicit -Wimport -Winit-self "
#                       "-Winline -Winvalid-pch -Wlong-long -Wmissing-braces "
#                       "-Wmissing-field-initializers "
#                       "-Wmissing-format-attribute -Wmissing-include-dirs "
#                       "-Wmissing-noreturn -Wpacked -Wparentheses "
#                       "-Wpointer-arith -Wredundant-decls -Wreturn-type "
#                       "-Wsequence-point -Wshadow -Wsign-compare "
#                       "-Wstack-protector -Wstrict-aliasing "
#                       "-Wstrict-aliasing=2 -Wswitch -Wswitch-default "
#                       "-Wswitch-enum -Wtrigraphs -Wuninitialized "
#                       "-Wunknown-pragmas -Wunreachable-code -Wunused "
#                       "-Wunused-function -Wunused-label -Wunused-parameter "
#                       "-Wunused-value -Wunused-variable -Wvariadic-macros "
#                       "-Wvolatile-register-var -Wwrite-strings")
#   join_arguments(CMAKE_CXX_FLAGS)
# ELSEIF(WIN32)
#   # Force to always compile with W4
#   if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
#     string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
#   else()
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
#   endif()
# endif(UNIX)



# Use clang if available
IF(EXISTS /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_OSX_DEPLOYMENT_TARGET "")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ferror-limit=5")
ENDIF(EXISTS /usr/bin/clang)

# set(CMAKE_CXX_COMPILER /usr/bin/g++)
# set(CMAKE_OSX_DEPLOYMENT_TARGET "")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0")


# Build the ax2550 library
rosbuild_add_library(${PROJECT_NAME} src/ax2550.cc)

# Build the ROS node
rosbuild_add_executable(ax2550_node src/ax2550_node.cc)
target_link_libraries(ax2550_node ${PROJECT_NAME})
