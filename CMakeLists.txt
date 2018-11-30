#  minimum required cmake version: 3.8.0
cmake_minimum_required(VERSION 3.8.0)

project(er_vision_pipeline)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# Removing some warnings from char * assignments and unused stuff from libigl
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -Wno-unused-result -Wno-deprecated-declarations")

# I know!! I have an int that is a texture pointer!!! fuck me, right?
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-to-pointer-cast")

set(CMAKE_BUILD_TYPE Release)
add_definitions( -DUSE_IMGUI )

find_package( PCL REQUIRED )

if (PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

set(DEPENDENCIES realsense2 ${PCL_LIBRARIES})

find_package(OpenGL)
if(NOT OPENGL_FOUND)
    message(FATAL_ERROR "\n\n OpenGL package is missing!\n\n")
endif()

list(APPEND DEPENDENCIES ${OPENGL_LIBRARIES})

if(WIN32)
    list(APPEND DEPENDENCIES glfw3)
else()
    # Find glfw header
    find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
        PATHS /usr/X11R6/include
              /usr/include/X11
              /opt/graphics/OpenGL/include
              /opt/graphics/OpenGL/contrib/libglfw
              /usr/local/include
              /usr/include/GL
              /usr/include
    )
    # Find glfw library
    find_library(GLFW_LIBRARIES NAMES glfw glfw3
            PATHS /usr/lib64
                  /usr/lib
                  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                  /usr/local/lib64
                  /usr/local/lib
                  /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                  /usr/X11R6/lib
    )
    list(APPEND DEPENDENCIES m ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES})
    include_directories(${GLFW_INCLUDE_DIR})
endif()

include_directories("libs/libigl/include/")
include_directories("libs/libigl/external/")
include_directories("libs/libigl/external/glad/include/")
include_directories("libs/libigl/external/imgui")
include_directories("libs/libigl/external/imgui/examples/")
include_directories("libs/libigl/external/libigl-imgui/")
include_directories("libs/libigl/external/glfw/include/")
include_directories("libs/libigl/external/imgui/examples/libs/gl3w/")


add_subdirectory(src)
