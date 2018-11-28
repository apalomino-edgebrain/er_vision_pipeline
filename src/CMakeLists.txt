#  minimum required cmake version: 3.8.0
cmake_minimum_required(VERSION 3.8.0)

project(er_vision_pipeline)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(RAW_ER_CPP application_state.cpp  er-pcl.cpp  er-pipeline.cpp  field_map.cpp  geo_tools.cpp  helpers_ui.cpp  process_3d.cpp  visualizer.cpp  visualizer_ui.cpp
	algebra/plane.cpp util/er-logging.cpp 
	filters/ground_filter.cpp  filters/plant_extraction.cpp  filters/plant_segmentation.cpp  filters/plant_separation.cpp
	../libs/libigl/external/imgui/examples/imgui_impl_glfw.cpp ../libs/libigl/external/imgui/imgui.cpp ../libs/libigl/external/imgui/imgui_draw.cpp	
	../libs/libigl/external/libigl-imgui/imgui_impl_opengl3.cpp
	../libs/libigl/external/glad/src/glad.c			
	)  
		   
add_executable(er-vision er-pcl.cpp ${RAW_ER_CPP})
target_link_libraries(er-vision ${DEPENDENCIES})
set_target_properties (er-vision PROPERTIES
    FOLDER "Examples/ER"
)

foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MT")
        string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
    endif(${flag_var} MATCHES "/MT")
endforeach(flag_var)

install(
    TARGETS

    er-vision

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)
