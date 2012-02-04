cmake_minimum_required(VERSION 2.6)

project(RenderLib)

# Platform specific definitions
if(WIN32)
    if(MSVC)
        # Other MSVC compilation flags
        add_definitions(-D_SCL_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
		add_definitions(-D NOMINMAX) # prevent windef.h min/max macros from interfering with stl's
		set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
    endif(MSVC)
endif (WIN32)

#specify app sources
function(create_source_group sourceGroupName relativeSourcePath)
	foreach(currentSourceFile ${ARGN})
		file(RELATIVE_PATH folder ${relativeSourcePath} ${currentSourceFile})
		get_filename_component(filename ${folder} NAME)
		string(REPLACE ${filename} "" folder ${folder})
		if(NOT folder STREQUAL "")
			string(REGEX REPLACE "/+$" "" folderlast ${folder})
			string(REPLACE "/" "\\" folderlast ${folderlast})
			source_group("${sourceGroupName}\\${folderlast}" FILES ${currentSourceFile})
		endif(NOT folder STREQUAL "")
	endforeach(currentSourceFile ${ARGN})
endfunction(create_source_group)

file(GLOB_RECURSE RENDERLIB_SOURCES source/dataStructs/*.cpp source/geometry/*.cpp include/*.h include/*.inl)
file(GLOB_RECURSE CORELIB_SOURCES source/CoreLib/*.cpp )

# mimic disk folder structure on the project
create_source_group( "" "${CMAKE_CURRENT_SOURCE_DIR}/source" "${CMAKE_CURRENT_SOURCE_DIR}/include" ${RENDERLIB_SOURCES} )

# add base to include directories

set( CORELIB_NAME "CoreLib" )
set( CORELIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source/CoreLib/include CACHE STRING "path to CoreLib include folder" )
set( CORELIB_OUTPUT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/source/CoreLib/lib CACHE STRING "path to CoreLib libray folder" )

set( RENDERLIB_NAME "RenderLib" )
set( RENDERLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "path to RenderLib include folder" )
set( RENDERLIB_OUTPUT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/lib )

include_directories( ${RENDERLIB_INCLUDE_DIR} ${CORELIB_INCLUDE_DIR})
#link_directories ( ${PUBLIC_LIBRARY_DIRS} ${CORELIB_OUTPUT_FOLDER})

# Core Lib
add_library(${CORELIB_NAME} STATIC ${CORELIB_SOURCES})
set_target_properties( ${CORELIB_NAME} PROPERTIES PREFIX "" )
set_target_properties( ${CORELIB_NAME} PROPERTIES OUTPUT_NAME ${CORELIB_NAME} )
set_target_properties( ${CORELIB_NAME} PROPERTIES LINKER_LANGUAGE C )
set_target_properties( ${CORELIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CORELIB_OUTPUT_FOLDER} )

# Render Lib

add_library(${RENDERLIB_NAME} STATIC ${RENDERLIB_SOURCES})
target_link_libraries( ${RENDERLIB_NAME} ${CORELIB_OUTPUT_FOLDER}/${CORELIB_NAME})

set_target_properties( ${RENDERLIB_NAME} PROPERTIES PREFIX "" )
set_target_properties( ${RENDERLIB_NAME} PROPERTIES OUTPUT_NAME ${RENDERLIB_NAME} )
set_target_properties( ${RENDERLIB_NAME} PROPERTIES LINKER_LANGUAGE C )
set_target_properties( ${RENDERLIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${RENDERLIB_OUTPUT_FOLDER} )

