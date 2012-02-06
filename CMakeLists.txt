cmake_minimum_required(VERSION 2.6)

project(RenderLib)


set( CORE_LIB "CoreLib" )
set( CORELIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CoreLib/include CACHE STRING "path to CoreLib include folder" )
set( CORELIB_LIB_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/../CoreLib/lib CACHE STRING "path to CoreLib libray folder" )

set( RENDER_LIB "RenderLib" )
set( RENDERLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "path to RenderLib include folder" )
set( RENDERLIB_OUTPUT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/lib )


# Platform specific definitions
if(WIN32)
    if(MSVC)
        # Other MSVC compilation flags
        add_definitions(-D_SCL_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
		add_definitions(-D NOMINMAX) # prevent windef.h min/max macros from interfering with stl's
		#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
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

file(GLOB_RECURSE CORELIB_SOURCES 
	${CORELIB_INCLUDE_DIR}/*.cpp 
	../CoreLib/include/*.h 
	../CoreLib/include/*.inl)

file(GLOB_RECURSE RENDERLIB_SOURCES 
	source/*.cpp 
	include/*.h 
	include/*.inl)


# mimic disk folder structure on the project
#create_source_group( "" "${CMAKE_CURRENT_SOURCE_DIR}/source" "${CMAKE_CURRENT_SOURCE_DIR}/include" ${RENDERLIB_SOURCES} )

# add base to include directories

include_directories( ${RENDERLIB_INCLUDE_DIR} ${CORELIB_INCLUDE_DIR})

# Core Lib

add_library(${CORE_LIB} STATIC IMPORTED)
if(WIN32)
	set_property( TARGET ${CORE_LIB} PROPERTY IMPORTED_LOCATION ${CORELIB_LIB_FOLDER}/${CORE_LIB}.lib )
else(WIN32)
	set_property( TARGET ${CORE_LIB} PROPERTY IMPORTED_LOCATION ${CORELIB_LIB_FOLDER}/${CORE_LIB}.a )
endif(WIN32)

# Render Lib

add_library(${RENDER_LIB} STATIC ${RENDERLIB_SOURCES})
target_link_libraries( ${RENDER_LIB} ${CORE_LIB})

set_target_properties( ${RENDER_LIB} PROPERTIES PREFIX "" )
set_target_properties( ${RENDER_LIB} PROPERTIES OUTPUT_NAME ${RENDER_LIB} )
set_target_properties( ${RENDER_LIB} PROPERTIES LINKER_LANGUAGE CXX )
set_target_properties( ${RENDER_LIB} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${RENDERLIB_OUTPUT_FOLDER} )