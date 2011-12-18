#-D CMAKE_BUILD_TYPE=Debug
# go to the build directory, cmake from there, specifying the relative path to where CMakeLists.txt is (cmake ..)
cmake_minimum_required(VERSION 2.6)
project(RenderLib)
set( RENDERLIB_NAME "RenderLib" )
set( RENDERLIB_OUTPUT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE} )

# --------- Setup the Library output Directory -------------
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${RENDERLIB_OUTPUT_FOLDER} )

#specify app sources
file(GLOB_RECURSE app_sources source/*.c source/*.cpp source/*.h source/*.inl source/*.hpp source/*.glsl source/*.ui)

# add base to include directories
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/source)

#sourceGroups( app_sources )
add_library(${RENDERLIB_NAME} STATIC ${app_sources})

SET_TARGET_PROPERTIES( ${RENDERLIB_NAME} PROPERTIES PREFIX "" )
SET_TARGET_PROPERTIES( ${RENDERLIB_NAME} PROPERTIES OUTPUT_NAME ${RENDERLIB_NAME} )
