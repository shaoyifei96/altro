# Used 3.23 features
# - PROJECT_IS_TOP_LEVEL (3.21)
# - FILE_SETS (3.23)
cmake_minimum_required(VERSION 3.20)
project(altro)
include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# cmake_policy(SET CMP0077 NEW)
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c  # version 3.4.0
  )
FetchContent_MakeAvailable(eigen)
# Organization info
set(ORGANIZATION_NAME_FULL "Robotic Exploration Lab")
set(ORGANIZATION_NAME "rexlab")

# Include needed modules
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# include(CMakePrintHelpers)
# message(STATUS "GOOD-1")
# include(FetchContent)
# message(STATUS "GOOD0")
# include(AltroCMakeFunctions)
# include(FindThreads)
# include(GNUInstallDirs)

#############################################
# Options
#############################################

# Handle default build type
set(ALTRO_DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Setting CMAKE_BUILD_TYPE to ${ALTRO_DEFAULT_BUILD_TYPE}")
  set(CMAKE_BUILD_TYPE ${ALTRO_DEFAULT_BUILD_TYPE} CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Use -fPIC option
# option(BUILD_SHARED_LIBS "Build Shared" OFF)
# if (BUILD_SHARED_LIBS)
# #  set(CMAKE_POSITION_INDEPENDENT_CODE ON FORCE)
#   message(STATUS "Building shared library.")
#   add_compile_options(-fPIC)
# else()
#   message(STATUS "Building static library. Note that a shared library is recommended.")
# endif()

#############################################
# Dependencies
#############################################

find_package(catkin REQUIRED)


add_subdirectory(deps)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES altro
 CATKIN_DEPENDS 
 DEPENDS
)
# find_package (Eigen3 3.4 REQUIRED)


add_subdirectory(src)


# add_dependencies(altro
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
# )


add_executable(main main.cpp)
target_link_libraries(main PRIVATE altro fmt::fmt Eigen3::Eigen)
target_include_directories(main
  PUBLIC include)


