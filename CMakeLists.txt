cmake_minimum_required(VERSION 2.8)

if( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR AND NOT MSVC_IDE )
  message(FATAL_ERROR "In-source builds are not allowed.
Please create a directory and run cmake from there, passing the path to this source directory as the last argument.
This process created the file `CMakeCache.txt' and the directory `CMakeFiles'. Please delete them.")
endif()

project(streetname-fixer)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CheckCXXCompilerFlag)
include(FindPackageHandleStandardArgs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(bitness 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)
  message(STATUS "Building on a 64 bit system")
else()
  message(WARNING "Building on a 32 bit system is unsupported")
endif()

if (WIN32 AND MSVC_VERSION LESS 1800)
  message(FATAL_ERROR "Building with Microsoft compiler needs Visual Studio 2013 or later (Express version works too)")
endif()

set(BOOST_COMPONENTS date_time filesystem iostreams program_options regex system thread unit_test_framework)


set(FixerSources streetname_fixer.cpp)
add_executable(streetname-fixer ${FixerSources})

# Check the release mode
if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Configuring fixer in debug mode")
  if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message(STATUS "adding profiling flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -fno-inline")
    set(CMAKE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -fno-inline")
  endif()
endif()
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Configuring fixer in release mode")
  # Check if LTO is available
  set(LTO_FLAGS "")
  CHECK_CXX_COMPILER_FLAG("-flto" HAS_LTO_FLAG)
  if (HAS_LTO_FLAG)
    set(LTO_FLAGS "${LTO_FLAGS} -flto")

    # Since gcc 4.9 the LTO format is non-standart ('slim'), so we need to use the build-in tools
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND
        NOT "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS "4.9.0")
      message(STATUS "Using gcc specific binutils for LTO.")
      set(CMAKE_AR     "/usr/bin/gcc-ar")
      set(CMAKE_RANLIB "/usr/bin/gcc-ranlib")
    endif()
  endif (HAS_LTO_FLAG)
endif()

if (NOT WIN32)
add_definitions(-DBOOST_TEST_DYN_LINK)
endif()

# Configuring compilers
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # using Clang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wunreachable-code -pedantic -fPIC")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using GCC
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -fPIC")
  if (WIN32) # using mingw
    add_definitions(-DM_PI=3.141592653589793238462643383) # define M_PI
    add_definitions(-DWIN32)
    SET(OPTIONAL_SOCKET_LIBS ws2_32 wsock32)
  endif()
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  # using Intel C++
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-intel -wd10237 -Wall -ipo -fPIC")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
  set(BOOST_COMPONENTS ${BOOST_COMPONENTS} date_time chrono zlib)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DNOMINMAX) # avoid min and max macros that can break compilation
  add_definitions(-D_USE_MATH_DEFINES) # define M_PI
  add_definitions(-D_WIN32_WINNT=0x0501)
endif()

# disable partitioning of LTO process when possible (fixes Debian issues)
set(LTO_PARTITION_FLAGS "")
CHECK_CXX_COMPILER_FLAG("-flto-partition=none" HAS_LTO_PARTITION_FLAG)
if    (HAS_LTO_PARTITION_FLAG)
  set(LTO_PARTITION_FLAGS "${LTO_PARTITION_FLAGS} -flto-partition=none")
endif (HAS_LTO_PARTITION_FLAG)

# Add Link-Time-Optimization flags, if supported (GCC >= 4.7) and enabled
set(CMAKE_CXX_FLAGS           "${CMAKE_CXX_FLAGS}           ${LTO_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    ${LTO_FLAGS} ${LTO_PARTITION_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${LTO_FLAGS} ${LTO_PARTITION_FLAGS}")

# Activate C++11
if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  ADD_DEFINITIONS(-std=c++11)
endif()

# Configuring other platform dependencies
if(APPLE)
  set(CMAKE_OSX_ARCHITECTURES "x86_64")
  message(STATUS "Set Architecture to x64 on OS X")
  exec_program(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
  string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
  if(OSXLIBSTD)
    message(STATUS "linking against ${OSXLIBSTD}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=${OSXLIBSTD}")
  elseif(DARWIN_VERSION GREATER 12)
    message(STATUS "linking against libc++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(streetname-fixer rt)
endif()

#Check Boost
set(BOOST_MIN_VERSION "1.49.0")
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Fatal error: Boost (version >= 1.49.0) required.\n")
endif()
include_directories(${Boost_INCLUDE_DIRS})

target_link_libraries(streetname-fixer ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

find_package(Threads REQUIRED)
target_link_libraries(streetname-fixer ${CMAKE_THREAD_LIBS_INIT})

find_package(EXPAT REQUIRED)
include_directories(${EXPAT_INCLUDE_DIRS})
target_link_libraries(streetname-fixer ${EXPAT_LIBRARIES})

find_package( STXXL REQUIRED )
include_directories(${STXXL_INCLUDE_DIR})
target_link_libraries(streetname-fixer ${STXXL_LIBRARY})

find_package( OSMPBF REQUIRED )
include_directories(${OSMPBF_INCLUDE_DIR})
target_link_libraries(streetname-fixer ${OSMPBF_LIBRARY})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
target_link_libraries(streetname-fixer ${PROTOBUF_LIBRARY})

find_package(BZip2 REQUIRED)
include_directories(${BZIP_INCLUDE_DIRS})
target_link_libraries(streetname-fixer ${BZIP2_LIBRARIES})

find_package(ZLIB REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRS})
target_link_libraries(streetname-fixer ${ZLIB_LIBRARY})

