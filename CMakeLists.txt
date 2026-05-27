cmake_minimum_required(VERSION 3.16)
project(MightMavlink CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
  mavlink_c
  GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
  GIT_TAG master
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(mavlink_c)
if(NOT mavlink_c_POPULATED)
  FetchContent_Populate(mavlink_c)
endif()

add_library(might_mavlink STATIC
  src/MightMavlink.cpp
  src/SerialPort.cpp
)
target_include_directories(might_mavlink
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${mavlink_c_SOURCE_DIR}
    ${mavlink_c_SOURCE_DIR}/common
)
target_compile_options(might_mavlink PRIVATE
  -Wall -Wextra -Wno-address-of-packed-member -Wno-cast-align
)
find_package(Threads REQUIRED)
target_link_libraries(might_mavlink PUBLIC Threads::Threads)

add_executable(board_mock examples/board_mock.cpp)
target_link_libraries(board_mock PRIVATE might_mavlink)

add_executable(loopback_test examples/loopback_test.cpp)
target_link_libraries(loopback_test PRIVATE might_mavlink)
target_include_directories(loopback_test PRIVATE
  ${mavlink_c_SOURCE_DIR}
  ${mavlink_c_SOURCE_DIR}/common
)
target_compile_options(loopback_test PRIVATE -Wno-address-of-packed-member -Wno-cast-align)
enable_testing()
add_test(NAME loopback COMMAND loopback_test)
