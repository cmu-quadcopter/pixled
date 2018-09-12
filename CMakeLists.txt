cmake_minimum_required(VERSION 2.8)
project(pixled)

set(CMAKE_CXX_STANDARD 14)

if(MSVC)
    set(dronecore_lib "${CMAKE_SOURCE_DIR}/external/DronecodeSDK/lib/dronecore.lib")
elseif(APPLE)
    set(dronecore_lib
        "${CMAKE_SOURCE_DIR}/external/DronecodeSDK/lib/libdronecore.a"
        "${CMAKE_SOURCE_DIR}/external/DronecodeSDK/lib/libdronecore_telemetry.dylib"
        )
else()
    set(dronecore_lib
        "${CMAKE_SOURCE_DIR}/external/DronecodeSDK/lib/libdronecore.so"
        "${CMAKE_SOURCE_DIR}/external/DronecodeSDK/lib/libdronecore_telemetry.so")
endif()

add_executable(pixled main.cpp)

target_include_directories(pixled PUBLIC ${CMAKE_SOURCE_DIR}/external/DronecodeSDK/include)

target_link_libraries(pixled ${dronecore_lib})
