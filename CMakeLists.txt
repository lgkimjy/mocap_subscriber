cmake_minimum_required(VERSION 3.16)

project(
    mocap_subscriber
    VERSION 0.1.0
    DESCRIPTION "Reusable NatNet mocap subscriber for OptiTrack/Motive"
    LANGUAGES CXX
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(mocap_natnet STATIC
    include/MoCapParser/NatNetClient.cpp
)

target_include_directories(mocap_natnet
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mocap_natnet
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        Threads::Threads
)

target_compile_options(mocap_natnet PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

add_executable(mocap_reader
    src/mocap_reader_main.cpp
)

target_link_libraries(mocap_reader PRIVATE mocap_natnet)

install(TARGETS mocap_natnet mocap_reader
    EXPORT mocap_subscriberTargets
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
