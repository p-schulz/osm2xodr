cmake_minimum_required(VERSION 3.20)
project(osm2xodr VERSION 0.1.0 LANGUAGES CXX)

option(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION "Build optional libOpenDRIVE read-back validation" OFF)
set(OSM2XODR_LIBOPENDRIVE_SOURCE_DIR "" CACHE PATH "Optional path to a local libOpenDRIVE source tree")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Osmium REQUIRED COMPONENTS io)
find_package(Threads REQUIRED)

add_executable(osm2xodr
    src/main.cpp
    src/cli.cpp
    src/infer.cpp
    src/model_builder.cpp
    src/osm_parse.cpp
    src/report.cpp
    src/xodr_writer.cpp
)

target_include_directories(osm2xodr PRIVATE include)

if(TARGET Osmium::Osmium)
    target_link_libraries(osm2xodr PRIVATE Osmium::Osmium Threads::Threads)
else()
    target_include_directories(osm2xodr SYSTEM PRIVATE ${OSMIUM_INCLUDE_DIRS})
    target_link_libraries(osm2xodr PRIVATE ${OSMIUM_LIBRARIES} Threads::Threads)
endif()

if(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION)
    set(OPENDRIVE_BUILD_TESTS OFF CACHE BOOL "" FORCE)

    if(OSM2XODR_LIBOPENDRIVE_SOURCE_DIR)
        if(NOT EXISTS "${OSM2XODR_LIBOPENDRIVE_SOURCE_DIR}/CMakeLists.txt")
            message(FATAL_ERROR "OSM2XODR_LIBOPENDRIVE_SOURCE_DIR does not contain a CMakeLists.txt: ${OSM2XODR_LIBOPENDRIVE_SOURCE_DIR}")
        endif()
        add_subdirectory("${OSM2XODR_LIBOPENDRIVE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/libOpenDRIVE")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libOpenDRIVE/CMakeLists.txt")
        add_subdirectory(external/libOpenDRIVE)
    else()
        include(FetchContent)
        FetchContent_Declare(
            libOpenDRIVE
            GIT_REPOSITORY https://github.com/pageldev/libOpenDRIVE.git
            GIT_TAG main
        )
        FetchContent_MakeAvailable(libOpenDRIVE)
    endif()

    target_compile_definitions(osm2xodr PRIVATE OSM2XODR_WITH_LIBOPENDRIVE=1)
    target_link_libraries(osm2xodr PRIVATE OpenDrive::OpenDrive)
endif()

if(MSVC)
    target_compile_options(osm2xodr PRIVATE /W4 /permissive-)
    target_compile_definitions(osm2xodr PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    target_compile_options(osm2xodr PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS osm2xodr RUNTIME DESTINATION bin)
