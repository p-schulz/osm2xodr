cmake_minimum_required(VERSION 3.20)
# C is only needed for external/tinyfiledialogs/tinyfiledialogs.c (a vendored C library, compiled
# into osm2xodr-gui when OSM2XODR_BUILD_GUI=ON); the CLI target and osm2xodr_core stay pure C++.
project(osm2xodr VERSION 0.1.0 LANGUAGES CXX C)

option(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION "Build optional libOpenDRIVE read-back validation" OFF)
option(OSM2XODR_BUILD_GUI "Build the experimental Dear ImGui-based GUI (osm2xodr-gui)" OFF)
set(OSM2XODR_LIBOPENDRIVE_SOURCE_DIR "" CACHE PATH "Optional path to a local libOpenDRIVE source tree")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Osmium REQUIRED COMPONENTS io)
find_package(Threads REQUIRED)

function(osm2xodr_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
        target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# Conversion pipeline (parsing, model building, XODR writing, reporting) shared by the CLI
# (src/main.cpp + src/cli.cpp) and the optional GUI (src/gui/*.cpp) -- an object library so it's
# compiled once and neither front end can drift from the other's copy of this logic.
add_library(osm2xodr_core OBJECT
    src/config.cpp
    src/infer.cpp
    src/model_builder.cpp
    src/osm_parse.cpp
    src/report.cpp
    src/xodr_writer.cpp
)
target_include_directories(osm2xodr_core PUBLIC include)
if(TARGET Osmium::Osmium)
    target_link_libraries(osm2xodr_core PUBLIC Osmium::Osmium Threads::Threads)
else()
    target_include_directories(osm2xodr_core SYSTEM PUBLIC ${OSMIUM_INCLUDE_DIRS})
    target_link_libraries(osm2xodr_core PUBLIC ${OSMIUM_LIBRARIES} Threads::Threads)
endif()
osm2xodr_set_warnings(osm2xodr_core)

add_executable(osm2xodr
    src/main.cpp
    src/cli.cpp
)
target_link_libraries(osm2xodr PRIVATE osm2xodr_core)
osm2xodr_set_warnings(osm2xodr)

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

# Experimental Dear ImGui GUI (osm2xodr-gui), off by default -- the CLI target above builds and
# behaves exactly as before regardless of this option. Vendored under external/imgui; SDL2 and
# OpenGL are expected to be available on the system (brew/apt package, or vcpkg on Windows --
# see vcpkg.json), same as any other native GUI toolkit, rather than vendored as source.
if(OSM2XODR_BUILD_GUI)
    find_package(SDL2 CONFIG QUIET)
    if(NOT TARGET SDL2::SDL2)
        find_package(SDL2 MODULE REQUIRED)
    endif()
    find_package(OpenGL REQUIRED)
    find_package(CURL REQUIRED)

    set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui")

    add_executable(osm2xodr-gui
        src/gui/main.cpp
        src/gui/gui_app.cpp
        src/gui/road_geometry.cpp
        src/gui/lgl_imagery.cpp
        "${IMGUI_DIR}/imgui.cpp"
        "${IMGUI_DIR}/imgui_draw.cpp"
        "${IMGUI_DIR}/imgui_tables.cpp"
        "${IMGUI_DIR}/imgui_widgets.cpp"
        "${IMGUI_DIR}/imgui_demo.cpp"
        "${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp"
        "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/external/tinyfiledialogs/tinyfiledialogs.c"
    )
    target_include_directories(osm2xodr-gui PRIVATE
        "${IMGUI_DIR}" "${IMGUI_DIR}/backends"
        "${CMAKE_CURRENT_SOURCE_DIR}/external/stb"
        "${CMAKE_CURRENT_SOURCE_DIR}/external/tinyfiledialogs"
    )
    target_link_libraries(osm2xodr-gui PRIVATE osm2xodr_core OpenGL::GL CURL::libcurl)
    if(APPLE)
        # macOS deprecated the whole legacy (non-Metal) OpenGL API in 10.14; still fully
        # functional (and what ImGui's own OpenGL3 backend uses), just noisy without this.
        target_compile_definitions(osm2xodr-gui PRIVATE GL_SILENCE_DEPRECATION)
    endif()
    if(WIN32)
        # tinyfiledialogs' native Windows dialogs (GetOpenFileNameW/GetSaveFileNameW/
        # SHBrowseForFolderW) live in these; not always pulled in by MSVC's default libs.
        target_link_libraries(osm2xodr-gui PRIVATE comdlg32 ole32 shell32)
    endif()

    if(TARGET SDL2::SDL2)
        target_link_libraries(osm2xodr-gui PRIVATE SDL2::SDL2)
        if(TARGET SDL2::SDL2main)
            target_link_libraries(osm2xodr-gui PRIVATE SDL2::SDL2main)
        endif()
    else()
        target_include_directories(osm2xodr-gui SYSTEM PRIVATE ${SDL2_INCLUDE_DIRS})
        target_link_libraries(osm2xodr-gui PRIVATE ${SDL2_LIBRARIES})
    endif()

    # Not using osm2xodr_set_warnings() here: it would also apply to the vendored ImGui/stb_image
    # sources compiled directly into this target, which aren't written to satisfy our warning flags.
    install(TARGETS osm2xodr-gui RUNTIME DESTINATION bin)
endif()

install(TARGETS osm2xodr RUNTIME DESTINATION bin)
