# Minimal FindOsmium.cmake for osm2xodr.
#
# libosmium is header-only, but its IO code requires external libraries.
# This finder creates an Osmium::Osmium interface target with the include
# directories and common IO libraries needed by this converter.

find_path(OSMIUM_INCLUDE_DIR
    NAMES osmium/version.hpp osmium/io/any_input.hpp
    PATH_SUFFIXES include
)

find_path(PROTOZERO_INCLUDE_DIR
    NAMES protozero/version.hpp protozero/types.hpp
    PATH_SUFFIXES include
)

find_package(ZLIB QUIET)
find_package(BZip2 QUIET)
find_package(EXPAT QUIET)
find_package(Threads QUIET)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Osmium
    REQUIRED_VARS
        OSMIUM_INCLUDE_DIR
        PROTOZERO_INCLUDE_DIR
        ZLIB_FOUND
        BZIP2_FOUND
        EXPAT_FOUND
        Threads_FOUND
)

if(Osmium_FOUND AND NOT TARGET Osmium::Osmium)
    add_library(Osmium::Osmium INTERFACE IMPORTED)
    set_target_properties(Osmium::Osmium PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OSMIUM_INCLUDE_DIR};${PROTOZERO_INCLUDE_DIR}"
    )

    target_link_libraries(Osmium::Osmium INTERFACE
        ZLIB::ZLIB
        BZip2::BZip2
        EXPAT::EXPAT
        Threads::Threads
    )
endif()

set(OSMIUM_INCLUDE_DIRS "${OSMIUM_INCLUDE_DIR};${PROTOZERO_INCLUDE_DIR}")
set(OSMIUM_LIBRARIES ZLIB::ZLIB BZip2::BZip2 EXPAT::EXPAT Threads::Threads)
