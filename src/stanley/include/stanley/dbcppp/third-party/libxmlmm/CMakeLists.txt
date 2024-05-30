
cmake_minimum_required(VERSION 3.12)

project(libxmlmm VERSION 1.0.0)

set(CMAKE_STATIC_LIBRARY_PREFIX "")
set(CMAKE_SHARED_LIBRARY_PREFIX "")

file(GLOB header
    "libxmlmm/*.h"
)
file(GLOB src
    "libxmlmm/*.cpp"
)

find_package(LibXml2 REQUIRED)

include_directories(${LIBXML2_INCLUDE_DIR})

add_library(libxmlmm SHARED "")
target_link_libraries(libxmlmm ${LIBXML2_LIBRARIES})

target_sources("libxmlmm"
    PRIVATE ${header}
    PRIVATE ${src}
)

install(
    TARGETS "libxmlmm"
    DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(
    DIRECTORY "libxmlmm"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libxmlmm
    FILES_MATCHING PATTERN "*.h")