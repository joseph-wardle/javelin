add_library(glad STATIC)
add_library(glad::glad ALIAS glad)

target_sources(glad PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/gl.c
)

target_include_directories(glad
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set_target_properties(glad PROPERTIES
        POSITION_INDEPENDENT_CODE ON
)
