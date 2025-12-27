add_library(javelin STATIC)

target_sources(javelin
    PUBLIC
    FILE_SET javelin_modules
        TYPE CXX_MODULES
        BASE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            core/types.cppm
            math/vec2.cppm
            math/vec3.cppm
            math/vec4.cppm
)