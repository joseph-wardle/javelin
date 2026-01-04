include_guard(GLOBAL)

add_library(javelin STATIC)
add_library(javelin::javelin ALIAS javelin)

target_compile_features(javelin PUBLIC cxx_std_23)

target_sources(javelin
    PUBLIC
        FILE_SET javelin_modules
        TYPE CXX_MODULES
        FILES
            core/logging.cppm
            core/types.cppm
            math/constants.cppm
            math/mat3.cppm
            math/mat4.cppm
            math/quat.cppm
            math/vec2.cppm
            math/vec3.cppm
            math/vec4.cppm
)


if(JAVELIN_ENABLE_TRACY)
    target_compile_definitions(javelin PUBLIC TRACY_ENABLE)
    target_link_libraries(javelin PRIVATE TracyClient)
endif()
