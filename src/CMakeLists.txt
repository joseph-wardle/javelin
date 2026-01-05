include_guard(GLOBAL)

add_library(javelin STATIC)
add_library(javelin::javelin ALIAS javelin)

target_compile_features(javelin PUBLIC cxx_std_23)

target_sources(javelin
    PUBLIC
        FILE_SET javelin_modules
        TYPE CXX_MODULES
        FILES
            core/app.cppm
            core/logging.cppm
            core/time.cppm
            core/types.cppm
            math/constants.cppm
            math/mat3.cppm
            math/mat4.cppm
            math/quat.cppm
            math/vec2.cppm
            math/vec3.cppm
            math/vec4.cppm
            physics/physics_system.cppm
            platform/platform.cppm
            platform/window.cppm
            render/render_system.cppm
            scene/scene.cppm
)


if(JAVELIN_ENABLE_TRACY)
    target_compile_definitions(javelin PUBLIC TRACY_ENABLE)
    target_link_libraries(javelin PRIVATE TracyClient)
endif()
