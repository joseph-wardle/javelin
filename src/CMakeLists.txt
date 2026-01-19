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
            platform/input.cppm
            platform/platform.cppm
            platform/window.cppm
            render/fly_camera.cppm
            render/color.cppm
            render/pipeline.cppm
            render/render_context.cppm
            render/render_device.cppm
            render/render_targets.cppm
            render/render_system.cppm
            render/types.cppm
            render/passes/geometry_pass.cppm
            render/passes/display_pass.cppm
            render/passes/world_grid_pass.cppm
            scene/camera.cppm
            scene/entity.cppm
            scene/physics_view.cppm
            scene/pose_channel.cppm
            scene/render_view.cppm
            scene/scene.cppm
            scene/shapes.cppm
)


if(JAVELIN_ENABLE_TRACY)
    target_compile_definitions(javelin PUBLIC TRACY_ENABLE)
    target_link_libraries(javelin PRIVATE TracyClient)
endif()

target_link_libraries(javelin PRIVATE glad glfw imgui)
