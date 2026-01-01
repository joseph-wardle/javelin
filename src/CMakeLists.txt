include_guard(GLOBAL)

# Canonical source lists live here (and only here).
set(JAVELIN_MODULE_REL
        core/types.cppm
        math/constants.cppm
        math/mat3.cppm
        math/mat4.cppm
        math/quat.cppm
        math/vec2.cppm
        math/vec3.cppm
        math/vec4.cppm
)

set(JAVELIN_CPP_REL
        # later: core/foo.cpp
)

function(javelin_attach_sources target)
    set(src_dir "${CMAKE_CURRENT_LIST_DIR}")

    set(module_abs "")
    foreach(f IN LISTS JAVELIN_MODULE_REL)
        list(APPEND module_abs "${src_dir}/${f}")
    endforeach()

    set(cpp_abs "")
    foreach(f IN LISTS JAVELIN_CPP_REL)
        list(APPEND cpp_abs "${src_dir}/${f}")
    endforeach()

    target_sources(${target}
            PUBLIC
            FILE_SET javelin_modules
            TYPE CXX_MODULES
            BASE_DIRS "${src_dir}"
            FILES ${module_abs}
            PRIVATE
            ${cpp_abs}
    )

    # Helpful once you start having non-module includes or C deps:
    target_include_directories(${target} PUBLIC "${src_dir}")
endfunction()

add_library(javelin STATIC)
add_library(javelin::javelin ALIAS javelin)

javelin_attach_sources(javelin)

target_compile_features(javelin PUBLIC cxx_std_23)

target_link_libraries(javelin PRIVATE doctest::doctest)

if(NOT JAVELIN_BUILD_TESTS)
    target_compile_definitions(javelin PRIVATE DOCTEST_CONFIG_DISABLE)
endif()

if(JAVELIN_ENABLE_TRACY)
    target_compile_definitions(javelin PUBLIC TRACY_ENABLE)
    target_link_libraries(javelin PRIVATE TracyClient)
endif()
