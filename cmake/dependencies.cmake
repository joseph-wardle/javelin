include_guard(GLOBAL)
include(${CMAKE_CURRENT_LIST_DIR}/cpm.cmake)

function(javelin_setup_dependencies)
    find_package(glfw3 CONFIG QUIET)

    if (NOT TARGET glfw)
        if (TARGET glfw::glfw)
            add_library(glfw ALIAS glfw::glfw)
        else()
            CPMAddPackage(
                    NAME glfw
                    GITHUB_REPOSITORY glfw/glfw
                    GIT_TAG 3.4
                    OPTIONS
                    "GLFW_BUILD_DOCS OFF"
                    "GLFW_BUILD_TESTS OFF"
                    "GLFW_BUILD_EXAMPLES OFF"
                    "GLFW_INSTALL OFF"
            )
        endif()
    endif()

    find_package(tracy CONFIG QUIET)
    if (NOT TARGET TracyClient AND NOT TARGET Tracy::TracyClient AND NOT TARGET tracy::TracyClient)
        CPMAddPackage(
                NAME tracy
                GITHUB_REPOSITORY wolfpld/tracy
                VERSION 0.13.1
        )
    endif()

    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/glad)
endfunction()
