include_guard(GLOBAL)
include(${CMAKE_CURRENT_LIST_DIR}/cpm.cmake)

function(javelin_setup_dependencies)
    CPMAddPackage(
            NAME doctest
            GITHUB_REPOSITORY doctest/doctest
            VERSION 2.4.12
            OPTIONS
            "DOCTEST_WITH_MAIN OFF"
            "DOCTEST_NO_INSTALL ON"
    )

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

    CPMAddPackage(
            NAME tracy
            GITHUB_REPOSITORY wolfpld/tracy
            VERSION 0.13.1
    )

    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/glad)
endfunction()
