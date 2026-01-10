include_guard(GLOBAL)
include(${CMAKE_CURRENT_LIST_DIR}/cpm.cmake)

function(javelin_setup_dependencies)
    # Define glad target first so other targets can link it as a target.
    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/glad)

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

    find_package(imgui CONFIG QUIET)
    if (NOT TARGET imgui AND NOT TARGET imgui::imgui)
        CPMAddPackage(
                NAME imgui
                GITHUB_REPOSITORY ocornut/imgui
                GIT_TAG v1.92.5
                DOWNLOAD_ONLY YES
        )

        if (imgui_ADDED)
            add_library(imgui STATIC
                    ${imgui_SOURCE_DIR}/imgui.cpp
                    ${imgui_SOURCE_DIR}/imgui_demo.cpp
                    ${imgui_SOURCE_DIR}/imgui_draw.cpp
                    ${imgui_SOURCE_DIR}/imgui_tables.cpp
                    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
                    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
                    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
            )
            add_library(imgui::imgui ALIAS imgui)

            target_include_directories(imgui
                    SYSTEM PUBLIC
                    ${imgui_SOURCE_DIR}
                    ${imgui_SOURCE_DIR}/backends
            )

            target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)

            find_package(OpenGL REQUIRED)
            target_link_libraries(imgui
                    PUBLIC
                    glfw
                    glad::glad
                    OpenGL::GL
            )
        endif()
    endif()
endfunction()
