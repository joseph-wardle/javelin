cmake_minimum_required(VERSION 4.0)

if(CMAKE_VERSION VERSION_LESS 4.0.3)
    set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "a9e1cf81-9932-4810-974b-6eccaf14e457") # 4.0.0 - 4.0.2
elseif(CMAKE_VERSION VERSION_LESS 4.3)
    set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444") # 4.0.3 - 4.2.x
elseif(CMAKE_VERSION VERSION_LESS 4.4)
    set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "451f2fe2-a8a2-47c3-bc32-94786d8fc91b") # 4.3.x
else()
    message(FATAL_ERROR
        "Unsupported CMake version ${CMAKE_VERSION} for experimental `import std` setup. "
        "Update CMAKE_EXPERIMENTAL_CXX_IMPORT_STD in the top-level CMakeLists.txt."
    )
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPERIMENTAL_CXX_MODULE_DYNDEP 1)

option(JAVELIN_BUILD_EXAMPLES "Build example executables." ON)
option(JAVELIN_BUILD_BENCH    "Build benchmark executables." ON)
option(JAVELIN_ENABLE_TESTS   "Build unit/integration tests."  OFF)
option(JAVELIN_ENABLE_TRACY   "Enable Tracy instrumentation."  ON)

project(javelin VERSION 0.1.0 LANGUAGES CXX C)

function(javelin_enable_import_std target)
    # Scope `import std` support to Javelin targets so third-party targets such
    # as imgui do not inherit the requirement.
    set_property(TARGET ${target} PROPERTY CXX_MODULE_STD ON)
endfunction()

include(cmake/dependencies.cmake)
javelin_setup_dependencies()

add_subdirectory(src)

if(JAVELIN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(JAVELIN_BUILD_BENCH)
    add_subdirectory(bench)
endif()

if(JAVELIN_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
