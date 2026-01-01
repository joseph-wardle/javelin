cmake_minimum_required(VERSION 4.0)

if(CMAKE_VERSION VERSION_LESS 4.0.3)
    set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "a9e1cf81-9932-4810-974b-6eccaf14e457") # 4.0.2
else()
    set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444") # 4.0.3+
endif()

set(CMAKE_CXX_MODULE_STD ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXPERIMENTAL_CXX_MODULE_DYNDEP 1)

option(JAVELIN_BUILD_EXAMPLES "Build example executables." ON)
option(JAVELIN_BUILD_TESTS    "Build unit/integration tests."  OFF)

option(JAVELIN_ENABLE_TRACY   "Enable Tracy instrumentation."  OFF)

project(javelin VERSION 0.1.0 LANGUAGES CXX C)

include(cmake/dependencies.cmake)
javelin_setup_dependencies()

add_subdirectory(src)

if(JAVELIN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(JAVELIN_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()