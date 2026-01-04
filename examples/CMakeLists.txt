function(javelin_add_example_from_dir dir)
    get_filename_component(name "${dir}" NAME)
    set(target "javelin_${name}")

    add_executable(${target})
    target_sources(${target} PRIVATE "${dir}/main.cpp")
    target_link_libraries(${target} PRIVATE javelin::javelin)

    target_link_libraries(${target} PRIVATE glfw glad::glad)
endfunction()

file(GLOB _example_mains CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*/main.cpp"
)

foreach(main_cpp IN LISTS _example_mains)
    get_filename_component(dir "${main_cpp}" DIRECTORY)
    javelin_add_example_from_dir("${dir}")
endforeach()
