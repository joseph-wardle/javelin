add_library(javelin_bench_support STATIC)
target_compile_features(javelin_bench_support PUBLIC cxx_std_23)
javelin_enable_import_std(javelin_bench_support)
target_sources(javelin_bench_support
    PUBLIC
        FILE_SET bench_support_modules
        TYPE CXX_MODULES
        FILES
            support/cli.cppm
            support/json.cppm
            support/stats.cppm
            support/timer.cppm
)
target_link_libraries(javelin_bench_support PUBLIC javelin::javelin)

function(javelin_add_bench_from_dir dir)
    get_filename_component(name "${dir}" NAME)
    set(target "javelin_bench_${name}")

    add_executable(${target})
    target_sources(${target} PRIVATE "${dir}/main.cpp")
    javelin_enable_import_std(${target})

    target_link_libraries(${target}
            PRIVATE
            javelin::javelin
            javelin_bench_support
    )
endfunction()


file(GLOB _bench_mains CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*/main.cpp"
)

foreach(main_cpp IN LISTS _bench_mains)
    get_filename_component(dir "${main_cpp}" DIRECTORY)
    javelin_add_bench_from_dir("${dir}")
endforeach()
