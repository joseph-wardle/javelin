add_executable(javelin_physics_tests)
target_compile_features(javelin_physics_tests PRIVATE cxx_std_23)
target_sources(javelin_physics_tests
    PRIVATE
        main.cpp
    PUBLIC
        FILE_SET test_modules
        TYPE CXX_MODULES
        FILES
            support/assert.cppm
            support/runner.cppm
            suites/sphere_sphere_contact.cppm
            suites/box_box_sat.cppm
            suites/solve_contact_velocities.cppm
)
target_link_libraries(javelin_physics_tests PRIVATE javelin::javelin)

add_test(NAME sphere_sphere_contact COMMAND javelin_physics_tests sphere_sphere_contact)
add_test(NAME box_box_sat COMMAND javelin_physics_tests box_box_sat)
add_test(NAME solve_contact_velocities COMMAND javelin_physics_tests solve_contact_velocities)
