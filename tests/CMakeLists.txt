add_executable(javelin_unit_tests doctest_main.cpp)
target_link_libraries(javelin_unit_tests PRIVATE javelin::javelin)

add_test(NAME javelin.unit COMMAND javelin_unit_tests)

#add_subdirectory(integration)
