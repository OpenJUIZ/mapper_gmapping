add_executable(core_test core_test.cpp)
target_link_libraries(core_test PRIVATE mapper_gmapping_core)
add_test(NAME core_test COMMAND core_test)
