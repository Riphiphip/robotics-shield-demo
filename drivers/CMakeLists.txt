cmake_minimum_required(VERSION 3.20.0)

target_include_directories(app PRIVATE .)
add_subdirectory(motors)
add_subdirectory(stspin240)
add_subdirectory(tb6612fng)
add_subdirectory(ls7184n)