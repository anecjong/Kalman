cmake_minimum_required(VERSION 3.10)
project(easy-kalman)

add_subdirectory(thirdparty)
add_subdirectory(filters)

add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC eigen matplotlib_cpp filters)
target_include_directories(main PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matplotlib_cpp
    )