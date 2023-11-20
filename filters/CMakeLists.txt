cmake_minimum_required(VERSION 3.10)
project(lib-filters)

set(FILTER_SRC_FILES
    src/simplefilters.cpp
    src/LKF.cpp
)

set(FILTER_INCLUDE_DIRS
    include
)

set(FILTER_LINK_LIBS
    eigen
)

add_library(filters ${FILTER_SRC_FILES})
target_include_directories(filters PUBLIC ${FILTER_INCLUDE_DIRS})
target_link_libraries(filters PUBLIC ${FILTER_LINK_LIBS})
