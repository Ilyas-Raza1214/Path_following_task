cmake_minimum_required(VERSION 3.22)

project(prog_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(
    path_follower
    src/Test.cpp
    src/PathFollower.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        Eigen3::Eigen
)