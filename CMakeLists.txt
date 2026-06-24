cmake_minimum_required(VERSION 3.20)

project(sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

# --- shared library: all the reusable code ---
add_library(mini
    src/Mesh.cpp
    src/State.cpp
    src/PCGSolver.cpp
    src/Exporter.cpp
)

target_include_directories(mini
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mini
    PUBLIC
        Eigen3::Eigen
)

# --- the app ---
add_executable(sim
    src/main.cpp
)
target_link_libraries(sim PRIVATE mini)

# --- tests ---
enable_testing()

add_executable(test_pcg tests/test_pcg.cpp)
target_link_libraries(test_pcg PRIVATE mini)
#add_test(NAME pcg_all COMMAND test_pcg)
add_test(NAME pcg_identity COMMAND test_pcg identity)
add_test(NAME pcg_small_spd COMMAND test_pcg small_spd)
add_test(NAME pcg_random_50x50 COMMAND test_pcg random_50x50_spd)
add_test(NAME pcg_random_500x500 COMMAND test_pcg random_500x500_spd)

