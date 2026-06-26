# cmake -S . -B build -G Ninja      -      -S where -B build dir name -G build system

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
    src/DOFMap.cpp
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

# --- tests ---     use  ctest --test-dir build/ --output-on-failure   for all
#                    or  ctest -R exe_name --test-dir build/ --out-on-failure   for a specific test
enable_testing()

# pcg test
add_executable(test_pcg tests/test_pcg.cpp)
target_link_libraries(test_pcg PRIVATE mini)
add_test(NAME pcg_identity COMMAND test_pcg identity)
add_test(NAME pcg_small_spd COMMAND test_pcg small_spd)
add_test(NAME pcg_random_50x50 COMMAND test_pcg random_50x50_spd)
add_test(NAME pcg_random_500x500 COMMAND test_pcg random_500x500_spd)

# dofmap test
add_executable(test_dofmap tests/test_dofmap.cpp)
target_link_libraries(test_dofmap PRIVATE mini)
add_test(NAME dofmap_apply_dirichlet COMMAND test_dofmap apply_dirichlet)
