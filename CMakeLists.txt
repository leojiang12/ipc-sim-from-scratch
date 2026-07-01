# cmake -S . -B build -G Ninja      -      -S where -B build dir name -G build system

cmake_minimum_required(VERSION 3.20)

project(sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

# --- shared library: all the reusable code ---
add_library(sim_lib
    src/Mesh.cpp
    src/State.cpp
    src/DOFMap.cpp
    src/utils.cpp
    src/PCGSolver.cpp
    src/InertiaEnergy.cpp
    src/TimeIntegrator.cpp
    src/Exporter.cpp
)

target_include_directories(sim_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sim_lib
    PUBLIC
        Eigen3::Eigen
)

# --- the app ---
add_executable(sim
    src/main.cpp
)

target_link_libraries(sim PRIVATE sim_lib)

# --- tests ---     

# list the names first — always do this when unsure
#   ctest --test-dir build -N

# run exactly one (anchored so it can't match others)
#   ctest --test-dir build -R '^dofmap_apply_dirichlet$' --output-on-failure

# run a whole family (partial match — runs all 4 pcg_* tests)
#   ctest --test-dir build -R pcg
enable_testing()

# interface library for header-only test_utils.h
add_library(test_utilities INTERFACE)
target_include_directories(test_utilities INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/tests)


# pcg test
add_executable(test_pcg tests/test_pcg.cpp)
target_link_libraries(test_pcg PRIVATE sim_lib test_utilities)

add_test(NAME pcg_identity COMMAND test_pcg identitys)
add_test(NAME pcg_small_spd COMMAND test_pcg small_spd)
add_test(NAME pcg_random_50x50 COMMAND test_pcg random_50x50_spd)
add_test(NAME pcg_random_500x500 COMMAND test_pcg random_500x500_spd)

# dofmap test
add_executable(test_dofmap tests/test_dofmap.cpp)
target_link_libraries(test_dofmap PRIVATE sim_lib test_utilities)
add_test(NAME dofmap_apply_dirichlet COMMAND test_dofmap apply_dirichlet)

# util functions test
add_executable(test_utils tests/test_utils.cpp)
target_link_libraries(test_utils PRIVATE sim_lib test_utilities)
add_test(NAME utils_make_PSD COMMAND test_utils make_PSD)

# energy functions test
add_executable(test_energies tests/test_energies.cpp)
target_link_libraries(test_energies PRIVATE sim_lib test_utilities)
add_test(NAME inertia_energy COMMAND test_energies inertia)

# Compiler flags    (private, so flags are not pushed onto everything linking to sim_lib)
target_compile_options(sim_lib      PRIVATE -Wall -Wextra -Wshadow)
target_compile_options(sim       PRIVATE -Wall -Wextra -Wshadow)
target_compile_options(test_pcg  PRIVATE -Wall -Wextra -Wshadow)
target_compile_options(test_dofmap PRIVATE -Wall -Wextra -Wshadow)

